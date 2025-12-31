import cohere
import uuid
import json
from typing import List, Dict, Any, Optional
from sqlalchemy.ext.asyncio import AsyncSession
from agents import Agent, Runner, AsyncOpenAI, OpenAIChatCompletionsModel
from agents.run import RunConfig
from app.core.config import settings
from app.services.qdrant import QdrantService
from app.models.api import ChatRequest, ChatResponse
from app.crud import chat as chat_crud

# Initialize Cohere client
co = cohere.Client(settings.COHERE_API_KEY)

def get_embedding(text: str) -> List[float]:
    """Generate vector embedding for a given text using Cohere"""
    response = co.embed(
        texts=[text],
        model="embed-english-v3.0",
        input_type="search_query"
    )
    return response.embeddings[0]

def search_context(query: str, limit: int = 3) -> List[str]:
    """Retrieve relevant context chunks from Qdrant"""
    vector = get_embedding(query)
    client = QdrantService.get_client()
    collection_name = QdrantService.get_collection_name()
    
    results = client.query_points(
        collection_name=collection_name,
        query=vector,
        limit=limit
    ).points
    
    # Extract text content. Prioritize 'text', fall back to string dump of payload
    return [hit.payload.get('text', str(hit.payload)) for hit in results]

async def process_chat(request: ChatRequest, db: AsyncSession) -> ChatResponse:
    """Process query with RAG and persistence"""
    
    # 1. Handle Conversation/Thread
    if request.thread_id:
        thread_id_str = str(request.thread_id)
        conversation = await chat_crud.get_conversation(db, thread_id_str)
        if not conversation:
             # Create new if not found
             conversation = await chat_crud.create_conversation(db)
             thread_id_str = conversation.id
    else:
        conversation = await chat_crud.create_conversation(db)
        thread_id_str = conversation.id

    # 2. Save User Message
    await chat_crud.add_message(db, thread_id_str, "user", request.message)

    # 3. Get History for Context
    history_messages = await chat_crud.get_messages(db, thread_id_str)
    
    # Prepare messages for agent
    agent_messages = []
    for m in history_messages:
        role = "assistant" if m.role in ["model", "assistant"] else "user"
        agent_messages.append({"role": role, "content": m.content})
    
    # 4. Search textbook (RAG) - using the current query
    context_chunks = search_context(request.message, limit=3)
    context_str = "\n\n".join(context_chunks)
    
    # 5. Create agent with context
    external_client = AsyncOpenAI(
        api_key=settings.GEMINI_API_KEY,
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
    )
    
    model = OpenAIChatCompletionsModel(
        model="gemini-2.5-flash",
        openai_client=external_client
    )
    
    config = RunConfig(
        model=model,
        model_provider=external_client,
        tracing_disabled=True
    )
    
    # Agent with embedded context
    agent = Agent(
        name="Textbook Assistant",
        instructions=f"""You are a helpful assistant for the Physical AI and Humanoid Robotics Textbook.

Context from textbook:
{context_str}

Use the context above to answer the user's question accurately. If the answer is not in the context, acknowledge this.""",
        model=model
    )
    
    # 6. Run agent
    result = await Runner.run(
        starting_agent=agent,
        input=agent_messages,
        run_config=config
    )
    
    final_answer = result.final_output

    # 7. Save Assistant Response
    await chat_crud.add_message(db, thread_id_str, "assistant", final_answer)
    
    return ChatResponse(
        answer=final_answer,
        sources=context_chunks,
        thread_id=thread_id_str  
    )
