import cohere
import uuid
import json
from typing import List, Dict, Any
from agents import Agent, Runner, AsyncOpenAI, OpenAIChatCompletionsModel
from agents.run import RunConfig
from app.core.config import settings
from app.services.qdrant import QdrantService
from app.models.api import ChatRequest, ChatResponse

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

def search_textbook_tool(query: str) -> str:
    """Search Physical AI and Humanoid Robotics textbook for relevant information.
    
    Args:
        query: Search query to find relevant textbook content
        
    Returns:
        Formatted context from the textbook
    """
    context_chunks = search_context(query, limit=3)
    return "\n\n".join(context_chunks)

def process_chat(request: ChatRequest) -> ChatResponse:
    """Process query with manual RAG (no agent tools)"""
    # 1. Search textbook
    context_chunks = search_context(request.message, limit=3)
    context_str = "\n\n".join(context_chunks)
    
    # 2. Create agent with context in instructions
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
    
    # Agent with embedded context (NO tools!)
    agent = Agent(
        name="Textbook Assistant",
        instructions=f"""You are a helpful assistant for the Physical AI and Humanoid Robotics Textbook.

Context from textbook:
{context_str}

Use the context above to answer the user's question accurately. If the answer is not in the context, acknowledge this.""",
        model=model
    )
    
    # 3. Run agent
    messages = [{"role": "user", "content": request.message}]
    result = Runner.run_sync(
        starting_agent=agent,
        input=messages,
        run_config=config
    )
    
    # 4. Return with sources
    return ChatResponse(
        answer=result.final_output,
        sources=context_chunks,
        thread_id=request.thread_id or uuid.uuid4()
    )