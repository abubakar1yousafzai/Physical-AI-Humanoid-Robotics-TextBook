from typing import List, Optional
from uuid import uuid4
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from app.models.sql import Conversation, Message

import uuid

async def create_conversation(db: AsyncSession, user_id: Optional[uuid.UUID] = None) -> Conversation:
    """Create a new conversation with a unique ID."""
    conversation_id = str(uuid.uuid4())
    db_conversation = Conversation(id=conversation_id, user_id=user_id)
    db.add(db_conversation)
    await db.commit()
    await db.refresh(db_conversation)
    return db_conversation

async def get_conversation(db: AsyncSession, conversation_id: str) -> Optional[Conversation]:
    """Retrieve a conversation by ID."""
    result = await db.execute(select(Conversation).where(Conversation.id == conversation_id))
    return result.scalar_one_or_none()

async def get_user_conversations(db: AsyncSession, user_id: uuid.UUID) -> List[Conversation]:
    """Retrieve all conversations for a specific user."""
    result = await db.execute(
        select(Conversation)
        .where(Conversation.user_id == user_id)
        .order_by(Conversation.created_at.desc())
    )
    return list(result.scalars().all())

async def add_message(
    db: AsyncSession, conversation_id: str, role: str, content: str
) -> Message:
    """Add a message to a conversation."""
    db_message = Message(
        conversation_id=conversation_id,
        role=role,
        content=content
    )
    db.add(db_message)
    await db.commit()
    await db.refresh(db_message)
    return db_message

async def get_messages(db: AsyncSession, conversation_id: str) -> List[Message]:
    """Retrieve all messages for a conversation, ordered by creation time."""
    result = await db.execute(
        select(Message)
        .where(Message.conversation_id == conversation_id)
        .order_by(Message.created_at.asc())
    )
    return list(result.scalars().all())
