---
id: 2
title: Update Phase 2 RAG Chatbot Integration
stage: constitution
date: 2025-12-23
surface: agent
model: gemini-2.0-flash-thinking-exp-1218
feature: none
branch: 001-create-docusaurus-book
user: INS-PIRE
command: update phase 2 in constitution.md
labels: ["constitution", "rag", "chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - null
---

## Prompt

update phase 2 in constitution.md
Phase 2 - RAG Chatbot Integration:

Technology Stack:
- AI/LLM: OpenAI Agents SDK (Assistants API)
- Backend: FastAPI (Python)
- Database: Neon Serverless Postgres (conversation history)
- Vector Database: Qdrant Cloud Free Tier (textbook embeddings)
- Embeddings: OpenAI text-embedding-3-small
- Chat Interface: React component in Docusaurus

Architecture:
1. Indexing: Book content → Embeddings → Qdrant DB
2. Query Flow: User → FastAPI → Qdrant (retrieve) → OpenAI Agent → Response
3. History: FastAPI → Neon Postgres (save conversations)

Key Features:
1. General chatbot for textbook questions
2. Text selection-based queries:
   - User highlights text in chapter
   - Floating button appears near selection
   - Click to ask question about selected text
3. Floating chat widget (bottom-right corner, all pages)
4. Conversation history tracking in Neon Postgres
5. Anonymous chat (without login) with limited history
6. Context-aware responses using RAG pipeline

Technical Implementation:
- Qdrant stores all book content as vector embeddings
- FastAPI orchestrates: Frontend ↔ Qdrant ↔ OpenAI Agent ↔ Neon DB
- Text selection: window.getSelection() + floating button UI
- Chat widget: Minimizable popup (bottom-right)
- Session storage: Browser (anonymous) or Neon DB (logged in)

## Response snapshot

I will read the `.specify/memory/constitution.md` file to locate the Phase 2 section and update it with the new specifications.
Successfully modified file: D:\Hackaton-Projects\Hackaton-Book\.specify\memory\constitution.md (1 replacements).

## Outcome

- ✅ Impact: Updated the project constitution with detailed requirements for the RAG chatbot phase.
- 🧪 Tests: No tests run; manual verification of file content.
- 📁 Files: Modified `.specify/memory/constitution.md`.
- 🔁 Next prompts: None.
- 🧠 Reflection: The update provides a much clearer technical roadmap for Phase 2.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
