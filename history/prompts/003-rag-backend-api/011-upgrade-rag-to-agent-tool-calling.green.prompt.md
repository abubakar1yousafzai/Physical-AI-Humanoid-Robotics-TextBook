---
id: 011
title: upgrade-rag-to-agent-tool-calling
stage: green
date: 2025-12-27
surface: agent
model: gemini
feature: rag-backend-api
branch: 003-rag-backend-api
user: user
command: upgrade rag.py
labels: ["refactor", "rag", "agent", "tool-calling", "openai-sdk"]
links:
  spec: specs/003-rag-backend-api/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - backend/app/services/rag.py
tests: []
---

## Prompt

Upgrade rag.py to use OpenAI Agents SDK with tool calling. Agent should decide when to search Qdrant.

## Response snapshot

Refactored `rag.py` to use the OpenAI SDK's tool-calling pattern. Defined `search_textbook` tool. Updated `process_chat` to handle tool call loops and multi-turn messaging.

## Outcome

- ✅ Impact: Satisfied FR-003 and aligned with constitutional agent requirements.
- 🧪 Tests: N/A
- 📁 Files: backend/app/services/rag.py
- 🔁 Next prompts: /sp.implement Phase 5
- 🧠 Reflection: Moving to an agentic approach allows the LLM to handle conversational queries and decides if the context is necessary, improving response relevance.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
