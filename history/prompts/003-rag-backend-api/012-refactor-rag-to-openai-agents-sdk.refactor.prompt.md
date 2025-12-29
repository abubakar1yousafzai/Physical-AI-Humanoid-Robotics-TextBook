---
id: 012
title: refactor-rag-to-openai-agents-sdk
stage: refactor
date: 2025-12-27
surface: agent
model: gemini
feature: rag-backend-api
branch: 003-rag-backend-api
user: user
command: refactor rag.py to openai-agents
labels: ["refactor", "rag", "openai-agents", "gemini", "agent-sdk"]
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

Refactor rag.py to use openai-agents library with Google Gemini. Keep search_context and get_embedding. Use Runner.run_sync.

## Response snapshot

Refactored `rag.py` to use `openai-agents` library. Created `search_textbook_tool` and `create_textbook_agent`. Updated `process_chat` to use `Runner.run_sync` and extract sources from tool history.

## Outcome

- ✅ Impact: Modernized agent architecture using true SDK abstraction.
- 🧪 Tests: N/A
- 📁 Files: backend/app/services/rag.py
- 🔁 Next prompts: /sp.implement Phase 5
- 🧠 Reflection: The `openai-agents` library simplifies the orchestration logic significantly compared to manual tool calling loops. It also handles conversation state more robustly.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
