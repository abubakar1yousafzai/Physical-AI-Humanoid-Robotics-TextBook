---
id: 014
title: fix-tool-instantiation-error
stage: misc
date: 2025-12-27
surface: agent
model: gemini
feature: rag-backend-api
branch: 003-rag-backend-api
user: user
command: fix tool instantiation error
labels: ["fix", "rag", "agent", "tool-calling"]
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

Fix Tool import error in rag.py: TypeError: Cannot instantiate typing.Union. Remove Tool import and pass function directly.

## Response snapshot

Removed `Tool` from imports and removed the `Tool` wrapper in `create_textbook_agent`. The `search_textbook_tool` function is now passed directly to the `Agent`'s `tools` list.

## Outcome

- ✅ Impact: Fixed runtime error and simplified tool registration.
- 🧪 Tests: N/A
- 📁 Files: backend/app/services/rag.py
- 🔁 Next prompts: /sp.implement Phase 5
- 🧠 Reflection: The `openai-agents` library handles function introspection automatically, and `Tool` is a type hint (Union) rather than an instantiable class in this context.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
