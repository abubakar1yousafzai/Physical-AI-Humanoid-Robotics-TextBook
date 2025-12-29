---
id: 013
title: fix-tool-registration-error
stage: green
date: 2025-12-27
surface: agent
model: gemini
feature: rag-backend-api
branch: 003-rag-backend-api
user: user
command: fix tool registration error
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

Fix tool registration error in rag.py: AttributeError: 'function' object has no attribute 'name'. Wrap search_textbook_tool in a Tool object.

## Response snapshot

Fixed `AttributeError` by importing the `Tool` class and wrapping the `search_textbook_tool` function within a `Tool` object before passing it to the `Agent`.

## Outcome

- ✅ Impact: Agent now correctly registers and identifies tools.
- 🧪 Tests: N/A
- 📁 Files: backend/app/services/rag.py
- 🔁 Next prompts: /sp.implement Phase 5
- 🧠 Reflection: The OpenAI Agents SDK requires tools to be defined as `Tool` objects rather than raw functions to ensure metadata like `name` and `description` are properly attached.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
