---
id: 015
title: simplify-rag-to-manual-context-injection
stage: refactor
date: 2025-12-27
surface: agent
model: gemini
feature: rag-backend-api
branch: 003-rag-backend-api
user: user
command: simplify rag.py to manual rag
labels: ["refactor", "rag", "agent", "manual-rag"]
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

Simplify rag.py to use manual RAG without agent tools. Retrieve context first and inject into agent instructions.

## Response snapshot

Refactored `process_chat` to perform Qdrant search manually before initializing the agent. The retrieved context is now embedded directly into the agent's instructions, removing the need for tool registration and handling.

## Outcome

- ✅ Impact: Eliminated tool registration errors and simplified the RAG flow.
- 🧪 Tests: N/A
- 📁 Files: backend/app/services/rag.py
- 🔁 Next prompts: /sp.implement Phase 5
- 🧠 Reflection: While agentic tool calling is powerful, manual RAG is often more robust for simple retrieve-then-answer workflows and avoids complexity with function schema validation.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
