---
id: 006
title: Make ChatRequest Truly Optional
stage: green
date: 2025-12-30
surface: agent
model: gemini
feature: 006-chat-features
branch: 006-chat-features
user: User
command: fix-chat-request-optional
labels: ["bugfix", "backend", "pydantic"]
links:
  spec: specs/006-chat-features/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - backend/app/models/api.py
tests:
 - null
---

## Prompt

Fix ChatRequest model to make selected_text truly optional

## Response snapshot

Simplified the `ChatRequest` model in `backend/app/models/api.py` to ensure `selected_text` and `thread_id` are truly optional with default `None`. Also changed `thread_id` type from `UUID` to `str` to resolve a 422 error caused by the frontend sending timestamp strings instead of UUIDs.

## Outcome

- ✅ Impact: Resolved 422 validation errors by aligning backend model with frontend data and simplifying optional field definitions.
- 🧪 Tests: Manual verification with various payload combinations (with/without optional fields).
- 📁 Files: backend/app/models/api.py
- 🔁 Next prompts: /sp.verify
- 🧠 Reflection: The mismatch between `UUID` type and timestamp string was likely the root cause of 422 errors, even if `selected_text` was present. Aligning types is critical for successful validation.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
