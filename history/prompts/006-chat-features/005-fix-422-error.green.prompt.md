---
id: 005
title: Fix 422 Error and Add Manual Send
stage: green
date: 2025-12-30
surface: agent
model: gemini
feature: 006-chat-features
branch: 006-chat-features
user: User
command: fix-422-validation
labels: ["bugfix", "backend", "frontend", "chat"]
links:
  spec: specs/006-chat-features/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - backend/app/models/api.py
 - docusaurus/src/components/ChatWidget/ChatWidget.jsx
 - docusaurus/src/components/ChatWidget/ChatInput.jsx
tests:
 - null
---

## Prompt

Fix 422 validation error and add manual send option in chat widget

## Response snapshot

Fixed the 422 validation error by adding `selected_text` as an Optional field in the backend Pydantic model. Updated the frontend `ChatWidget` to ensure `selected_text` is sent as `null` (not empty string) when no text is selected. Also added an `onKeyDown` handler to `ChatInput` to allow sending messages by pressing the Enter key.

## Outcome

- ✅ Impact: Resolved 422 error on message send and improved UX with Enter key support.
- 🧪 Tests: Manual verification required.
- 📁 Files: Modified backend/app/models/api.py, docusaurus/src/components/ChatWidget/ChatWidget.jsx, docusaurus/src/components/ChatWidget/ChatInput.jsx
- 🔁 Next prompts: /sp.verify
- 🧠 Reflection: The 422 error was caused by a schema mismatch between frontend and backend. Ensuring explicit `null` handling in frontend and updating backend schema resolved it.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
