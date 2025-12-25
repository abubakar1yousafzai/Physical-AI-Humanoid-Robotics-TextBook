---
id: 10
title: Refine ingestion tests and add quiz filtering
stage: tasks
date: 2025-12-24
surface: agent
model: Gemini 2.0 Flash
feature: textbook-embedding-system
branch: 002-textbook-embedding-system
user: user
command: none
labels: [refactor, test, ingestion, filtering]
links:
  spec: specs/002-textbook-embedding-system/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - backend/main.py
 - backend/test_ingestion.py
 - backend/pyproject.toml
 - backend/uv.lock
tests:
 - backend/test_ingestion.py
---

## Prompt

Update test_ingestion.py with three critical fixes

File: backend/test_ingestion.py
... (Fixes for test_chunk_text, test_extract_text_structure, test_get_all_urls with quiz filtering, and manual test section)

## Response snapshot

Implemented the requested fixes in `backend/test_ingestion.py` and updated `backend/main.py` to ensure quiz URLs are filtered out. Added `pytest` and `lxml` to backend dependencies.

## Outcome

- ✅ Impact: Improved test reliability and ensured sensitive quiz content is not indexed for the chatbot.
- 🧪 Tests: `cd backend; uv run pytest test_ingestion.py` passed (3/3).
- 📁 Files: `backend/main.py`, `backend/test_ingestion.py`, `backend/pyproject.toml`
- 🔁 Next prompts: /sp.implement phase 4
- 🧠 Reflection: Validating the structure of extracted data and filtering sensitive URLs (quizzes) are critical steps for a production-ready RAG pipeline.

## Evaluation notes (flywheel)

- Failure modes observed: Missing `lxml` parser for XML sitemap.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
