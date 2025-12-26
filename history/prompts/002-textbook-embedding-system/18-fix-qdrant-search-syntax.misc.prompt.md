---
id: 18
title: Fix Qdrant search syntax in test_retrieval
stage: misc
date: 2025-12-24
surface: agent
model: Gemini 2.0 Flash
feature: textbook-embedding-system
branch: 002-textbook-embedding-system
user: user
command: none
labels: [fix, bug, qdrant, search]
links:
  spec: specs/002-textbook-embedding-system/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - backend/main.py
tests:
 - python -m py_compile backend/main.py
---

## Prompt

Fix Qdrant search method in backend/main.py test_retrieval() function

Error: 'QdrantClient' object has no attribute 'search'

Location: test_retrieval() function, line ~219-225

Replace incorrect search syntax:
results = qdrant_client.search(
    collection_name=COLLECTION_NAME,
    query_vector=query_embedding,
    limit=3
)

With correct Qdrant client method:
results = qdrant_client.search_points(
    collection_name=COLLECTION_NAME,
    vector=query_embedding,
    limit=5
)

This uses the correct method name (search_points) and parameter name (vector) for the installed Qdrant client version.

## Response snapshot

Fixed the Qdrant search syntax in `backend/main.py` by replacing the `search` method with `search_points` and updating the parameter name to `vector`.

## Outcome

- ✅ Impact: Resolved a runtime error in the verification phase, allowing the search functionality to be tested correctly.
- 🧪 Tests: `python -m py_compile backend/main.py` passed.
- 📁 Files: `backend/main.py`
- 🔁 Next prompts: None
- 🧠 Reflection: Keeping up with specific SDK version API changes is crucial for external integrations.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
