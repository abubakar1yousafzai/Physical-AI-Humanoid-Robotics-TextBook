# Tasks: Textbook Content Embedding and Vector Storage System

**Feature Branch**: `002-textbook-embedding-system`
**Spec**: [specs/002-textbook-embedding-system/spec.md](spec.md)
**Plan**: [specs/002-textbook-embedding-system/plan.md](plan.md)

## Phase 1: Environment Setup

- [x] T001 Initialize backend project with `uv` in `backend/`
- [x] T002 Configure `.env` and `.env.example` in `backend/` with COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY placeholders
- [x] T003 Install python dependencies (cohere, qdrant-client, requests, beautifulsoup4, python-dotenv, tiktoken) via `uv`

## Phase 2: Foundational Tasks

**Goal**: Establish the core project structure and shared utilities.

- [ ] T004 Create `backend/main.py` skeleton with imports and main execution block
- [ ] T005 Implement environment loading and validation in `backend/main.py`
- [ ] T006 Implement shared logging configuration in `backend/main.py`

## Phase 3: Content Ingestion and Embedding (User Story 1)

**Goal**: Fetch, parse, chunk, and embed textbook content.
**Independent Test**: Run `main.py` (partial) to verify chunks are generated and embeddings created for a single URL.

- [ ] T007 [US1] Implement `get_all_urls(base_url)` in `backend/main.py` to parse sitemap
- [ ] T008 [P] [US1] Implement `extract_text_from_url(url)` in `backend/main.py` using BeautifulSoup4
- [ ] T009 [P] [US1] Implement `chunk_text(text)` in `backend/main.py` (1000 tokens, 100 overlap)
- [ ] T010 [US1] Implement `embed(texts)` in `backend/main.py` using Cohere API
- [ ] T011 [US1] Create integration test script `backend/test_ingestion.py` to verify fetching and chunking logic

## Phase 4: Vector Storage and Metadata (User Story 2)

**Goal**: Store generated embeddings with rich metadata in Qdrant.
**Independent Test**: Verify vectors are stored in Qdrant Cloud with correct payload fields.

- [ ] T012 [US2] Implement `create_collection()` in `backend/main.py` for Qdrant setup (size 1024, Cosine)
- [ ] T013 [US2] Implement `save_chunk_to_qdrant(...)` in `backend/main.py` mapping metadata to payload
- [ ] T014 [US2] Update `main()` loop in `backend/main.py` to orchestrate full pipeline (Fetch -> Chunk -> Embed -> Save)
- [ ] T015 [US2] Add rate limiting/backoff logic to `save_chunk_to_qdrant` in `backend/main.py`

## Phase 5: Verification and Retrieval (User Story 3)

**Goal**: Verify system end-to-end and test retrieval.
**Independent Test**: Run verification script and confirm relevant results for sample queries.

- [ ] T016 [US3] Implement `verify_indexing()` function in `backend/main.py` to query Qdrant count
- [ ] T017 [US3] Implement `test_retrieval(query)` in `backend/main.py` to print top-k matches
- [ ] T018 [US3] Add command-line arguments to `backend/main.py` (e.g., `--verify`, `--run`) for mode selection

## Phase 6: Polish & Documentation

- [ ] T019 Create `backend/README.md` with setup and usage instructions
- [ ] T020 Review and finalize error handling (network retries, empty pages) in `backend/main.py`
- [ ] T021 Clean up temporary files or logs

## Dependencies

1. **US1 (Ingestion)**: Depends on Phase 1 & 2 setup.
2. **US2 (Storage)**: Depends on US1 (needs chunks to store).
3. **US3 (Verification)**: Depends on US2 (needs data in DB to verify).

## Parallel Execution Opportunities

- **T008 (Extraction)** and **T009 (Chunking)** are independent logic functions and can be implemented in parallel.
- **T016 (Verify Count)** and **T017 (Test Retrieval)** can be implemented in parallel once the client is set up.

## Implementation Strategy

1. **MVP**: Complete Phase 1-4 (US1 & US2) to get data into the DB.
2. **Verification**: Add Phase 5 (US3) to ensure quality.
3. **Polish**: Finalize documentation and error handling.
