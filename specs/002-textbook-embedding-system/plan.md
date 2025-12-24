# Implementation Plan: Textbook Content Embedding and Vector Storage System

**Branch**: `002-textbook-embedding-system` | **Date**: 2025-12-24 | **Spec**: [specs/002-textbook-embedding-system/spec.md](spec.md)
**Input**: Feature specification from `specs/002-textbook-embedding-system/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build a Python CLI script (`main.py`) that indexes deployed textbook content into a Qdrant vector database using Cohere embeddings. The system will scrape the deployed Vercel app, chunk the text, generate embeddings, and store them with rich metadata in a single run.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**: `uv` (package manager), `cohere`, `qdrant-client`, `requests`, `beautifulsoup4`, `python-dotenv`, `tiktoken`
**Storage**: Qdrant Cloud (Free Tier)
**Testing**: `pytest` (for unit tests if needed, though mostly verification script)
**Target Platform**: Local execution / CLI (indexing script)
**Project Type**: Python Script / Backend Tool
**Performance Goals**: Complete indexing in < 15 minutes.
**Constraints**: 1GB storage limit (Qdrant Free Tier), single-run indexing.
**Scale/Scope**: ~32 documents, ~200-300 chunks.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Principle 3 (Phased Development)**: Compliant. This is the foundation for Phase 2 (RAG Chatbot).
- **Principle 5 (Technical Accuracy)**: Compliant. Script will be verified.
- **Phase 2 Requirements**: Explicitly uses the specified stack (Cohere `embed-english-v3.0`, Qdrant).

## Project Structure

### Documentation (this feature)

```text
specs/002-textbook-embedding-system/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── schema.md
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── main.py           # Complete indexing system
├── pyproject.toml    # UV dependencies
├── .env              # API keys
├── .env.example      # Template
└── README.md         # Documentation
```

**Structure Decision**: A dedicated `backend/` directory is created to house the Python logic, keeping it separate from the Docusaurus frontend. This aligns with a future "Full-stack" structure where FastAPI might also live in `backend/`.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| None | N/A | N/A |

## Plan Details

### Phase 1: Environment Setup
- Initialize `backend/` project with `uv`.
- Install dependencies (`cohere`, `qdrant-client`, etc.).
- Configure `.env` and `.env.example`.

### Phase 2: Build main.py Script
- Implement `get_all_urls` (Sitemap parsing / discovery).
- Implement `extract_text_from_url` (BS4 scraping).
- Implement `chunk_text` (1000 tokens, 100 overlap).
- Implement `embed` (Cohere API).
- Implement `create_collection` (Qdrant setup).
- Implement `save_chunk_to_qdrant`.
- Implement `main` orchestration.

### Phase 3: Execution & Verification
- Run indexing pipeline.
- Verify Qdrant collection size and content.
- Test retrieval with sample queries.

### Phase 4: Documentation
- Add `backend/README.md`.