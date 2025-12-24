# Feature Specification: Textbook Content Embedding and Vector Storage System

**Feature Branch**: `002-textbook-embedding-system`  
**Created**: 2025-12-24  
**Status**: Draft  
**Input**: User description: "Textbook Content Embedding and Vector Storage System Target audience: RAG chatbot backend that needs searchable textbook knowledge Focus: Converting deployed textbook content into searchable vector embeddings..."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Content Ingestion and Embedding (Priority: P1)

As a system developer, I need the system to automatically fetch, process, and embed all textbook content so that it can be stored for future retrieval.

**Why this priority**: This is the core functionality; without embedding the content, no retrieval is possible.

**Independent Test**: Can be fully tested by running the indexing script and verifying that the correct number of documents (preface + modules + chapters) are processed and sent to the vector database.

**Acceptance Scenarios**:

1. **Given** the live textbook URL, **When** the indexing process runs, **Then** it fetches all 32 documents (Preface, 6 module intros, 25 chapters).
2. **Given** fetched content, **When** processed, **Then** it is chunked into segments of 1000 tokens with 100 token overlap.
3. **Given** chunked content, **When** embedding generation is triggered, **Then** vector embeddings are created using the Cohere model.

---

### User Story 2 - Vector Storage and Metadata (Priority: P2)

As a system developer, I need the embeddings to be stored with rich metadata so that retrieval can be filtered and attributed correctly.

**Why this priority**: Essential for the RAG system to provide context and cite sources.

**Independent Test**: Can be tested by querying the vector database and inspecting the payload of stored vectors.

**Acceptance Scenarios**:

1. **Given** generated embeddings, **When** stored in Qdrant, **Then** each vector includes metadata for module_id, chapter_id, title, url, content_type, and document_type.
2. **Given** the free tier constraints, **When** indexing completes, **Then** the total storage usage remains within the 1GB limit.

---

### User Story 3 - Verification and Retrieval (Priority: P3)

As a system developer, I need to verify that the stored embeddings can be retrieved using semantic queries so that I know the system is ready for the RAG chatbot.

**Why this priority**: Validates the end-to-end quality of the indexing.

**Independent Test**: Run a verification script with sample queries and check if relevant textbook chunks are returned.

**Acceptance Scenarios**:

1. **Given** a sample query (e.g., "What is physical AI?"), **When** the verification script runs, **Then** it returns relevant text chunks from the vector database.
2. **Given** the verification script, **When** it completes, **Then** it outputs a success/failure report based on retrieval quality.

### Edge Cases

- **Network Failure**: What happens if the deployed textbook URL or sitemap is unreachable? (System should log error and retry or exit gracefully)
- **Rate Limits**: What happens if the embedding API or Vector DB API rate limits are hit? (System should implement backoff or adhere to rate limits)
- **Empty Content**: What happens if a chapter page is empty or parsing fails? (System should log a warning and skip to the next document)
- **Storage Limit**: What happens if the total vectors exceed the free tier storage? (System should check usage pre- or post-batch and warn/fail)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST fetch content from the deployed textbook website URL.
- **FR-002**: System MUST parse the sitemap to identify all relevant pages (Preface, Module Intros, Chapters).
- **FR-003**: System MUST split text content into chunks of 1000 tokens with a 100-token overlap.
- **FR-004**: System MUST generate vector embeddings for each text chunk using a compatible embedding model.
- **FR-005**: System MUST store embeddings in a vector database collection.
- **FR-006**: System MUST attach metadata to each stored embedding, including: module identifier, chapter identifier, page title, source URL, content type, and document type.
- **FR-007**: System MUST include a verification mechanism to test retrieval of stored content.
- **FR-008**: System MUST operate within the defined storage constraints of the target environment.
- **FR-009**: System MUST complete the entire indexing process in a single run (target < 15 minutes).

### Key Entities *(include if feature involves data)*

- **Textbook Chunk**: Represents a segment of text from the textbook, containing the text content, vector embedding, and associated metadata.
- **Metadata**: Structured information attached to each chunk (Module ID, Chapter ID, Title, URL, Type).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: System successfully indexes 100% of the targeted 32 documents (Preface, 6 Module Intros, 25 Chapters).
- **SC-002**: Indexing process completes successfully in under 15 minutes.
- **SC-003**: 100% of stored vectors contain valid non-null metadata for Title and URL.
- **SC-004**: Verification script returns at least one relevant result for 5 out of 5 predefined sample queries.
- **SC-005**: Storage usage in the vector database does not exceed the defined free tier limits (1GB).

## Assumptions & Constraints

- **Technology Stack**:
    - Embedding Model: Cohere `embed-english-v3.0`
    - Vector Database: Qdrant Cloud Free Tier
    - Language: Python
- **Content Source**: https://textbook-physical-ai-humanoid-robotics.vercel.app/
- **Limitations**:
    - One-time indexing (no real-time updates).
    - English language only.
    - No API endpoints or user interface (CLI/Script only).