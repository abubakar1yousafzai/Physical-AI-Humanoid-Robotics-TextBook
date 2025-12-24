# Research: Textbook Content Embedding and Vector Storage System

## Technology Decisions

### Package Manager: `uv`
- **Decision**: Use `uv` for Python dependency management.
- **Rationale**: Fast, reliable, and modern package manager. Matches user preference.
- **Alternatives Considered**: `pip`, `poetry`, `conda` (rejected based on specific user request).

### Embedding Model: Cohere `embed-english-v3.0`
- **Decision**: Use Cohere's `embed-english-v3.0` model.
- **Rationale**: Strong performance for English text semantic search. Specified in Constitution and Spec.
- **Alternatives Considered**: OpenAI `text-embedding-3-small` (rejected as per Constitution update).

### Vector Database: Qdrant Cloud (Free Tier)
- **Decision**: Use Qdrant Cloud.
- **Rationale**: Provides a free tier adequate for the project scale (1GB limit), robust API, and Python client support.
- **Alternatives Considered**: Pinecone, Milvus, Chroma (rejected based on specific user request/constraints).

### HTML Parsing: BeautifulSoup4
- **Decision**: Use `beautifulsoup4`.
- **Rationale**: Standard, robust library for parsing HTML and extracting text content.
- **Alternatives Considered**: `lxml`, `scrapy` (BS4 is sufficient and simpler for this scraping task).

### HTTP Requests: `requests`
- **Decision**: Use `requests`.
- **Rationale**: Standard synchronous HTTP library for Python. Simple and effective for this sequential indexing script.
- **Alternatives Considered**: `httpx`, `aiohttp` (async not strictly necessary for this scale, but `httpx` is modern; sticking to `requests` as per plan).
