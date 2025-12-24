# Data Model: Textbook Content Embedding and Vector Storage System

## Entities

### Textbook Chunk
Represents a single segment of text extracted from the textbook, ready for storage and retrieval.

| Field | Type | Description |
|---|---|---|
| `chunk_id` | `UUID` (or `int`) | Unique identifier for the chunk (can be Qdrant auto-generated or deterministic based on content). |
| `text` | `String` | The actual text content of the chunk (approx. 1000 tokens). |
| `embedding` | `List[float]` | The vector representation of the text (dimension depends on Cohere model, likely 1024). |
| `metadata` | `Object` | See Metadata structure below. |

### Metadata
Contextual information attached to each vector to facilitate filtering and source attribution.

| Field | Type | Description | Example |
|---|---|---|---|
| `module_id` | `String` | Identifier of the module (e.g., "module-01"). | `module-01` |
| `chapter_id` | `String` | Identifier of the chapter (e.g., "chapter-01"). | `chapter-01` |
| `title` | `String` | Title of the page/chapter. | `Introduction to Physical AI` |
| `url` | `String` | Full URL of the source page. | `https://.../module-01/chapter-01` |
| `content_type` | `String` | Type of content (e.g., "text", "code"). | `text` |
| `document_type` | `String` | Type of document source. | `chapter`, `module_intro`, `preface` |
| `chunk_index` | `Integer` | Order of the chunk within the document. | `0`, `1`, `2`... |

## Storage Schema (Qdrant Collection)

**Collection Name**: `physical-ai-textbook`

**Vector Params**:
- **Size**: 1024 (for Cohere `embed-english-v3.0`)
- **Distance**: `Cosine`

**Payload Schema**:
- All fields defined in `Metadata` will be stored as payload fields.
- `text` field will also be stored in payload for retrieval.
