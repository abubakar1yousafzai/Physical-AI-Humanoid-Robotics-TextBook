# Interface Definition: Textbook Embedding

Since this feature is a CLI script, the "API" is the interface with Qdrant and the internal function signatures.

## Qdrant Collection Schema

- **Collection Name**: `physical-ai-textbook`
- **Vector Size**: 1024
- **Distance Metric**: Cosine

## Payload Structure

```json
{
  "module_id": "string",
  "chapter_id": "string",
  "title": "string",
  "url": "string",
  "content_type": "string",
  "document_type": "string",
  "chunk_index": "integer",
  "text": "string"
}
```
