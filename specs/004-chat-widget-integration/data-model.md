# Data Model: Chat Widget Integration

## Frontend Entities

### ChatMessage
Represents a single message in the conversation history.

| Field | Type | Description |
|---|---|---|
| `id` | string (UUID) | Unique identifier for the message (generated frontend side or by backend). |
| `role` | enum | `'user'` or `'assistant'`. |
| `content` | string | The text content of the message. |
| `timestamp` | number | Unix timestamp of when the message was sent/received. |
| `sources` | SourceCitation[] | Optional list of sources (for assistant messages only). |

### SourceCitation
Represents a reference used by the RAG system.

| Field | Type | Description |
|---|---|---|
| `title` | string | Title of the source document/chapter. |
| `url` | string | URL link to the specific section (if applicable). |
| `snippet` | string | Short text excerpt from the source. |
| `relevance` | number | Optional relevance score (0-1). |

### ChatState
The internal state of the `ChatWidget` component.

| Field | Type | Description |
|---|---|---|
| `isOpen` | boolean | Visibility of the chat panel. |
| `messages` | ChatMessage[] | Array of message objects. |
| `threadId` | string \| null | Current session ID (UUID). |
| `isLoading` | boolean | True when waiting for API response. |
| `error` | string \| null | Current error message (if any). |
| `inputText` | string | Current value of input field. |

## Persistence
**Storage**: `localStorage`
**Key**: `chat_thread_id`
**Value**: UUID string (e.g., `"550e8400-e29b-41d4-a716-446655440000"`)
