# Data Model: Chat Features

## Storage Schema (localStorage)

### Key: `chat_history`
- **Type**: `Array<ConversationMetadata>`
- **Description**: List of all available conversations, ordered by most recent.
- **Limit**: Max 50 items.

```typescript
interface ConversationMetadata {
  id: string;          // UUID
  title: string;       // First 30 chars of first user message
  timestamp: number;   // Unix timestamp of last update
  preview: string;     // Short preview of last message
}
```

### Key: `chat_thread_{id}`
- **Type**: `Array<Message>`
- **Description**: The actual messages for a specific conversation.

```typescript
interface Message {
  id: string;          // UUID
  role: 'user' | 'assistant' | 'system';
  content: string;
  timestamp: number;
}
```

### Key: `current_thread_id`
- **Type**: `string | null`
- **Description**: The ID of the currently active conversation.

## React State Model (ChatWidget.jsx)

```javascript
state = {
  isOpen: boolean,           // Widget visibility
  messages: Message[],       // Current thread messages
  isLoading: boolean,        // AI processing state
  selectedText: string,      // Context from selection
  showHistory: boolean,      // History panel visibility
  activeThreadId: string,    // Current conversation ID
  conversations: ConversationMetadata[] // Cached history list
}
```

## API Contracts

### Backend Request Update
The `/chat` endpoint payload will be updated to include optional `context`.

```json
{
  "message": "User query here",
  "history": [ ... ],
  "selected_text": "Optional selected text from textbook"
}
```

*Note: Backend implementation is separate, but frontend must send this field.*
