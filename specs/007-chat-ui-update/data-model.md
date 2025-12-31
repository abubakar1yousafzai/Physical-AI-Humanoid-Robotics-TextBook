# Data Model: Chat Widget UI Updates

## State Updates

### ChatWidget.jsx
No new persistent state, but `conversations` state will need to be updated after deletion.

### HistoryPanel.jsx
New local state for managing the active dropdown menu:

```javascript
const [openMenuId, setOpenMenuId] = useState(null); // ID of the conversation with open menu
```

## LocalStorage Operations

### Delete Operation
When deleting conversation `targetId`:

1.  **Read** `chat_history` (Array).
2.  **Filter** out item where `id === targetId`.
3.  **Write** updated array back to `chat_history`.
4.  **Remove** item `chat_thread_${targetId}`.
5.  **Remove** `chat_thread_id` IF it matches `targetId` (reset active state).

## Component Interfaces

### HistoryPanel Props

```javascript
props = {
  // Existing props...
  onDeleteConversation: (id: string) => void // New callback
}
```

### ChatWidget Callbacks

```javascript
const handleDeleteConversation = (id) => {
  // 1. Remove from localStorage (history list + thread messages)
  // 2. Update conversations state
  // 3. If id == activeThreadId, reset to new chat
}
```
