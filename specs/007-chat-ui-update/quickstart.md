# Quickstart: Chat Widget UI Updates

## Prerequisites
- Node.js & NPM installed
- Docusaurus project running (`npm run start` in `docusaurus/`)

## Visual Verification
1.  **Open Chat**: Click the chat bubble.
2.  **Header Check**: Confirm "Close" (X) is on the LEFT, "History" (Document icon) is on the RIGHT.
3.  **Message Check**: Send a message. Confirm response bubble has NO avatar.
4.  **Copy Icon**: Hover over AI response. Confirm "Copy" button uses SVG icon (rectangles), not emoji.

## Functional Verification (Delete)
1.  **Create History**: Send a few messages to create a conversation. Start a new chat and create another.
2.  **Open History**: Click the "History" (document) icon.
3.  **Check Menu**: Hover/Click the 3-dots on a conversation item.
4.  **Delete**: Select "Delete". Confirm `window.confirm` dialog appears.
5.  **Verify Removal**: 
    - Item disappears from list.
    - Check DevTools -> Application -> LocalStorage: `chat_thread_{id}` key is gone.
6.  **Active Delete**: Open a conversation, then open history and delete *that specific* conversation.
    - Verify chat resets to empty "New Chat" state.
