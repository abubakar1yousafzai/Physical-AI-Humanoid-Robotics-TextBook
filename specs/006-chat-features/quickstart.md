# Quickstart: Chat Features Development

## Prerequisites
- Node.js & NPM installed
- Docusaurus project running (`npm run start` in `docusaurus/`)

## Testing Text Selection
1. Open any page in the Docusaurus site.
2. Select a paragraph of text (>10 chars).
3. Verify "✨ Ask AI" popup appears above selection.
4. Click popup -> Chat should open with text pre-filled.

## Testing History
1. Send a message in the chat.
2. Refresh the page.
3. Open chat -> Message should still be there.
4. Click "New Chat" -> Chat should clear.
5. Click History icon (top left of chat) -> Sidebar should show previous chat.

## Testing Copy
1. Ask AI a question.
2. Wait for response.
3. Hover over response (or look for icon).
4. Click Clipboard icon.
5. Verify "Copied!" text appears.
6. Paste into a notepad to confirm.
