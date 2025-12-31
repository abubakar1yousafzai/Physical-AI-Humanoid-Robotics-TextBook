# Tasks: Chat Widget UI Updates

**Branch**: `007-chat-ui-update` | **Spec**: [specs/007-chat-ui-update/spec.md](./spec.md) | **Plan**: [specs/007-chat-ui-update/plan.md](./plan.md)

## Phase 1: Foundational (Icons & Layout)

*Goal: Modernize the chat interface with SVG icons and improved layout (User Story 1).*

- [X] T001 [US1] Replace History button emoji with SVG icon in `docusaurus/src/components/ChatWidget/ChatWidget.jsx`
- [X] T002 [US1] Swap header buttons (Close left, History right) in `docusaurus/src/components/ChatWidget/ChatWidget.jsx`
- [X] T003 [US1] Update header styling for new button layout in `docusaurus/src/components/ChatWidget/ChatWidget.css`
- [X] T004 [US1] Replace Copy button emoji with SVG icon in `docusaurus/src/components/ChatWidget/ChatMessage.jsx`
- [X] T005 [US1] Remove Assistant avatar from `docusaurus/src/components/ChatWidget/ChatMessage.jsx`

## Phase 2: User Story 2 - Delete Conversation (Priority: P1)

*Goal: Allow users to delete conversations from history.*

- [X] T006 [US2] Implement `handleDeleteConversation` logic (localStorage & state update) in `docusaurus/src/components/ChatWidget/ChatWidget.jsx`
- [X] T007 [US2] Pass `onDeleteConversation` prop to `HistoryPanel` in `docusaurus/src/components/ChatWidget/ChatWidget.jsx`
- [X] T008 [US2] Add menu state (`openMenuId`) and toggle logic to `docusaurus/src/components/ChatWidget/HistoryPanel.jsx`
- [X] T009 [US2] Render "Three-dot" menu button and "Delete" dropdown for each item in `docusaurus/src/components/ChatWidget/HistoryPanel.jsx`
- [X] T010 [US2] Implement "click outside" handler to close open menus in `docusaurus/src/components/ChatWidget/HistoryPanel.jsx`
- [X] T011 [US2] Wire up "Delete" action with `window.confirm` in `docusaurus/src/components/ChatWidget/HistoryPanel.jsx`
- [X] T012 [US2] Style dropdown menu and 3-dot button in `docusaurus/src/components/ChatWidget/HistoryPanel.css`

## Phase 3: Polish & Verification

*Goal: Ensure UI quality across devices.*

- [X] T013 Verify mobile responsiveness for new header layout
- [X] T014 Verify dropdown menu positioning and touch targets on mobile
- [X] T015 Verify visual consistency of inline SVGs (color, size)

## Dependencies

- **US1 (Icons/Layout)**: Independent.
- **US2 (Delete)**: Depends on `HistoryPanel` existence (already present).

## Parallel Execution Examples

- **Team A**: Work on US1 (Icons/Layout) - T001-T005
- **Team B**: Work on US2 (Delete Logic & UI) - T006-T012

## Implementation Strategy

1.  **Visual Refresh**: Tackle Phase 1 first to clear visual debt and simple DOM changes.
2.  **Logic Core**: Implement the delete logic in `ChatWidget` (T006).
3.  **UI Interaction**: Build the menu in `HistoryPanel` and connect it to the logic.
