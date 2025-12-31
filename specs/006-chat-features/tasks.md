# Tasks: Chat Features

**Branch**: `006-chat-features` | **Spec**: [specs/006-chat-features/spec.md](./spec.md) | **Plan**: [specs/006-chat-features/plan.md](./plan.md)

## Phase 1: Setup

*Goal: Initialize new component files and styles.*

- [X] T001 [P] Create TextSelectionPopup component file in `docusaurus/src/components/ChatWidget/TextSelectionPopup.jsx`
- [X] T002 [P] Create TextSelectionPopup CSS file in `docusaurus/src/components/ChatWidget/TextSelectionPopup.css`
- [X] T003 [P] Create HistoryPanel component file in `docusaurus/src/components/ChatWidget/HistoryPanel.jsx`
- [X] T004 [P] Create HistoryPanel CSS file in `docusaurus/src/components/ChatWidget/HistoryPanel.css`

## Phase 2: Foundational

*Goal: Clean up existing code and prepare for new features.*

- [X] T005 Remove SourceCitation component import and usage from `docusaurus/src/components/ChatWidget/ChatMessage.jsx`
- [X] T006 Remove SourceCitation component file `docusaurus/src/components/ChatWidget/SourceCitation.jsx`
- [X] T007 Remove SourceCitation styling from `docusaurus/src/components/ChatWidget/ChatWidget.css` (if any)

## Phase 3: User Story 1 - Text Selection (Priority: P1)

*Goal: Enable users to ask AI about selected text.*

- [X] T008 [US1] Implement selection detection logic in `docusaurus/src/components/ChatWidget/TextSelectionPopup.jsx`
- [X] T009 [US1] Implement "Ask AI" button rendering and styling in `docusaurus/src/components/ChatWidget/TextSelectionPopup.jsx` and `docusaurus/src/components/ChatWidget/TextSelectionPopup.css`
- [X] T010 [US1] Add `selectedText` state to `docusaurus/src/components/ChatWidget/ChatWidget.jsx`
- [X] T011 [US1] Import and integrate `TextSelectionPopup` in `docusaurus/src/components/ChatWidget/ChatWidget.jsx`
- [X] T012 [US1] Implement `handleAskAI` callback in `docusaurus/src/components/ChatWidget/ChatWidget.jsx` to open chat and prefill input

## Phase 4: User Story 2 - Conversation History (Priority: P1)

*Goal: Save and restore conversation history.*

- [X] T013 [US2] Define localStorage helper functions (save/load) in `docusaurus/src/components/ChatWidget/ChatWidget.jsx` (or internal helper)
- [X] T014 [US2] Implement `conversations` and `current_thread_id` state in `docusaurus/src/components/ChatWidget/ChatWidget.jsx`
- [X] T015 [US2] Implement `saveMessage` logic to update localStorage when messages are sent in `docusaurus/src/components/ChatWidget/ChatWidget.jsx`
- [X] T016 [US2] Implement `loadHistory` logic on component mount in `docusaurus/src/components/ChatWidget/ChatWidget.jsx`
- [X] T017 [US2] Implement `HistoryPanel` UI to list conversations in `docusaurus/src/components/ChatWidget/HistoryPanel.jsx`
- [X] T018 [US2] Style `HistoryPanel` in `docusaurus/src/components/ChatWidget/HistoryPanel.css`
- [X] T019 [US2] Integrate `HistoryPanel` into `docusaurus/src/components/ChatWidget/ChatWidget.jsx` with toggle state (`showHistory`)

## Phase 5: User Story 4 - Copy Message (Priority: P1)

*Goal: Allow users to copy AI responses.*

- [X] T020 [US4] Add copy button to `docusaurus/src/components/ChatWidget/ChatMessage.jsx`
- [X] T021 [US4] Implement `copyToClipboard` functionality using `navigator.clipboard` in `docusaurus/src/components/ChatWidget/ChatMessage.jsx`
- [X] T022 [US4] Add visual feedback ("Copied!") state in `docusaurus/src/components/ChatWidget/ChatMessage.jsx`
- [X] T023 [US4] Style copy button in `docusaurus/src/components/ChatWidget/ChatWidget.css`

## Phase 6: User Story 3 - New Chat (Priority: P2)

*Goal: specific feature to start a fresh conversation.*

- [X] T024 [US3] Add "New Chat" button to `docusaurus/src/components/ChatWidget/HistoryPanel.jsx`
- [X] T025 [US3] Implement `startNewChat` function in `docusaurus/src/components/ChatWidget/ChatWidget.jsx` to clear current thread and reset state

## Phase 7: Polish & Integration

*Goal: Final backend integration and UI polish.*

- [X] T026 Update `handleSendMessage` in `docusaurus/src/components/ChatWidget/ChatWidget.jsx` to include `selected_text` in API payload
- [X] T027 Verify mobile responsiveness of History Panel and Text Selection Popup
- [X] T028 Verify cross-browser compatibility (via manual check)

## Dependencies

- **US1 (Text Selection)**: Independent.
- **US2 (History)**: Independent.
- **US4 (Copy)**: Independent.
- **US3 (New Chat)**: Depends on US2 (History) infrastructure.
- **Polish**: Depends on all stories.

## Parallel Execution Examples

- **Team A**: Work on US1 (Text Selection) - T001, T002, T008, T009
- **Team B**: Work on US4 (Copy) - T020, T021, T022, T023
- **Team C**: Work on US2 (History) - T003, T004, T013-T019

## Implementation Strategy

1.  **MVP**: Start with **US4 (Copy)** as it's the simplest and self-contained.
2.  **Core**: Implement **US1 (Text Selection)** and **US2 (History)** in parallel or sequence.
3.  **Refinement**: Finish with **US3 (New Chat)** and **Polish**.
