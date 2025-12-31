# Implementation Plan: Chat Features

**Branch**: `006-chat-features` | **Date**: 2025-12-30 | **Spec**: [specs/006-chat-features/spec.md](./spec.md)
**Input**: Feature specification from `specs/006-chat-features/spec.md`

## Summary

Enhance the existing Docusaurus chat widget with three key features: text selection queries ("Ask AI"), local conversation history management, and message copying. Additionally, streamline the UI by removing source citations. Storage will be handled client-side using `localStorage`.

## Technical Context

**Language/Version**: JavaScript (React)
**Framework**: Docusaurus
**Storage**: localStorage (Client-side)
**Deployment**: Local development, Vercel
**Testing**: Manual verification, Browser testing (Chrome, Firefox, Safari)
**Target Platform**: Web (Desktop & Mobile)
**Constraints**: No new NPM packages; use native browser APIs.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Principle 1: Educational Excellence**: Enhances learning by allowing direct queries on text.
- **Principle 2: Interactive Learning**: Increases interactivity.
- **Principle 6: Accessibility**: Must ensure new UI elements are accessible.
- **Principle 7: Performance**: localStorage operations must not block UI.
- **Phase 2 Requirements**: Explicitly aligns with "Text selection-based queries", "Conversation history tracking", "Clean UI".

**Result**: PASS

## Project Structure

### Documentation (this feature)

```text
specs/006-chat-features/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
└── tasks.md             # Phase 2 output
```

### Source Code (repository root)

```text
docusaurus/
└── src/
    ├── components/
    │   ├── ChatWidget.jsx        # Main widget to modify
    │   ├── ChatMessage.jsx       # Message display to modify
    │   ├── TextSelectionPopup.jsx # NEW: Popup for text selection
    │   ├── HistoryPanel.jsx      # NEW: Sidebar for history
    │   ├── css/
    │   │   ├── ChatWidget.css    # Existing styles
    │   │   ├── TextSelectionPopup.css # NEW styles
    │   │   └── HistoryPanel.css  # NEW styles
    │   └── SourceCitation.jsx    # TO REMOVE
```

**Structure Decision**: enhance existing Docusaurus `src/components` with new React components and CSS files.

## High-Level Architecture

### Components
1.  **ChatWidget**: Orchestrator. Manages state (`selectedText`, `showHistory`, `conversations`).
2.  **TextSelectionPopup**: Listens for `selectionchange`. Renders "Ask AI" button.
3.  **HistoryPanel**: Displays list of `conversations` from `localStorage`. Loads thread on click.
4.  **ChatMessage**: Displays message content. Handles "Copy" action.

### Data Flow
1.  **Selection**: User selects text -> `TextSelectionPopup` detects -> Click -> Updates `ChatWidget` state -> Opens Chat.
2.  **History**: App load -> `ChatWidget` reads `localStorage` -> Populates `conversations`.
3.  **New Message**: User sends -> `ChatWidget` updates state -> Saves to `localStorage` (`chat_thread_{id}`).

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| None      |            |                                     |