# Implementation Plan: Chat Widget UI Updates

**Branch**: `007-chat-ui-update` | **Date**: 2025-12-31 | **Spec**: [specs/007-chat-ui-update/spec.md](./spec.md)
**Input**: Feature specification from `specs/007-chat-ui-update/spec.md`

## Summary

Modernize the Chat Widget UI by replacing emoji icons with inline SVGs, reorganizing the header layout (close left, history right), and adding a "delete conversation" feature via a three-dot menu in the history panel. All data persistence changes are client-side (`localStorage`).

## Technical Context

**Language/Version**: JavaScript (React)
**Framework**: Docusaurus
**Storage**: localStorage (Client-side)
**Testing**: Manual visual and functional verification
**Target Platform**: Web (Desktop & Mobile)
**Constraints**: Use inline SVGs (no new dependencies), maintain existing data structure.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Principle 6: Accessibility**: SVG icons should have appropriate `aria-label` attributes.
- **Principle 7: Performance**: Inline SVGs are performant and don't require network requests.
- **Phase 2 Requirements**: Enhancements align with the "Floating chat widget" and "Conversation history tracking" features.

**Result**: PASS

## Project Structure

### Documentation (this feature)

```text
specs/007-chat-ui-update/
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
    │   ├── ChatWidget.jsx        # Modify: Header layout, props
    │   ├── ChatMessage.jsx       # Modify: Copy icon, remove avatar
    │   ├── HistoryPanel.jsx      # Modify: Add delete menu
    │   └── css/
    │       ├── ChatWidget.css    # Modify: Styles for new layout/icons
    │       └── HistoryPanel.css  # Modify: Styles for menu
```

**Structure Decision**: Modify existing React components and CSS files in `docusaurus/src/components/ChatWidget/`.

## High-Level Architecture

### Components
1.  **ChatWidget**: 
    - Update header JSX: Move Close button to start, History button to end.
    - Replace `🕐` with SVG.
    - Handle `onDeleteConversation` callback from `HistoryPanel`.
2.  **HistoryPanel**: 
    - Add "Three-dot" menu trigger to each item.
    - Implement dropdown state (`openMenuId`).
    - Handle delete action: remove from `conversations` prop (up to parent) and `localStorage`.
3.  **ChatMessage**:
    - Remove avatar element.
    - Replace `📋` with SVG.

### Data Flow
1.  **Delete Action**: User clicks Delete -> `HistoryPanel` calls `onDelete` -> `ChatWidget` updates state & `localStorage` -> UI updates.
2.  **Active Conversation Delete**: If deleted ID == active ID -> `ChatWidget` resets to "New Chat".

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| None      |            |                                     |