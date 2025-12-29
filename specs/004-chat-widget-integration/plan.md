# Implementation Plan - Chat Widget Integration

**Feature**: Chat Widget Integration (Spec-4)
**Branch**: `004-chat-widget-integration`
**Date**: 2025-12-29
**Status**: Draft

## Technical Context

**Language**: JavaScript/JSX (React)
**Framework**: Docusaurus (React-based)
**Project Type**: Frontend Component Integration
**Key Libraries**:
- React 18+ (bundled with Docusaurus)
- Fetch API (native browser)
- localStorage (native browser)

**Technical Architecture**:

### Component Structure
`docusaurus/src/components/ChatWidget/`
- `ChatWidget.jsx`: Main container component
- `ChatMessage.jsx`: Individual message display (User/Assistant)
- `SourceCitation.jsx`: Collapsible sources component
- `ChatInput.jsx`: Input field + send button
- `ChatWidget.css`: Component-specific styling
- `index.js`: Export for easy import

### State Management
- `isOpen` (boolean): Panel visibility
- `messages` (array): Chat history objects `{ id, role, content, sources }`
- `threadId` (string): Conversation session ID
- `input` (string): Current message text
- `loading` (boolean): API call status
- `error` (string): Error message state

### API Flow
1. **User Input** → `ChatWidget` → `sendMessage()`
2. **Fetch POST** → `http://localhost:8000/api/chat`
3. **Backend Response** → `{ answer, sources, thread_id }`
4. **Update State** → `messages` array + `threadId`
5. **Persistence** → `localStorage` (save `thread_id`)
6. **Render** → `ChatMessage` components

### Data Flow
- **Hierarchy**: `ChatWidget` > `ChatHeader`, `ChatMessages` (> `ChatMessage` > `SourceCitation`), `ChatInput`
- **Props**:
    - `messages` passed to `ChatMessages`
    - `onSend` passed to `ChatInput`
    - `sources` passed to `SourceCitation`

## Constitution Check

### Principles
- **Educational Excellence**: N/A (UI feature)
- **Interactive Learning**: Enhances interactivity by allowing students to ask questions.
- **Phased Development**: Phase 1 (Textbook) is complete. Phase 2 (RAG Chatbot) is in progress. This feature matches Phase 2 requirements.
- **Content Structure**: N/A
- **Technical Accuracy**: Code for widget must be robust and error-free.
- **Accessibility**: Widget must be responsive (mobile/desktop) and usable.
- **Performance**: Should not block main thread or slow down page load.

### Constraints
- **Deployment**: Must work on static build (Docusaurus).
- **Mobile Compatibility**: Responsive design required.
- **Security**: Secure API communication (HTTPS in prod, localhost in dev).

### Gates
- [x] Spec approved? (Yes, `specs/004-chat-widget-integration/spec.md` exists)
- [x] Dependencies ready? (Backend API from Spec-3 is assumed ready)
- [x] Technology allowed? (React, Fetch, localStorage are standard)

## Implementation Phases

### Phase 0: Research & Prerequisites
- [ ] Verify Docusaurus is running locally.
- [ ] Verify Backend API is accessible at `http://localhost:8000/api/chat`.
- [ ] Check CORS configuration on backend (must allow frontend origin).

### Phase 1: Design & Data Model
- [ ] Create `specs/004-chat-widget-integration/data-model.md` defining component props and state.
- [ ] Create `specs/004-chat-widget-integration/contracts/api-schema.yaml` defining expected API interface.
- [ ] Create `specs/004-chat-widget-integration/quickstart.md`.

### Phase 2: Component Structure (Scaffolding)
- [ ] Create directory `docusaurus/src/components/ChatWidget/`.
- [ ] Create `ChatWidget.css` with basic styling (floating button, panel).
- [ ] Create `ChatWidget.jsx` with open/close state and basic markup.
- [ ] Create `index.js` exporting the widget.
- [ ] Integrate `ChatWidget` into Docusaurus Layout (swizzling or root wrapper).
- **Verification**: Floating button appears on site; clicks toggle a placeholder panel.

### Phase 3: Message Display Components
- [ ] Create `ChatMessage.jsx` to render user vs. assistant messages.
- [ ] Create `SourceCitation.jsx` to render collapsible sources.
- [ ] Create `ChatInput.jsx` for text entry.
- [ ] Update `ChatWidget.jsx` to render these sub-components with mock data.
- **Verification**: UI renders mock conversation correctly with styling.

### Phase 4: API Integration
- [ ] Implement `sendMessage` function in `ChatWidget.jsx` using `fetch`.
- [ ] Handle loading states (disable input, show spinner).
- [ ] Handle error states (network error, 500s).
- [ ] Parse real response from backend and update `messages` state.
- **Verification**: Can send message to real backend and see real response.

### Phase 5: Thread Persistence
- [ ] Implement `localStorage` logic to save `thread_id`.
- [ ] On mount, check `localStorage` for `thread_id` and (optional) load history if API supports it, or just keep ID for context.
- **Verification**: Refreshing page maintains the session ID (conversation continuity).

### Phase 6: Polish & Responsive Design
- [ ] Refine CSS for mobile devices (full screen panel vs popup).
- [ ] Ensure "Source" details don't overflow.
- [ ] Add "Thinking..." animations.
- [ ] Finalize colors to match site theme.

### Phase 7: Verification
- [ ] Verify SC-001: Button visible on all pages.
- [ ] Verify SC-002: End-to-end message flow.
- [ ] Verify SC-003: Citations expand/collapse.
- [ ] Verify SC-004: Thread persistence.
- [ ] Verify SC-005: Error handling.