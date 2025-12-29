# Actionable Tasks: Chat Widget Integration

**Feature**: Chat Widget Integration (Spec-4)
**Branch**: `004-chat-widget-integration`
**Status**: Draft

## Implementation Strategy
- **MVP First**: Start with a simple floating button and panel (US1).
- **Incremental Features**: Add messaging (US2), then citations (US3), then persistence (US4).
- **Component-Based**: Build small, isolated components (`ChatMessage`, `SourceCitation`) and compose them.
- **State Lifting**: Keep state in `ChatWidget` and pass down via props.

## Dependencies
1. **Setup** (Phase 1) must complete before **Foundational** (Phase 2).
2. **Foundational** (Phase 2) must complete before any User Story phases.
3. **US1** (Phase 3) is a prerequisite for US2, US3, US4.
4. **US2** (Phase 4) is a prerequisite for US3 (Citations need messages).
5. **US4** (Phase 6) can be done in parallel with US3 (Phase 5) after US2.

## Parallel Execution Opportunities
- **Styling vs Logic**: One developer can work on CSS while another builds the React logic.
- **Component Development**: `SourceCitation` (US3) can be built independently of `ChatInput` (US2) once scaffolding is done.

---

### Phase 1: Setup
**Goal**: Ensure environment is ready for development.

- [x] T001 Verify Docusaurus runs locally `npm start`
- [x] T002 Verify backend API is accessible at `http://localhost:8000/api/chat`
- [x] T003 Verify CORS configuration on backend allows localhost:3000

### Phase 2: Foundational
**Goal**: Establish file structure and base styling.

- [x] T004 Create component directory `docusaurus/src/components/ChatWidget/`
- [x] T005 [P] Create base CSS file `docusaurus/src/components/ChatWidget/ChatWidget.css`
- [x] T006 Create index export file `docusaurus/src/components/ChatWidget/index.js`

### Phase 3: User Story 1 - Open and Close Chat Widget
**Goal**: User can toggle the chat interface.
**Independent Test**: Click floating button -> Panel opens. Click close -> Panel closes.

- [x] T007 [US1] Create `ChatWidget.jsx` with basic open/close state in `docusaurus/src/components/ChatWidget/ChatWidget.jsx`
- [x] T008 [US1] Integrate `ChatWidget` into Docusaurus root layout (Swizzle `Layout` or add to `Root` wrapper)
- [x] T009 [US1] Style the floating button and panel container in `docusaurus/src/components/ChatWidget/ChatWidget.css`

### Phase 4: User Story 2 - Send Message and Receive Response
**Goal**: Functional chat loop with backend.
**Independent Test**: Send "Hello" -> See "Thinking..." -> See backend response.

- [x] T010 [P] [US2] Create `ChatInput.jsx` component in `docusaurus/src/components/ChatWidget/ChatInput.jsx`
- [x] T011 [P] [US2] Create `ChatMessage.jsx` component in `docusaurus/src/components/ChatWidget/ChatMessage.jsx`
- [x] T012 [US2] Implement `sendMessage` function with `fetch` in `docusaurus/src/components/ChatWidget/ChatWidget.jsx`
- [x] T013 [US2] Handle loading state and disable input during fetch in `docusaurus/src/components/ChatWidget/ChatWidget.jsx`
- [x] T014 [US2] Handle API errors and display user-friendly error message in `docusaurus/src/components/ChatWidget/ChatWidget.jsx`

### Phase 5: User Story 3 - View Source Citations
**Goal**: Display RAG sources.
**Independent Test**: Ask factual question -> Click "Sources" -> See details.

- [x] T015 [P] [US3] Create `SourceCitation.jsx` component in `docusaurus/src/components/ChatWidget/SourceCitation.jsx`
- [x] T016 [US3] Update `ChatMessage.jsx` to render `SourceCitation` when sources exist in `docusaurus/src/components/ChatWidget/ChatMessage.jsx`
- [x] T017 [US3] Style citation details (collapsible) in `docusaurus/src/components/ChatWidget/ChatWidget.css`

### Phase 6: User Story 4 - Maintain Thread History
**Goal**: Persist conversation ID.
**Independent Test**: Refresh page -> `thread_id` remains in localStorage -> Chat continues.

- [x] T018 [US4] Implement `localStorage` saving of `thread_id` in `docusaurus/src/components/ChatWidget/ChatWidget.jsx`
- [x] T019 [US4] Implement initialization logic to read `thread_id` on mount in `docusaurus/src/components/ChatWidget/ChatWidget.jsx`

### Phase 7: Polish & Cross-Cutting Concerns
**Goal**: Production-ready UX.

- [x] T020 Refine mobile responsiveness (full screen panel on small screens) in `docusaurus/src/components/ChatWidget/ChatWidget.css`
- [x] T021 Add "Thinking..." animation/indicator in `docusaurus/src/components/ChatWidget/ChatWidget.css`
- [x] T022 Final manual verification of all User Stories
