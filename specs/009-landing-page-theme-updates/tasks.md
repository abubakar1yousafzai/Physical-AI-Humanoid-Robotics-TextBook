# Tasks: Landing Page Content Updates & Theme Fixes

**Input**: Design documents from `/specs/009-landing-page-theme-updates/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, quickstart.md

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 [P] Verify local docusaurus environment and checkout branch `009-landing-page-theme-updates`
- [X] T002 [P] Identify all default Infima green color usage in `docusaurus/src/css/custom.css`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T003 [P] Define neon theme color variables in `:root` of `docusaurus/src/css/custom.css`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Correct Landing Page Content (Priority: P1) 🎯 MVP

**Goal**: Update the landing page to reflect the correct 6-module curriculum and stats.

**Independent Test**: Visit the landing page and verify hero subtitle, stats (6, 23, 1000+, 5000+), and all 6 module cards.

### Implementation for User Story 1

- [X] T004 [US1] Update hero subtitle and typing effect logic in `docusaurus/src/pages/index.js`
- [X] T005 [US1] Update stats section data values (6 modules, 23 chapters) in `docusaurus/src/pages/index.js`
- [X] T006 [US1] Update module cards array with 6 modules and unique descriptions in `docusaurus/src/pages/index.js`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently.

---

## Phase 4: User Story 2 - Consistent Neon Theme Across App (Priority: P1)

**Goal**: Apply the neon cyan/purple theme consistently across all UI elements and remove all green.

**Independent Test**: Navigate Navbar, Sidebar, Footer, Chat, and Popups to verify neon theme and zero green.

### Implementation for User Story 2

- [X] T007 [US2] Replace Infima primary color variables with neon cyan (#00fff2) in `docusaurus/src/css/custom.css`
- [X] T008 [US2] Implement linear-gradient and styling for navbar title in `docusaurus/src/css/custom.css`
- [X] T009 [US2] Update sidebar link hover and active states to use neon cyan in `docusaurus/src/css/custom.css`
- [X] T010 [US2] Style footer with dark gradient, neon cyan links, and neon glow border in `docusaurus/src/css/custom.css`
- [X] T011 [US2] Update chat toggle button and primary actions to neon cyan in `docusaurus/src/components/ChatWidget/ChatWidget.css`
- [X] T012 [US2] Update chat header and user messages to neon purple theme in `docusaurus/src/components/ChatWidget/ChatWidget.css`
- [X] T013 [US2] Apply neon cyan/purple gradient and glow to "Ask AI" popup in `docusaurus/src/components/ChatWidget/TextSelectionPopup.css`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently.

---

## Phase 5: User Story 3 - Light/Dark Theme Support (Priority: P2)

**Goal**: Ensure the neon theme is readable and aesthetically pleasing in both light and dark modes.

**Independent Test**: Toggle between light and dark modes and verify readability of all neon elements.

### Implementation for User Story 3

- [X] T014 [US3] Add light mode specific contrast adjustments for neon cyan in `docusaurus/src/css/custom.css`
- [X] T015 [US3] Verify visual accessibility and contrast for neon elements in both themes.

**Checkpoint**: All user stories should now be independently functional.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T016 [P] Audit all interactive elements for smooth animations and neon glow effects.
- [X] T017 [P] Perform final cross-browser visual check against all Success Criteria (SC-001 to SC-005).
- [X] T018 Run `quickstart.md` validation.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately.
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories.
- **User Stories (Phase 3+)**: All depend on Foundational phase completion.
- **Polish (Final Phase)**: Depends on all desired user stories being complete.

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories.
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories.
- **User Story 3 (P3)**: Can start after US2 is largely defined to ensure contrast adjustments cover all elements.

### Parallel Opportunities

- T001 and T002 can run in parallel.
- US1 (T004-T006) and US2 (T007-T013) can largely run in parallel as they touch different parts of files or different files entirely.
- Polish tasks T016 and T017 can run in parallel.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Phase 3: User Story 1 (Landing Page Content)
4. **STOP and VALIDATE**: Verify content accuracy.

### Incremental Delivery

1. Foundation ready.
2. Add US1 → Test independently (Content MVP).
3. Add US2 → Test independently (Theme consistency).
4. Add US3 → Test independently (Light/Dark support).

---

## Notes

- [P] tasks = different files or independent logical blocks.
- [Story] label maps task to specific user story for traceability.
- Each user story is independently completable and testable.
- Commit after each task or logical group.
