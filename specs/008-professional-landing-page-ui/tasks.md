# Tasks: Professional Landing Page UI

**Feature Branch**: `008-professional-landing-page-ui`
**Spec**: [specs/008-professional-landing-page-ui/spec.md](specs/008-professional-landing-page-ui/spec.md)
**Plan**: [specs/008-professional-landing-page-ui/plan.md](specs/008-professional-landing-page-ui/plan.md)

## Phase 1: CSS Foundation (Blocking)
**Goal**: Establish the neon theme variables and animation keyframes required by all other components.

- [X] T001 Define CSS variables for neon colors (cyan, purple, pink) and dark background in `docusaurus/src/css/custom.css`
- [X] T002 Implement `@keyframes` for `neonGlow`, `gradientShift`, and `fadeInUp` in `docusaurus/src/css/custom.css`
- [X] T003 Create utility classes for glassmorphism (`.glass`), neon text (`.neon-text`), and basic layout in `docusaurus/src/css/custom.css`
- [X] T004 Prepare `docusaurus/src/pages/index.js` by removing default Docusaurus components (keeping Layout) to accept new HTML structure

## Phase 2: Hero Section (US1 - P1)
**Goal**: Create the high-impact hero section with animated gradients and typing effects.
**Independent Test**: Verify hero section renders with neon glow and typing effect works on load.

- [X] T005 [US1] Add Hero HTML structure (title, subtitle, buttons) to `docusaurus/src/pages/index.js`
- [X] T006 [US1] Style Hero container with full viewport height and animated gradient background in `docusaurus/src/css/custom.css`
- [X] T007 [US1] Apply neon glow effects to Hero title and CTA buttons in `docusaurus/src/css/custom.css`
- [X] T008 [US1] Implement typing effect logic (vanilla JS) within a `useEffect` or script tag in `docusaurus/src/pages/index.js`
- [X] T009 [US1] Ensure Hero subtitle has a static fallback state (visible text) before animation starts in `docusaurus/src/css/custom.css`

## Phase 3: Module Cards (US2 - P1)
**Goal**: Implement interactive glassmorphism cards for the modules.
**Independent Test**: Verify hover effects (lift + border glow) on module cards.

- [X] T010 [US2] Add HTML structure for 3-column Module Cards grid to `docusaurus/src/pages/index.js`
- [X] T011 [US2] Implement CSS Grid layout for modules (1 col mobile, 3 col desktop) in `docusaurus/src/css/custom.css`
- [X] T012 [US2] Style individual module cards with glassmorphism background (`rgba(255,255,255,0.05)`) in `docusaurus/src/css/custom.css`
- [X] T013 [US2] Add hover interactions (transform Y-axis, neon border color change) in `docusaurus/src/css/custom.css`

## Phase 4: Chat Widget Integration (US4 - P1)
**Goal**: Re-skin the existing chat widget to match the neon theme.
**Independent Test**: Open chat widget and verify colors match the defined neon palette.

- [X] T014 [US4] Inspect and target existing Docusaurus/Infima chat widget selectors in `docusaurus/src/css/custom.css`
- [X] T015 [US4] Override chat widget primary colors and borders to use neon cyan/purple variables in `docusaurus/src/css/custom.css`

## Phase 5: Features Showcase (US3 - P2)
**Goal**: Display features with staggered animation on scroll.
**Independent Test**: Scroll down to Features section and verify elements fade in sequentially.

- [X] T016 [US3] Add Features Grid HTML structure (6 items) to `docusaurus/src/pages/index.js`
- [X] T017 [US3] Style Feature icons and text with neon accents in `docusaurus/src/css/custom.css`
- [X] T018 [US3] Create `.visible` class in `docusaurus/src/css/custom.css` that triggers `fadeInUp` animation

## Phase 6: Stats & Global Animations
**Goal**: Add the Stats section and implement the global scroll observer logic.

- [X] T019 Add Stats Section HTML (4 counters) to `docusaurus/src/pages/index.js`
- [X] T020 Style Stats numbers and labels in `docusaurus/src/css/custom.css`
- [X] T021 Implement `IntersectionObserver` in `docusaurus/src/pages/index.js` to toggle `.visible` class on scroll elements
- [X] T022 Implement numeric counter animation (0 to N) logic in `docusaurus/src/pages/index.js`

## Phase 7: Polish & Edge Cases
**Goal**: Ensure responsiveness and handle edge cases like ultra-wide screens.

- [X] T023 Add media query for Ultra-Wide screens (>1920px) to cap container max-width at 1600px in `docusaurus/src/css/custom.css`
- [X] T024 Verify and fix mobile padding/margins for all sections in `docusaurus/src/css/custom.css`
- [X] T025 Define fallback styles for `no-js` environments (ensure content is visible without opacity: 0) in `docusaurus/src/css/custom.css`

## Dependencies
- Phase 1 blocks all other phases.
- Phase 2, 3, 4, 5, 6 can technically be built in parallel order-wise, but sharing `custom.css` suggests sequential or careful merging.
- Phase 6 (JS) depends on HTML structures from Phases 2, 3, 5, 6.

## Parallel Execution
- T006, T012, T017 (CSS styling) can be done while T008, T021 (JS Logic) are drafted.
