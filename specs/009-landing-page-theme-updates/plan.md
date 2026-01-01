# Implementation Plan - Landing Page Content Updates & Theme Fixes

**Feature Name**: Landing Page Content Updates & Theme Fixes
**Feature Branch**: `009-landing-page-theme-updates`
**Status**: Draft
**Created**: 2026-01-01

## 1. Technical Context

### 1.1 Current System State
- **Framework**: Docusaurus v3.6.3 (Classic template)
- **UI Libraries**: javaScript, Infima (CSS framework)
- **Styling**: `custom.css` for global overrides, CSS Modules for components
- **Key Components**: 
    - `index.js` (Landing page)
    - `ChatWidget` (AI assistant)
    - `TextSelectionPopup` (Context-aware AI query)
    - Sidebar and Navbar (Docusaurus defaults)

### 1.2 Constraints & Invariants
- **Performance**: Must maintain fast loading times (Lighthouse score > 90)
- **Responsiveness**: Must be fully functional and visually sound on mobile (viewport width >= 320px)
- **Consistency**: All UI elements MUST adhere to the neon cyan (#00fff2) and neon purple (#bf00ff) color palette. Green is explicitly forbidden.
- **Theme**: Must support both light and dark modes with proper contrast.

---

## 2. Constitution Check

| Principle | Check | Status |
|---|---|---|
| Educational Excellence | Content updates accurately reflect the 6-module structure. | ✅ |
| Interactive Learning | Neon theme enhances visual engagement. | ✅ |
| Accessibility | Contrast ratios for neon text must be verified for WCAG compliance. | ⚠️ |
| Performance | Smooth animations and gradients must not cause jank. | ✅ |
| Phased Development | Consistent with Phase 2 (Chatbot) and overall project branding. | ✅ |

---

## 3. Research (Phase 0)

### 3.1 Research Tasks
- **R-001**: Verify contrast ratios for `#00fff2` (Neon Cyan) on light and dark backgrounds.
- **R-002**: Identify all Infima variables related to the default green color to ensure complete removal.
- **R-003**: Investigate smooth animation techniques for linear gradients on text and borders.

---

## 4. Design & Contracts (Phase 1)

### 4.1 Data Model
No new database entities. Update local `index.js` data arrays for modules and stats.

### 4.2 Contracts
No new API endpoints.

### 4.3 Quickstart
1. Review `specs/009-landing-page-theme-updates/spec.md`.
2. Apply `custom.css` overrides.
3. Update `index.js` content.
4. Verify chat widget and popup styles.
5. Test light/dark mode toggles.

---

## 5. Implementation Steps (Phase 2)

### 5.1 Step 1: Landing Page Content (index.js)
- Update hero subtitle typing effect.
- Update stats section values and labels.
- Replace module cards array with the 6-module curriculum.

### 5.2 Step 2: Global CSS (custom.css)
- Override Infima green variables.
- Add Navbar title gradient.
- Style Sidebar links (cyan hover).
- Style Footer (dark gradient + neon glow).

### 5.3 Step 3: Component Styling (ChatWidget & Popup)
- Apply neon theme to `ChatWidget.css`.
- Apply neon theme to `TextSelectionPopup.css`.
- Update `TextSelectionPopup.jsx` if inline styles are used.

### 5.4 Step 4: Verification
- Manual visual audit.
- Browser cross-compatibility check.
- Theme toggle testing.

---

## 6. Definition of Done
- [ ] Landing page shows correct 6-module curriculum.
- [ ] Stats show 6 Modules, 23 Chapters, 1000+ Concepts, 5000+ AI Responses.
- [ ] No green colors visible in Navbar, Sidebar, Footer, or Chat.
- [ ] Navbar title has the cyan-to-purple gradient.
- [ ] All animations are smooth.
- [ ] Light and dark modes are fully functional.