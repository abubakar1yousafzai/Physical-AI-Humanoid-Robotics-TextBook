---
id: 1
title: Landing Page Spec Creation
stage: spec
date: 2026-01-01
surface: agent
model: gemini-2.0-flash-thinking-exp
feature: landing-page-theme-updates
branch: 009-landing-page-theme-updates
user: INSPIRED COMP
command: /sp.specify
labels: ["spec", "landing-page", "theme", "neon"]
links:
  spec: specs/009-landing-page-theme-updates/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/009-landing-page-theme-updates/spec.md
  - specs/009-landing-page-theme-updates/checklists/requirements.md
tests:
  - Specification quality checklist validation
---

## Prompt

# Spec-9: Landing Page Content Updates & Theme Fixes

## Overview

Fix incorrect content on landing page and apply neon theme consistently across all UI elements including sidebar, navbar, footer, chat widget, and text selection popup.

---

## User Stories

### US1: Correct Landing Page Content (P1)
**Given** user visits landing page  
**When** viewing hero and stats sections  
**Then** see correct heading, module count (6), and chapter count (23)

**Acceptance:**
- Hero subtitle: "Master the Future of Robotics and Embodied Intelligence"
- Stats show: 6 Modules, 23 Chapters, 1000+ Concepts, 5000+ AI Responses
- All 6 module cards with correct names and chapter counts

---

### US2: Consistent Neon Theme Across App (P1)
**Given** user navigates the app  
**When** viewing any section (sidebar, navbar, footer, chat, text selection)  
**Then** see consistent neon cyan/purple theme (NO green)

**Acceptance:**
- Sidebar links: cyan hover (not green)
- Navbar title: gradient (cyan → purple)
- Footer: dark with neon accents
- Text selection "Ask AI" button: neon cyan/purple
- Chat widget: matches neon theme
- Book title hover: neon cyan (not green)

---

### US3: Light/Dark Theme Support (P2)
**Given** user switches between light and dark themes  
**When** toggling theme  
**Then** colors adjust properly with good contrast

**Acceptance:**
- Dark mode: neon colors on dark background
- Light mode: adjusted neon colors for readability
- No broken colors in either mode

---

## Functional Requirements

### Content Updates

**FR-001:** Hero subtitle MUST be "Master the Future of Robotics and Embodied Intelligence"

**FR-002:** Stats section MUST show:
- 6 Modules
- 23 Chapters  
- 1000+ Concepts
- 5000+ AI Responses

**FR-003:** Module cards MUST display (6 total):

| # | Title | Chapters |
|---|-------|----------|
| 01 | Introduction to Physical AI | 4 |
| 02 | ROS 2 Fundamentals | 4 |
| 03 | Robot Simulation with Gazebo | 4 |
| 04 | Introduction to the NVIDIA Isaac Platform | 4 |
| 05 | Introduction to Humanoid Robot Development | 4 |
| 06 | Conversational Robotics | 3 |

**FR-004:** Each module card MUST have unique description (not generic text)

---

### Theme Fixes

**FR-005:** Sidebar menu links MUST use neon cyan (#00fff2) on hover (remove green)

**FR-006:** Navbar title "Physical AI & Humanoid Robotics" MUST have:
- Gradient: linear-gradient(135deg, #00fff2 0%, #bf00ff 100%)
- Same style as landing page hero title

**FR-007:** Text selection "Ask AI" popup MUST use neon theme:
- Background: neon cyan (#00fff2) or purple (#bf00ff)
- Text: white
- Hover: darker shade
- Remove any green colors

**FR-008:** Chat widget MUST match neon theme:
- Button: neon cyan background
- Panel borders: neon glow
- Headers: gradient text

**FR-009:** Footer MUST use:
- Dark background with gradient
- Neon cyan links
- Border top with neon glow

**FR-010:** All Docusaurus default green colors MUST be replaced with neon theme colors

---

## Success Criteria

- **SC-001:** All content numbers correct (6 modules, 23 chapters)
- **SC-002:** Zero green colors visible anywhere in app
- **SC-003:** Consistent neon cyan/purple/pink theme throughout
- **SC-004:** Light and dark modes both work correctly
- **SC-005:** All interactive elements styled with neon theme
---
ensure smooth animations

## Response snapshot

Created feature specification and quality checklist for "Landing Page Content Updates & Theme Fixes".

## Outcome

- ✅ Impact: Established a clear roadmap for updating landing page content and ensuring visual consistency with a neon theme.
- 🧪 Tests: Validated the specification against the quality checklist (100% pass).
- 📁 Files: Created `specs/009-landing-page-theme-updates/spec.md` and `specs/009-landing-page-theme-updates/checklists/requirements.md`.
- 🔁 Next prompts: /sp.plan
- 🧠 Reflection: The user provided a very detailed description, making the specification process straightforward.

## Evaluation notes (flywheel)

- Failure modes observed: `create-new-feature.ps1` had a positional parameter error but successfully created the branch and directory. `create-phr.ps1` failed to run due to path/alias issues.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Fix PowerShell script pathing in future iterations.
