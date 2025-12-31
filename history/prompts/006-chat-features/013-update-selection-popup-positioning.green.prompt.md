---
id: 013
title: Update TextSelectionPopup positioning to fixed
stage: green
date: 2025-12-30
surface: agent
model: gemini
feature: 006-chat-features
branch: 006-chat-features
user: User
command: fix-selection-popup-positioning
labels: ["ui", "fix", "chat"]
links:
  spec: specs/006-chat-features/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - docusaurus/src/components/ChatWidget/TextSelectionPopup.jsx
 - docusaurus/src/components/ChatWidget/TextSelectionPopup.css
tests:
 - null
---

## Prompt

Update TextSelectionPopup.jsx position calculation:

In the handleSelectionChange function where position is calculated (around line 25-26), update it to use fixed positioning instead of absolute:

Change:
const top = rect.top + window.scrollY - 40; 
const left = rect.left + window.scrollX + (rect.width / 2);

To:
const top = rect.top - 50; // Use viewport coordinates
const left = rect.left + (rect.width / 2);

This will position the popup relative to the viewport instead of the document, so it will work correctly when scrolling.

Also update the CSS file TextSelectionPopup.css:
Change position from 'absolute' to 'fixed' in the .text-selection-popup class.

## Response snapshot

Updated `TextSelectionPopup.jsx` to calculate coordinates relative to the viewport, enabling correct positioning when used with `position: fixed`. The CSS already utilized `fixed` positioning from a previous update, but the JSX was still accounting for scroll offsets, causing incorrect placement on scrolled pages.

## Outcome

- ✅ Impact: Fixed the "Ask AI" popup positioning issue on scrolled pages.
- 🧪 Tests: Manual verification on scrolled content.
- 📁 Files: docusaurus/src/components/ChatWidget/TextSelectionPopup.jsx
- 🔁 Next prompts: /sp.verify
- 🧠 Reflection: Viewport-relative coordinates (`getBoundingClientRect`) are the correct choice when the element itself is `position: fixed`.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
