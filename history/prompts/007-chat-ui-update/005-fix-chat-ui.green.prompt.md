---
id: 005
title: Fix Chat UI Icons and Layout
stage: green
date: 2025-12-31
surface: agent
model: gemini
feature: 007-chat-ui-update
branch: 007-chat-ui-update
user: User
command: fix-chat-ui
labels: ["bugfix", "ui", "chat", "react"]
links:
  spec: specs/007-chat-ui-update/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - docusaurus/src/components/ChatWidget/ChatWidget.jsx
 - docusaurus/src/components/ChatWidget/ChatMessage.jsx
 - docusaurus/src/components/ChatWidget/ChatWidget.css
tests:
 - null
---

## Prompt

# Chat Widget UI Fix: Icons + Tooltips + Assistant Name

## Changes Required

### 1. Use Clock SVG Icon for History
**Action:** Replace history icon with clock SVG (NOT emoji ⏰)

### 2. Fix Header Layout - Both Buttons RIGHT Side
**Correct:** `Physical AI Assistant [History] [Close]`

### 3. Add Tooltips
- **Copy button:** `title="Copy"`
- **History button:** `title="History"`

### 4. Update Assistant Name
**Change:** "Assistant" → "Physical AI Assistant"

## Response snapshot

Implemented all requested UI fixes:
1.  **Icon Update**: Replaced history icon with the specified clock SVG in `ChatWidget.jsx`.
2.  **Layout Fix**: Updated `ChatWidget.jsx` and `ChatWidget.css` to move both history and close buttons to the right side of the header.
3.  **Tooltips**: Added `title` attributes to the history button (`ChatWidget.jsx`) and copy button (`ChatMessage.jsx`).
4.  **Name Change**: Updated "Assistant" to "Physical AI Assistant" in `ChatWidget.jsx`.

## Outcome

- ✅ Impact: Corrected the chat header layout, standardized the icons, and improved accessibility with tooltips.
- 🧪 Tests: Visual verification needed.
- 📁 Files: `ChatWidget.jsx`, `ChatMessage.jsx`, `ChatWidget.css`
- 🔁 Next prompts: /sp.verify
- 🧠 Reflection: Small UI fixes like these significantly improve polish and usability.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
