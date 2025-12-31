---
id: 012
title: Prevent selection clearing on popup click
stage: green
date: 2025-12-30
surface: agent
model: gemini
feature: 006-chat-features
branch: 006-chat-features
user: User
command: fix-selection-clear-on-popup-click
labels: ["ux", "react", "chat"]
links:
  spec: specs/006-chat-features/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - docusaurus/src/components/ChatWidget/TextSelectionPopup.jsx
tests:
 - null
---

## Prompt

Update TextSelectionPopup.jsx:

1. Add a ref after line 6:
   const isClickingPopup = useRef(false);

2. Add check at the start of handleSelectionChange (line 10):
   if (isClickingPopup.current) {
     isClickingPopup.current = false;
     return;
   }

3. Add new handleMouseDown function before handleClick:
   const handleMouseDown = (e) => {
     e.preventDefault();
     e.stopPropagation();
     isClickingPopup.current = true;
     console.log('Mouse down on popup');
   };

4. Add onMouseDown handler to the div (line 73):
   onMouseDown={handleMouseDown}

This will prevent the selection from clearing when clicking the popup.

## Response snapshot

Modified `TextSelectionPopup.jsx` to prevent the `mouseup` event listener (which handles selection changes) from clearing the selection state when the user specifically interacts with the "Ask AI" popup. Used a `ref` and `onMouseDown` to track and ignore these specific interactions.

## Outcome

- ✅ Impact: Fixed a race condition where clicking the popup would trigger a "deselect" event before the "click" event could process the selection.
- 🧪 Tests: Manual verification that clicking the "Ask AI" button now successfully triggers the chat widget.
- 📁 Files: docusaurus/src/components/ChatWidget/TextSelectionPopup.jsx
- 🔁 Next prompts: /sp.verify
- 🧠 Reflection: Intercepting `onMouseDown` is a standard pattern for preventing focus/selection loss in custom UI elements.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
