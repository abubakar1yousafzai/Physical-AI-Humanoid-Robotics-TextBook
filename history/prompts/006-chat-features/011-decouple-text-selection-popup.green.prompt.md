---
id: 011
title: Decouple Text Selection Popup using Custom Events
stage: green
date: 2025-12-30
surface: agent
model: gemini
feature: 006-chat-features
branch: 006-chat-features
user: User
command: update-text-selection-popup-events
labels: ["refactor", "events", "chat"]
links:
  spec: specs/006-chat-features/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - docusaurus/src/components/ChatWidget/TextSelectionPopup.jsx
 - docusaurus/src/components/ChatWidget/ChatWidget.jsx
tests:
 - null
---

## Prompt

Update TextSelectionPopup.jsx file:

In the handleClick function, instead of calling onAskAI(selection), dispatch a custom event:

Replace the onAskAI call with:

if (selection) {
  const event = new CustomEvent('askAI', { 
    detail: { text: selection } 
  });
  window.dispatchEvent(event);
  console.log('askAI event dispatched');
  
  setSelection(null);
  window.getSelection().removeAllRanges();
}

Also remove the onAskAI prop check - just check if selection exists.

## Response snapshot

Decoupled `TextSelectionPopup` from `ChatWidget` by using a custom `askAI` event instead of a callback prop. Updated `handleClick` to dispatch the event and removed the unused `onAskAI` prop from both the component signature and its instantiation in `ChatWidget`.

## Outcome

- ✅ Impact: Improved component decoupling and resolved potential prop-drilling/context issues for triggering the chat widget.
- 🧪 Tests: Manual verification that clicking the "Ask AI" button still opens the chat with the correct context.
- 📁 Files: docusaurus/src/components/ChatWidget/TextSelectionPopup.jsx, docusaurus/src/components/ChatWidget/ChatWidget.jsx
- 🔁 Next prompts: /sp.verify
- 🧠 Reflection: Custom events are an effective way to communicate between independent components in a React app, especially for global UI elements like a chat widget.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
