---
id: 010
title: Add Custom Event Listener for Chat Trigger
stage: green
date: 2025-12-30
surface: agent
model: gemini
feature: 006-chat-features
branch: 006-chat-features
user: User
command: add-ask-ai-event-listener
labels: ["react", "events", "chat"]
links:
  spec: specs/006-chat-features/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - docusaurus/src/components/ChatWidget/ChatWidget.jsx
 - docusaurus/src/components/ChatWidget/TextSelectionPopup.jsx
tests:
 - null
---

## Prompt

Update the ChatWidget.js file:

Add a new useEffect hook after the existing useEffects (around line 98) to listen for custom 'askAI' events:

useEffect(() => {
  const handleAskAIEvent = (event) => {
    console.log('askAI event received:', event.detail.text);
    setSelectedText(event.detail.text);
    setIsOpen(true);
  };

  window.addEventListener('askAI', handleAskAIEvent);

  return () => {
    window.removeEventListener('askAI', handleAskAIEvent);
  };
}, []);

This will listen for the custom event from TextSelectionPopup and open the chat when "Ask AI" is clicked.

## Response snapshot

Added the `useEffect` hook to `ChatWidget.jsx` to listen for the custom `askAI` event. Also updated `TextSelectionPopup.jsx` to dispatch this event when the "Ask AI" button is clicked.

## Outcome

- ✅ Impact: Enabled a decoupled event-based trigger for the chat widget from text selection.
- 🧪 Tests: Manual verification by clicking the popup button.
- 📁 Files: docusaurus/src/components/ChatWidget/ChatWidget.jsx, docusaurus/src/components/ChatWidget/TextSelectionPopup.jsx
- 🔁 Next prompts: /sp.verify
- 🧠 Reflection: Event listeners provide a flexible way to trigger component actions across the application without deep prop drilling.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
