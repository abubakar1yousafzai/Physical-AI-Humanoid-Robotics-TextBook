# Implementation Plan - Enhanced Chat Loading States

## Technical Context
**Feature**: Enhanced Chat Loading States
**Specs**: `specs/011-chat-loading-states/spec.md`
**Branch**: `011-chat-loading-states`

## Constitution Check
- **Educational Excellence**: N/A (UI feature)
- **Interactive Learning**: Improves user engagement with the chatbot.
- **Phased Development**: Aligned with Phase 2 (RAG Chatbot).
- **Content Structure**: N/A
- **Technical Accuracy**: Code must be performant (CSS animations).
- **Accessibility**: Must include `aria-label` for screen readers.
- **Performance**: Use CSS animations, no heavy JS libraries.

## Implementation Strategy

### Overview
This plan focuses on replacing the static "Thinking..." text in the chat widget with a polished, animated 3-dot loading indicator that matches the project's neon theme. The implementation will be purely CSS-driven for performance and will ensure accessibility standards are met.

### Implementation Phases

#### Phase 1: CSS Animation Development
**Goal**: Create the visual styles and keyframes for the bouncing dots.
- **Files**: `frontend/src/components/ChatWidget/ChatWidget.css`
- **Actions**:
    - Remove old `.typing-indicator` text styles.
    - Add `.typing-indicator` container styles.
    - Add `.typing-indicator span` styles for the dots (width, height, background color/gradient, border-radius).
    - Define `@keyframes bounce` for the animation.
    - Add `animation-delay` for staggered effect.

#### Phase 2: Component Integration
**Goal**: Update the React component to render the new structure.
- **Files**: `frontend/src/components/ChatWidget/ChatWidget.jsx`
- **Actions**:
    - Locate the `loading` conditional rendering block.
    - Replace the text "Thinking..." with the 3-span structure.
    - Add `aria-label="Thinking"` to the container.

#### Phase 3: Testing & Polish
**Goal**: Verify smoothness and accessibility.
- **Actions**:
    - Verify 60fps performance.
    - check screen reader announcement.
    - Adjust colors to match `var(--neon-cyan)` and `var(--neon-purple)`.

### Detailed Design & Code Examples

#### CSS Implementation (`ChatWidget.css`)

```css
/* Container for the dots */
.typing-indicator {
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 8px 12px;
  background: rgba(255, 255, 255, 0.05);
  border-radius: 12px;
  width: fit-content;
  margin-top: 4px;
}

/* Individual dots */
.typing-indicator span {
  width: 8px;
  height: 8px;
  background: var(--neon-cyan);
  border-radius: 50%;
  display: inline-block;
  animation: bounce 1.4s infinite ease-in-out both;
  box-shadow: 0 0 10px var(--neon-cyan);
}

/* Staggered delays */
.typing-indicator span:nth-child(1) {
  animation-delay: -0.32s;
}
.typing-indicator span:nth-child(2) {
  animation-delay: -0.16s;
}

/* Keyframes */
@keyframes bounce {
  0%, 80%, 100% { 
    transform: scale(0);
    opacity: 0.5;
  }
  40% { 
    transform: scale(1);
    opacity: 1;
  }
}
```

#### JSX Implementation (`ChatWidget.jsx`)

```jsx
{loading && (
  <div className="chat-message assistant-message">
    <div className="typing-indicator" aria-label="Thinking">
      <span></span>
      <span></span>
      <span></span>
    </div>
  </div>
)}
```

### Testing Checklist

- [ ] **Visual**: Dots appear immediately when a message is sent.
- [ ] **Visual**: Dots follow the cyan/neon theme and have a glow effect.
- [ ] **Visual**: Animation is smooth (staggered bounce) and loops correctly.
- [ ] **Functional**: Indicator disappears when the response arrives.
- [ ] **Accessibility**: VoiceOver/NVDA announces "Thinking" or "Loading".
- [ ] **Responsiveness**: Looks correct on mobile (correct padding/size).

