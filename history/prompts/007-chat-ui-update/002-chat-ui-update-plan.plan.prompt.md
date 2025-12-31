---
id: 002
title: Chat UI Update Plan
stage: plan
date: 2025-12-31
surface: agent
model: gemini
feature: 007-chat-ui-update
branch: 007-chat-ui-update
user: User
command: /sp.plan
labels: ["plan", "design", "ui", "chat"]
links:
  spec: specs/007-chat-ui-update/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/007-chat-ui-update/plan.md
 - specs/007-chat-ui-update/research.md
 - specs/007-chat-ui-update/data-model.md
 - specs/007-chat-ui-update/quickstart.md
tests:
 - null
---

## Prompt

/sp.plan 

# Chat Widget UI Updates: Plan

## Architecture
```
ChatWidget
├── HistoryPanel
│   └── Item: [Content] [⋮ Menu]
│       └── Dropdown: Delete
└── Chat: [Close] Assistant [History]
    └── Message: Content [Copy SVG]
```

**Delete Flow:** Click ⋮ → Confirm → localStorage clean → UI update → (if active) new chat

---

## Phases

### Phase 1: Icons (Visual only, low risk)
- Remove assistant avatar
- Replace copy icon → SVG
- Replace history icon → SVG
- CSS updates

### Phase 2: Layout (DOM changes, low risk)
- Swap header buttons
- CSS alignment

### Phase 3: Delete (New feature, medium risk)
- Add 3-dot menu
- Dropdown + state
- Click-outside handler
- Delete handlers (Panel + Widget)
- Connect props
- CSS

---

## Key Decisions

| Decision | Options | Choice | Why |
|----------|---------|--------|-----|
| **Icons** | A) Inline SVG<br>B) Icon lib<br>C) Components | A | No deps, simple |
| **Confirm** | A) `window.confirm()`<br>B) Custom modal<br>C) Toast+undo | A | Simple, works |
| **Menu** | A) Hover+mobile<br>B) Always visible<br>C) Hover only | A | Clean + mobile-friendly |
| **Active Delete** | A) New chat<br>B) Close panel<br>C) Load previous | A | Matches existing pattern |

---

## Testing Strategy

### Visual Checks
- [ ] No emojis, all SVG
- [ ] Copy = overlapping rectangles
- [ ] History = file document
- [ ] Close left, History right
- [ ] No assistant avatar

### Functional Checks
- [ ] Menu shows on hover
- [ ] Dropdown opens/closes
- [ ] Confirm dialog appears
- [ ] localStorage cleaned (both keys)
- [ ] Active delete → new chat
- [ ] Copy/history/selection still work

### Edge Cases
- [ ] Delete only conversation
- [ ] Delete while loading
- [ ] Rapid clicks
- [ ] Click outside closes menu

### localStorage Validation
**Before:** `chat_history` has item, `chat_thread_[ID]` exists  
**After:** Both removed, UI updated

---

## Technical Details

**Icons:** 
- Copy: 18×18, overlapping rectangles
- History: 20×20, file with fold
- Delete: 16×16, trash can
- All: `currentColor`, `strokeWidth="2"`

**State:** `openMenuId` (single menu open)

**Props:** `ChatWidget` → `onDeleteConversation` → `HistoryPanel`

**localStorage Keys:**
- `chat_history` (array)
- `chat_thread_[ID]` (messages)
- `chat_thread_id` (active)

---

## Quality Checks

**Per Phase:**
1. Test in isolation
2. Commit if pass
3. Integration test
4. Move to next

**Rollback:**
- Phase 1-2 fail → revert to emojis
- Phase 3 fail → remove delete, keep icons
- Data corrupt → clear localStorage

**Final:**
- Cross-browser (Chrome, Firefox, Safari)
- Mobile (iOS, Android)
- All checkboxes

## Response snapshot

Generated implementation plan, research, data model, and quickstart guide for chat UI updates. Plan covers 3 phases: Icons, Layout, Delete. Validated against Constitution (Performance, Accessibility). Defined clear data flow for delete operation and testing strategy.

## Outcome

- ✅ Impact: Established technical roadmap for visual refresh and delete feature.
- 🧪 Tests: Defined visual and functional checks in plan and quickstart.
- 📁 Files: Plan, Research, Data Model, Quickstart.
- 🔁 Next prompts: /sp.tasks
- 🧠 Reflection: Phased approach allows for safe, incremental UI updates. Inline SVGs minimize dependencies.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
