# Research: Chat Features

## Decisions & Rationale

### 1. Storage Strategy
- **Decision**: Use `localStorage` for conversation history.
- **Rationale**: 
  - Meets the requirement for "anonymous" history (per Constitution Phase 2).
  - Zero infrastructure cost/complexity for MVP.
  - Sufficient for the 50-conversation limit requirement.
- **Alternatives Considered**: 
  - `IndexedDB`: Overkill for simple text messages and limited history depth.
  - `Neon Postgres`: Required for logged-in users, but `localStorage` is chosen for the current anonymous user scope.

### 2. Text Selection
- **Decision**: Use native `window.getSelection()` API.
- **Rationale**: 
  - No external dependencies required.
  - Supported across all target browsers.
  - Provides coordinates for positioning the popup.
- **Implementation Note**: Need to handle "selection finish" detection carefully (e.g., `mouseup` or debounce `selectionchange`) to avoid popup flickering.

### 3. Clipboard Interaction
- **Decision**: Use `navigator.clipboard.writeText()`.
- **Rationale**: 
  - Modern standard for clipboard access.
  - Promise-based, allowing for easy success/error handling (e.g., "Copied!" toast).
- **Fallback**: Fallback to `document.execCommand('copy')` if `navigator.clipboard` is unavailable (legacy support), though most modern browsers support the API.

### 4. Styling
- **Decision**: CSS Modules or separate CSS files imported by components.
- **Rationale**: Consistent with existing Docusaurus/React patterns in the project.

## Unknowns Resolved
- **Security**: No encryption needed for localStorage (per Spec clarifications).
- **Limits**: 50 conversation limit defined in Spec/Plan.
