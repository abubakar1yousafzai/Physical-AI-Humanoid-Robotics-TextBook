# Quickstart: Chat Widget Development

## Prerequisites
1. **Node.js**: v18+ installed.
2. **Backend**: Running locally at `http://localhost:8000`.
   - Refer to Spec-3 or `backend/README.md` for setup.

## Setup
1. **Navigate to Docusaurus root**:
   ```bash
   cd docusaurus
   ```
2. **Install dependencies** (if any new ones are added, but standard `npm install` works):
   ```bash
   npm install
   ```

## Running Locally
1. **Start Docusaurus**:
   ```bash
   npm start
   ```
   Access at `http://localhost:3000`.

2. **Verify Widget**:
   - Look for the floating button in the bottom-right corner.
   - Open console to check for errors.

## Development Workflow
1. **Edit Components**:
   - Files located in `src/components/ChatWidget/`.
   - `ChatWidget.jsx` is the entry point.
2. **Styling**:
   - Edit `ChatWidget.css`.
   - Docusaurus hot-reloads CSS changes.
3. **Mocking Backend**:
   - If backend is offline, you can mock `sendMessage` in `ChatWidget.jsx` to return a resolved Promise with dummy data.

## Testing
- **Manual**: Open widget, type message, check response.
- **Responsiveness**: Use browser dev tools to simulate mobile view.
