# Quickstart: Landing Page & Theme Updates

## 1. Setup
- Ensure you are on branch `009-landing-page-theme-updates`.
- Start docusaurus: `cd docusaurus && npm start`.

## 2. Implementation Order

### Step 1: index.js (Content)
- Open `docusaurus/src/pages/index.js`.
- Update the `modules` array with the 6 modules defined in `spec.md`.
- Update the `stats` array.
- Update the `hero-subtitle` constant in the `useEffect`.

### Step 2: custom.css (Theme)
- Open `docusaurus/src/css/custom.css`.
- Update `:root` variables to remove green.
- Add `.navbar__title` gradient styles.
- Add sidebar and footer neon overrides.
- Add `[data-theme='light']` contrast adjustments for cyan.

### Step 3: Chat & Popup (Component Styles)
- Open `docusaurus/src/components/ChatWidget/ChatWidget.css` and `TextSelectionPopup.css`.
- Apply neon cyan/purple gradients and box-shadows.

## 3. Verification
- Visit `localhost:3000`.
- Verify hero text typing effect.
- Check all 6 module cards.
- Highlight text in any chapter to trigger the "Ask AI" popup.
- Toggle between dark and light modes.
