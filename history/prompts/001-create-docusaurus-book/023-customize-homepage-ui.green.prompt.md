---
id: "002"
title: "Customize Docusaurus homepage UI"
stage: "green"
date: "2025-12-20"
surface: "agent"
model: "gemini-2.0-flash-exp"
feature: "001-create-docusaurus-book"
branch: "main"
user: "user"
command: "replace"
labels:
  - "docusaurus"
  - "ui"
  - "branding"
  - "config"
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "docusaurus/docusaurus.config.js"
  - "docusaurus/src/pages/index.js"
tests: []
---

# Prompt

Customize Docusaurus homepage UI

Action Required:
Update docusaurus/docusaurus.config.js and homepage files to customize branding and layout.

Changes needed:

1. Site Title & Branding:
   - Change site title from "My Site" to "Physical AI & Humanoid Robotics"
   - Remove dinosaur logo from top-left
   - Change "Tutorial" nav link to "Book" (should link to /docs/preface)
   - Remove "Blog" nav link

2. Homepage Hero Section:
   - Change main heading from "My site" to "Physical AI & Humanoid Robotics"
   - Change subtitle from "Dinosaurs are cool" to something like "Master the Future of Robotics and Embodied Intelligence"
   - Change button text from "Docusaurus Tutorial - 5min ⏱️" to "Start Learning →"
   - Button should link to /docs/preface (textbook starting point)

3. Homepage Features Section:
   - REMOVE all three feature cards (Easy to Use, Focus on What Matters, Powered by React)
   - Replace with custom content or leave empty/minimal

4. Additional improvements:
   - Update site tagline
   - Ensure branding is consistent throughout
   - Update footer if needed

Files to modify:
- docusaurus/docusaurus.config.js (main config)
- docusaurus/src/pages/index.js or index.tsx (homepage)
- docusaurus/src/css/custom.css (if styling needed)

Make the homepage professional and focused on the Physical AI textbook content.

# Response

I have updated `docusaurus/docusaurus.config.js` and `docusaurus/src/pages/index.js` to customize the Docusaurus homepage.

**Changes:**
1.  **`docusaurus/docusaurus.config.js`**:
    *   Updated `title` to "Physical AI & Humanoid Robotics".
    *   Updated `tagline` to "Master the Future of Robotics and Embodied Intelligence".
    *   Removed the default logo configuration.
    *   Renamed "Tutorial" navbar item to "Book" and linked it to `/docs/preface`.
    *   Removed the "Blog" navbar item.
    *   Disabled the blog plugin.
    *   Updated the footer links and copyright text.

2.  **`docusaurus/src/pages/index.js`**:
    *   Updated the Hero section button text to "Start Learning →" and linked it to `/docs/preface`.
    *   Removed the `HomepageFeatures` component and its import to remove the feature cards.
    *   Updated the page description metadata.

The homepage is now focused on the textbook content with the updated branding.
