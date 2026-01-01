# Feature Specification: Landing Page Content Updates & Theme Fixes

**Feature Branch**: `009-landing-page-theme-updates`  
**Created**: 2026-01-01  
**Status**: Draft  
**Input**: User description: "Fix incorrect content on landing page and apply neon theme consistently across all UI elements including sidebar, navbar, footer, chat widget, and text selection popup."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Correct Landing Page Content (Priority: P1)

User visits the landing page to get an overview of the curriculum and expects to see accurate information about the course content, including the correct number of modules and chapters.

**Why this priority**: Accurate content is essential for user trust and provides a correct representation of the educational material.

**Independent Test**: Can be tested by visiting the landing page and verifying all numbers and titles against the curriculum source of truth.

**Acceptance Scenarios**:

1. **Given** user is on the landing page, **When** viewing the hero section, **Then** the subtitle is "Master the Future of Robotics and Embodied Intelligence".
2. **Given** user scrolls to the stats section, **When** viewing metrics, **Then** it shows 6 Modules, 23 Chapters, 1000+ Concepts, and 5000+ AI Responses.
3. **Given** user views the modules section, **When** checking cards, **Then** all 6 modules are displayed with their specific titles and chapter counts.

---

### User Story 2 - Consistent Neon Theme (Priority: P1)

User navigates through different parts of the application (book, chat, landing page) and expects a visually cohesive experience using a neon cyan and purple theme, without any clashing colors like the default green.

**Why this priority**: Visual consistency and professional branding are key to the user experience and perceived quality of the textbook.

**Independent Test**: Can be tested by navigating all UI components (navbar, sidebar, footer, chat, popups) and verifying the absence of green and consistent use of neon cyan/purple.

**Acceptance Scenarios**:

1. **Given** user is browsing the textbook, **When** hovering over sidebar links or the book title, **Then** the highlight color is neon cyan, not green.
2. **Given** user selects text, **When** the "Ask AI" popup appears, **Then** it uses the neon cyan/purple theme.
3. **Given** user opens the chat widget, **When** interacting with messages and buttons, **Then** all elements match the neon glow aesthetic.

---

### User Story 3 - Theme Mode Support (Priority: P2)

User prefers either a light or dark interface and expects the neon theme to adapt appropriately to ensure readability and high contrast in both modes.

**Why this priority**: Accessibility and user preference for different lighting conditions.

**Independent Test**: Toggle between light and dark modes and verify that all neon elements remain legible and aesthetically pleasing.

**Acceptance Scenarios**:

1. **Given** user toggles to light mode, **When** viewing neon text or buttons, **Then** colors are adjusted to maintain contrast against a light background.
2. **Given** user toggles to dark mode, **When** viewing the interface, **Then** the neon glow effects are prominent against the dark background.

### Edge Cases

- **Screen Responsiveness**: Ensure module cards and stats layout correctly on mobile devices without text overflow.
- **Dynamic Content**: Verify that the typing effect in the hero section correctly handles the new longer subtitle.
- **Animation Performance**: Ensure smooth transitions and neon glow animations do not cause performance lag on lower-end devices.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Hero subtitle MUST be updated to "Master the Future of Robotics and Embodied Intelligence".
- **FR-002**: Stats section MUST reflect exactly: 6 Modules, 23 Chapters, 1000+ Concepts, 5000+ AI Responses.
- **FR-003**: Module cards MUST display exactly 6 items as defined in the curriculum (Introduction to Physical AI, ROS 2 Fundamentals, Robot Simulation with Gazebo, Introduction to the NVIDIA Isaac Platform, Introduction to Humanoid Robot Development, Conversational Robotics).
- **FR-004**: Each module card MUST have a unique, relevant description instead of generic placeholder text.
- **FR-005**: All occurrences of Docusaurus default green (#2e8555) MUST be replaced with neon cyan (#00fff2) or appropriate neon theme colors.
- **FR-006**: The Navbar title MUST feature a linear gradient from neon cyan (#00fff2) to neon purple (#bf00ff).
- **FR-007**: Sidebar menu highlights and hover states MUST use neon cyan (#00fff2).
- **FR-008**: The "Ask AI" text selection popup MUST be styled with neon cyan/purple backgrounds and white text.
- **FR-009**: The Chat Widget MUST use neon cyan for the primary toggle and send buttons, and neon purple for user messages/headers.
- **FR-010**: The Footer MUST feature neon cyan links and a top border with a neon glow effect.

### Key Entities *(include if feature involves data)*

- **Module**: Represents a high-level learning unit with a title, chapter count, and description.
- **Theme Configuration**: Global styling variables defining the neon color palette and glow effects.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% accuracy of landing page stats (6 modules, 23 chapters).
- **SC-002**: Zero instances of default green color in the entire application UI.
- **SC-003**: Consistent application of neon cyan (#00fff2) and neon purple (#bf00ff) across 5+ distinct UI areas (Navbar, Sidebar, Footer, Chat, Popups).
- **SC-004**: Contrast ratios in both Light and Dark modes meet accessibility standards for all neon text elements.
- **SC-005**: Hero typing animation completes without errors using the updated subtitle.