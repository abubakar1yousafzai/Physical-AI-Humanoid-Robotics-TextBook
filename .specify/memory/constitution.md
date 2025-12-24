<!--
Sync Impact Report:
- Version change: 0.1.0 -> 1.0.0
- List of modified principles: All principles have been replaced.
- Added sections: All sections have been replaced.
- Removed sections: All sections have been replaced.
- Templates requiring updates:
  - ✅ .specify/templates/plan-template.md
  - ✅ .specify/templates/spec-template.md
  - ✅ .specify/templates/tasks-template.md
- Follow-up TODOs: None
-->

# Constitution for Physical AI & Humanoid Robotics

| Version | Ratification Date | Last Amended Date |
|---|---|---|
| 1.0.0 | 2025-12-14 | 2025-12-14 |

## 1. Governance and Amendment

This constitution is the supreme governing document for the **Physical AI & Humanoid Robotics** project. It defines the core principles, standards, and constraints that guide our development.

- **Amendment Process**: Amendments require team consensus.
- **Versioning**: Changes follow Semantic Versioning (Major, Minor, Patch).
- **Compliance**: All project activities must comply with this constitution.

## 2. Core Principles

### Principle 1: Educational Excellence
- **Rule**: Content must be technically accurate, pedagogically sound, and progressively structured from fundamentals to advanced concepts.
- **Rationale**: To ensure the educational material is effective and provides a solid learning foundation.

### Principle 2: Interactive Learning
- **Rule**: Every chapter must include code examples, visual diagrams, practical exercises, and end-of-chapter quizzes (5-10 questions).
- **Rationale**: To engage users and reinforce learning through hands-on experience.

### Principle 3: Phased Development
- **Rule**: Phase 1 (Textbook) must be complete before Phase 2 (RAG Chatbot), then Phase 3 (Better-Auth), then Phase 4 (Bonus Features).
- **Rationale**: To ensure a structured and incremental development process.

### Principle 4: Content Structure
- **Rule**: Each chapter requires (1) Title & Overview with learning objectives, (2) Main content with examples, (3) Visual elements, (4) Interactive quiz, (5) Summary, (6) Additional resources.
- **Rationale**: To maintain a consistent and organized structure across all content.

### Principle 5: Technical Accuracy
- **Rule**: All code examples must be tested and runnable, commands verified, and version numbers specified.
- **Rationale**: To provide reliable and accurate technical information to the users.

### Principle 6: Accessibility
- **Rule**: The platform must be mobile-responsive, use gender-neutral language, and culturally sensitive examples.
- **Rationale**: To make the content accessible and inclusive for a diverse audience.

### Principle 7: Performance
- **Rule**: The website must have fast loading times, no broken links, and be cross-browser compatible.
- **Rationale**: To provide a smooth and professional user experience.

## 3. Key Standards

- **Course Coverage**: The course is organized into 6 modules.
- **Book Structure**:
    - PREFACE: Welcome and book overview
    - MODULE 1: Introduction to Physical AI (4 chapters)
    - MODULE 2: ROS 2 Fundamentals (4 chapters)
    - MODULE 3: Robot Simulation with Gazebo (4 chapters)
    - MODULE 4: NVIDIA Isaac Platform (4 chapters)
    - MODULE 5: Humanoid Robot Development (4 chapters)
    - MODULE 6: Conversational Robotics (3 chapters)
- **Module/Chapter Structure**: Each module has an introduction page. Each chapter is a separate page with detailed content and a quiz.
- **File Structure**: Content is organized in a module-based structure within the `docs/` directory.
- **Navigation**: A sidebar allows users to expand modules to see chapters.
- **Code Standards**: All code must include syntax highlighting, comments, and installation instructions.
- **Terminology**: Technical terms must be defined on their first use.

## 4. Constraints

- **Deployment**: The project will be deployed on GitHub Pages or Vercel.
- **Quiz Format**: Quizzes will consist of 5-10 multiple-choice questions per chapter.
- **Mobile Compatibility**: All features must be fully functional on mobile devices.
- **Security**: HTTPS will be enforced, and the application will be protected against XSS/CSRF attacks with proper input validation.

## 5. Phase Requirements

### Phase 1 - Textbook
- Complete PREFACE and all 6 module introduction pages.
- All 25 chapters are fully documented with content and quizzes.
- All code examples are tested and working.
- Sidebar navigation is correctly configured.
- The site is successfully deployed.

### Phase 2 - RAG Chatbot Integration

**Technology Stack:**
- **AI/LLM**: OpenAI Agents SDK (Assistants API)
- **Backend**: FastAPI (Python)
- **Database**: Neon Serverless Postgres (conversation history)
- **Vector Database**: Qdrant Cloud Free Tier (textbook embeddings)
- **Embeddings**: Cohere `embed-english-v3.0`
- **Chat Interface**: React component in Docusaurus

**Architecture:**
1. **Indexing**: Book content → Cohere Embeddings → Qdrant DB
2. **Query Flow**: User → FastAPI → Qdrant (retrieve) → OpenAI Agent → Response
3. **History**: FastAPI → Neon Postgres (save conversations)

**Key Features:**
1. **General chatbot** for textbook questions.
2. **Text selection-based queries**:
   - User highlights text in chapter.
   - Floating button appears near selection.
   - Click to ask question about selected text.
3. **Floating chat widget** (bottom-right corner, all pages).
4. **Conversation history tracking** in Neon Postgres.
5. **Anonymous chat** (without login) with limited history.
6. **Context-aware responses** using RAG pipeline.

**Technical Implementation:**
- **Qdrant** stores all book content as vector embeddings.
- **FastAPI** orchestrates: Frontend ↔ Qdrant ↔ OpenAI Agent ↔ Neon DB.
- **Text selection**: `window.getSelection()` + floating button UI.
- **Chat widget**: Minimizable popup (bottom-right).
- **Session storage**: Browser (anonymous) or Neon DB (logged in).

### Phase 3 - Better-Auth
- Secure signup/signin with email verification.
- A user background questionnaire (7 questions).
- User profiles with progress tracking.
- JWT for session management.

### Phase 4 - Bonus Features
- Personalization button to adjust content based on user background.
- Urdu translation button with RTL support.

## 6. Success Criteria

- PREFACE and all 6 modules are documented.
- All chapters include quizzes.
- The website has a professional appearance and functionality.
- There are zero broken links or console errors.
- The application is mobile-responsive across all devices.
- The site is successfully deployed and accessible via a public URL.