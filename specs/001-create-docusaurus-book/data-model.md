# Data Model for Create Docusaurus Book

This document outlines the data entities for the interactive textbook.

## Entities

### Book
The main entity representing the entire textbook.

- **Attributes**:
    - `title`: The title of the book.
    - `author`: The author of the book.

### Preface
A single entity for the book's introduction.

- **Attributes**:
    - `title`: The title of the preface.
    - `content`: The content of the preface.

### Module
An entity representing a module of the book.

- **Attributes**:
    - `id`: A unique identifier for the module (e.g., `module-01`).
    - `title`: The title of the module.
    - `introduction`: An introductory text for the module.
- **Relationships**:
    - Has many `Chapter`s.

### Chapter
An entity representing a chapter of the book.

- **Attributes**:
    - `id`: A unique identifier for the chapter (e.g., `chapter-01`).
    - `title`: The title of the chapter.
    - `content`: The main content of the chapter.
    - `readingTime`: An estimated reading time for the chapter.
- **Relationships**:
    - Belongs to a `Module`.
    - Has one `Quiz`.

### Quiz
An entity representing a quiz.

- **Attributes**:
    - `id`: A unique identifier for the quiz.
- **Relationships**:
    - Belongs to a `Chapter`.
    - Has many `Question`s.

### Question
An entity representing a quiz question.

- **Attributes**:
    - `id`: A unique identifier for the question.
    - `text`: The text of the question.
    - `options`: A list of possible answers.
    - `correctAnswer`: The correct answer from the options.
    - `explanation`: An explanation for the correct answer.
- **Relationships**:
    - Belongs to a `Quiz`.
