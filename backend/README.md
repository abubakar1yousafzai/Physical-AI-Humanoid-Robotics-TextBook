# Textbook Content Embedding System

This backend script is responsible for scraping the deployed Docusaurus textbook, chunking the content, generating embeddings with Cohere, and storing them in a Qdrant vector database.

## Prerequisites

- Python 3.10+
- `uv` package manager
- API Keys for Cohere and Qdrant

## Setup

1.  **Navigate to this directory**:
    ```bash
    cd backend
    ```

2.  **Install dependencies**:
    ```bash
    uv sync
    ```

3.  **Configure Environment**:
    Create a `.env` file by copying `.env.example` and filling in your API keys.

## Usage

The script is controlled via command-line arguments.

- **Run the full indexing pipeline**:
  This will scrape, chunk, embed, and upload all content.
  ```bash
  uv run python main.py --run
  ```

- **Verify the number of vectors in the database**:
  ```bash
  uv run python main.py --verify
  ```

- **Test retrieval with a sample query**:
  ```bash
  uv run python main.py --test-query "What is physical AI?"
  ```

## Architecture

- **`main.py`**: The core script orchestrating the entire pipeline.
- **`.env`**: Stores API keys and configuration secrets.
- **`pyproject.toml` / `uv.lock`**: Manages Python dependencies.
