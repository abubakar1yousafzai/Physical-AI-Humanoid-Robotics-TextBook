# Quickstart: Textbook Content Embedding and Vector Storage System

## Prerequisites

- Python 3.10+
- `uv` package manager installed (`curl -LsSf https://astral.sh/uv/install.sh | sh`)
- Qdrant Cloud account (Free Tier) with API Key and URL.
- Cohere API Key.

## Setup

1.  **Navigate to the backend directory** (create if it doesn't exist, though the script should be run from project root context usually, but code is in `backend/`):
    ```bash
    cd backend
    ```

2.  **Install dependencies**:
    ```bash
    uv sync
    # OR if just running the script directly with uv run
    # (Dependencies are defined in pyproject.toml)
    ```

3.  **Configure Environment**:
    Copy `.env.example` to `.env` and fill in your credentials.
    ```bash
    cp .env.example .env
    ```
    
    Edit `.env`:
    ```ini
    COHERE_API_KEY=your_cohere_key
    QDRANT_URL=your_qdrant_cluster_url
    QDRANT_API_KEY=your_qdrant_key
    DEPLOYED_BOOK_URL=https://textbook-physical-ai-humanoid-robotics.vercel.app
    ```

## Usage

1.  **Run the Indexing Script**:
    ```bash
    uv run main.py
    ```
    This will:
    - Scrape the textbook.
    - Chunk the content.
    - Generate embeddings.
    - Upload to Qdrant.

2.  **Verify**:
    The script should output the number of chunks indexed. You can also inspect your Qdrant dashboard to see the `physical-ai-textbook` collection.

## Troubleshooting

- **401 Unauthorized**: Check your API keys in `.env`.
- **Connection Error**: Check your internet connection and `QDRANT_URL`.
- **Rate Limit**: If the script fails due to API limits, wait and retry. The script does not currently implement auto-backoff for critical failures, but simple retries might work.
