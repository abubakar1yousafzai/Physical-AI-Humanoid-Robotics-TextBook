import os
import sys
import logging
from typing import List, Dict, Optional
import time
import requests
from bs4 import BeautifulSoup
import tiktoken

from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient

# --- Configuration & Logging ---

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

def load_environment():
    """Loads environment variables and validates required keys."""
    load_dotenv()
    
    required_keys = [
        "COHERE_API_KEY",
        "QDRANT_URL",
        "QDRANT_API_KEY",
        "DEPLOYED_BOOK_URL"
    ]
    
    missing_keys = []
    for key in required_keys:
        if not os.getenv(key):
            missing_keys.append(key)
    
    if missing_keys:
        logger.error(f"Missing required environment variables: {', '.join(missing_keys)}")
        sys.exit(1)
        
    logger.info("Environment variables loaded and validated successfully.")

# --- Global Clients (initialized in main) ---
cohere_client = None
qdrant_client = None

# --- Core Functions ---

def get_all_urls(base_url: str) -> List[str]:
    """Parses sitemap to find all relevant textbook URLs."""
    sitemap_url = f"{base_url.rstrip('/')}/sitemap.xml"
    try:
        response = requests.get(sitemap_url)
        response.raise_for_status()
        
        soup = BeautifulSoup(response.content, 'xml')
        urls = [loc.text for loc in soup.find_all('loc')]
        
        # Filter for relevant content (preface, modules, chapters)
        # CRITICAL: Exclude quiz URLs to prevent revealing answers
        relevant_urls = [
            u for u in urls 
            if any(k in u for k in ['preface', 'module-', 'chapter-'])
            and 'quiz' not in u.lower()
        ]
        
        logger.info(f"Found {len(relevant_urls)} relevant URLs from sitemap.")
        return relevant_urls
    except Exception as e:
        logger.error(f"Failed to fetch sitemap: {e}")
        return []

def extract_text_from_url(url: str) -> Optional[Dict[str, str]]:
    """Fetches and parses a single URL to extract text and metadata."""
    try:
        response = requests.get(url)
        response.raise_for_status()
        
        soup = BeautifulSoup(response.content, 'html.parser')
        
        # Remove navigation, header, footer, etc. specific to Docusaurus
        for unwanted in soup.find_all(['nav', 'footer', 'header', 'script', 'style']):
            unwanted.decompose()
            
        # Extract main content - Docusaurus usually puts content in <main> or class="markdown"
        main_content = soup.find('main') or soup.find('article') or soup.body
        
        if not main_content:
            logger.warning(f"No main content found for {url}")
            return None
            
        text = main_content.get_text(separator=' ', strip=True)
        title = soup.title.string if soup.title else "Untitled"
        
        # Extract simple metadata from URL
        parts = url.split('/')
        module_id = next((p for p in parts if 'module-' in p), 'general')
        chapter_id = next((p for p in parts if 'chapter-' in p), 'overview')
        doc_type = 'chapter' if 'chapter-' in url else ('module_intro' if 'module-' in url else 'preface')
        
        return {
            "url": url,
            "title": title,
            "text": text,
            "module_id": module_id,
            "chapter_id": chapter_id,
            "document_type": doc_type,
            "content_type": "text"
        }
    except Exception as e:
        logger.error(f"Failed to extract content from {url}: {e}")
        return None

def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 100) -> List[str]:
    """Splits text into chunks with overlap using tiktoken encoder."""
    encoding = tiktoken.get_encoding("cl100k_base") # OpenAI encoding, good proxy for English text
    tokens = encoding.encode(text)
    
    chunks = []
    start = 0
    total_tokens = len(tokens)
    
    if total_tokens <= chunk_size:
        return [text]
        
    while start < total_tokens:
        end = min(start + chunk_size, total_tokens)
        chunk_tokens = tokens[start:end]
        chunk_text = encoding.decode(chunk_tokens)
        chunks.append(chunk_text)
        
        if end == total_tokens:
            break
            
        start += (chunk_size - overlap)
        
    return chunks

def embed(texts: List[str]) -> List[List[float]]:
    """Generates embeddings for a list of texts using Cohere."""
    if not texts:
        return []
        
    try:
        # Cohere embed-english-v3.0 dimensions: 1024
        response = cohere_client.embed(
            texts=texts,
            model="embed-english-v3.0",
            input_type="search_document"
        )
        return response.embeddings
    except Exception as e:
        logger.error(f"Failed to generate embeddings: {e}")
        return []

def create_collection():
    """Initializes the Qdrant collection if it doesn't exist."""
    # TODO: Implement T012
    pass

def save_chunk_to_qdrant(chunk_id: str, vector: List[float], payload: Dict):
    """Saves a single chunk embedding and metadata to Qdrant."""
    # TODO: Implement T013
    pass

def verify_indexing():
    """Verifies the number of vectors stored in Qdrant."""
    # TODO: Implement T016
    pass

def test_retrieval(query: str):
    """Tests retrieval with a sample query."""
    # TODO: Implement T017
    pass

# --- Main Execution ---

def main():
    logger.info("Starting Textbook Embedding System...")
    load_environment()
    
    # Initialize Clients
    global cohere_client, qdrant_client
    cohere_client = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )
    
    logger.info("Clients initialized.")
    
    # TODO: Orchestrate pipeline (T014)

if __name__ == "__main__":
    main()