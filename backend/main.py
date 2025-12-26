import os
import sys
import logging
from typing import List, Dict, Optional
import time
import uuid
import argparse
import requests
from bs4 import BeautifulSoup
import tiktoken

from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct

# --- Configuration & Logging ---

COLLECTION_NAME = "physical-ai-textbook"

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
    try:
        if not qdrant_client.collection_exists(COLLECTION_NAME):
            logger.info(f"Creating collection {COLLECTION_NAME}...")
            qdrant_client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=VectorParams(size=1024, distance=Distance.COSINE)
            )
            logger.info(f"Collection {COLLECTION_NAME} created.")
        else:
            logger.info(f"Collection {COLLECTION_NAME} already exists.")
    except Exception as e:
        logger.error(f"Failed to create collection: {e}")
        sys.exit(1)

def save_chunk_to_qdrant(chunk_id: str, vector: List[float], payload: Dict):
    """Saves a single chunk embedding and metadata to Qdrant with retry logic."""
    max_retries = 3
    for attempt in range(max_retries):
        try:
            qdrant_client.upsert(
                collection_name=COLLECTION_NAME,
                points=[
                    PointStruct(
                        id=chunk_id,
                        vector=vector,
                        payload=payload
                    )
                ]
            )
            return
        except Exception as e:
            if attempt == max_retries - 1:
                logger.error(f"Failed to save chunk {chunk_id} after {max_retries} attempts: {e}")
                # Don't exit, just log failure for this chunk
            else:
                logger.warning(f"Error saving chunk {chunk_id}, retrying ({attempt + 1}/{max_retries})...")
                time.sleep(2 ** attempt) # Exponential backoff

def verify_indexing():
    """Verifies the number of vectors stored in Qdrant."""
    try:
        count = qdrant_client.count(collection_name=COLLECTION_NAME).count
        logger.info(f"Total vectors in collection '{COLLECTION_NAME}': {count}")
        return count
    except Exception as e:
        logger.error(f"Failed to verify indexing: {e}")
        return 0

def test_retrieval(query: str):
    """Tests retrieval with a sample query."""
    try:
        logger.info(f"Testing retrieval for query: '{query}'")
        
        # 1. Embed query
        query_embedding = cohere_client.embed(
            texts=[query],
            model="embed-english-v3.0",
            input_type="search_query"
        ).embeddings[0]
        
        # 2. Search Qdrant
        results = qdrant_client.query_points(
            collection_name=COLLECTION_NAME,
            query=query_embedding,
            limit=5
        ).points
        
        logger.info("Search Results:")
        for res in results:
            title = res.payload.get("title", "Unknown")
            score = res.score
            chunk_preview = res.payload.get("text", "")[:100] + "..."
            logger.info(f"  - [{score:.4f}] {title}: {chunk_preview}")
            
    except Exception as e:
        logger.error(f"Failed to test retrieval: {e}")

# --- Main Execution ---

def run_indexing():
    """Runs the complete indexing pipeline."""
    create_collection()
    
    base_url = os.getenv("DEPLOYED_BOOK_URL")
    logger.info(f"Scanning {base_url}...")
    urls = get_all_urls(base_url)
    
    if not urls:
        logger.error("No URLs found to process. Exiting.")
        return

    total_chunks = 0
    
    for url in urls:
        logger.info(f"Processing {url}...")
        data = extract_text_from_url(url)
        
        if not data:
            continue
            
        chunks = chunk_text(data['text'])
        logger.info(f"  - Generated {len(chunks)} chunks.")
        
        if not chunks:
            continue
            
        embeddings = embed(chunks)
        
        if not embeddings or len(embeddings) != len(chunks):
            logger.error(f"  - Failed to generate embeddings for {url}")
            continue
            
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            chunk_id = str(uuid.uuid4())
            payload = {
                "module_id": data['module_id'],
                "chapter_id": data['chapter_id'],
                "title": data['title'],
                "url": data['url'],
                "content_type": data['content_type'],
                "document_type": data['document_type'],
                "chunk_index": i,
                "text": chunk
            }
            
            save_chunk_to_qdrant(chunk_id, embedding, payload)
            total_chunks += 1
            
        # Polite delay between pages
        time.sleep(0.5)

    logger.info(f"Indexing complete. Total chunks processed: {total_chunks}")

def main():
    parser = argparse.ArgumentParser(description="Textbook Content Embedding System")
    parser.add_argument("--run", action="store_true", help="Run full indexing pipeline")
    parser.add_argument("--verify", action="store_true", help="Verify vector count in DB")
    parser.add_argument("--test-query", type=str, help="Test retrieval with a specific query")
    
    args = parser.parse_args()
    
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
    
    if args.run:
        run_indexing()
        
    if args.verify:
        verify_indexing()
        
    if args.test_query:
        test_retrieval(args.test_query)
        
    if not (args.run or args.verify or args.test_query):
        parser.print_help()
        logger.info("No action specified. Use --run, --verify, or --test-query.")

if __name__ == "__main__":
    main()