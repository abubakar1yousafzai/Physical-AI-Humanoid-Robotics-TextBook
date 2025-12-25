import pytest
from main import get_all_urls, extract_text_from_url, chunk_text
import os
from dotenv import load_dotenv

# Load env to get base url
load_dotenv()
BASE_URL = os.getenv("DEPLOYED_BOOK_URL")

@pytest.mark.skipif(not BASE_URL, reason="DEPLOYED_BOOK_URL not set")
def test_get_all_urls():
    urls = get_all_urls(BASE_URL)
    assert len(urls) > 0, "No URLs found"
    assert any("module-01" in u for u in urls), "No module-01 URLs found"
    
    # CRITICAL: Verify no quiz URLs are included
    quiz_urls = [u for u in urls if 'quiz' in u.lower()]
    assert len(quiz_urls) == 0, f"Quiz URLs should be excluded: {quiz_urls}"

def test_chunk_text():
    # Create text with predictable token count
    # Average: 1 word ≈ 1.3 tokens, so 1500 words ≈ 1950 tokens
    text = " ".join(["word"] * 1500)
    
    chunks = chunk_text(text, chunk_size=1000, overlap=100)
    
    assert len(chunks) >= 2, "Should split into at least 2 chunks"
    assert len(chunks) <= 3, "Should not create excessive chunks"
    
    # Verify chunks are not empty
    for chunk in chunks:
        assert len(chunk) > 0, "Chunk should not be empty"

def test_extract_text_structure():
    """Test that extract_text_from_url returns correct structure."""
    if not BASE_URL:
        pytest.skip("BASE_URL not set")
    
    urls = get_all_urls(BASE_URL)
    if not urls:
        pytest.skip("No URLs found")
        
    sample_data = extract_text_from_url(urls[0])
    
    if sample_data:
        # Verify all required fields exist
        required_fields = ["url", "title", "text", "module_id", 
                          "chapter_id", "document_type", "content_type"]
        for field in required_fields:
            assert field in sample_data, f"Missing field: {field}"

if __name__ == "__main__":
    # Simple manual run
    if BASE_URL:
        print("Testing URL extraction...")
        urls = get_all_urls(BASE_URL)
        print(f"Found {len(urls)} URLs")
        
        # Check quiz filtering
        quiz_urls = [u for u in urls if 'quiz' in u.lower()]
        print(f"Quiz URLs (should be 0): {len(quiz_urls)}")
        
        if urls:
            print(f"Sample: {urls[0]}")
            print("Testing extraction on sample...")
            data = extract_text_from_url(urls[0])
            if data:
                print(f"Title: {data.get('title')}")
                print(f"Text length: {len(data.get('text', ''))}")
                
                chunks = chunk_text(data['text'])
                print(f"Generated {len(chunks)} chunks")
    else:
        print("Set DEPLOYED_BOOK_URL to run manual test")