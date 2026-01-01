from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import os
from dotenv import load_dotenv

load_dotenv()

# Initialize
model = SentenceTransformer('all-MiniLM-L6-v2')
qdrant_url = os.getenv('QDRANT_URL')
qdrant_api_key = os.getenv('QDRANT_API_KEY')

if not qdrant_url:
    # Fallback for local development if not in env
    print("Warning: QDRANT_URL not found in env, defaulting to localhost:6333")
    qdrant_url = "http://localhost:6333"

client = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key
)

collection_name = "physical-ai-textbook"

# Step 1: Get existing data
print("📥 Fetching existing chunks from Qdrant...")
try:
    existing_points_result = client.scroll(
        collection_name=collection_name,
        limit=10000, # Increased limit to try to catch more
        with_payload=True,
        with_vectors=False
    )
    existing_points = existing_points_result[0]
except Exception as e:
    print(f"Error fetching points: {e}")
    existing_points = []

print(f"Found {len(existing_points)} chunks")

if not existing_points:
    print("No points found to re-index. Exiting.")
    exit(0)

# Step 2: Recreate collection with new dimensions
print("🔄 Recreating collection with 384 dimensions...")
client.recreate_collection(
    collection_name=collection_name,
    vectors_config=VectorParams(
        size=384,  # Sentence Transformers dimension
        distance=Distance.COSINE
    )
)

# Step 3: Re-embed and upload
print("🔢 Generating new embeddings...")
points = []
for point in existing_points:
    text = point.payload.get('text', '')
    if text:
        vector = model.encode(text).tolist()
        points.append(PointStruct(
            id=point.id,
            vector=vector,
            payload=point.payload
        ))

if points:
    print(f"📤 Uploading {len(points)} points to Qdrant...")
    client.upsert(
        collection_name=collection_name,
        points=points
    )
    print("✅ Re-indexing complete!")
else:
    print("⚠️ No points generated for upload.")
