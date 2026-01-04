# Physical AI & Humanoid Robotics Textbook Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-15

## Active Technologies

- Python 3.11
- FastAPI
- OpenAI Agents SDK
- AsyncOpenAI
- Gemini API
- Qdrant Vector Database
- Docusaurus

## Project Structure

```text
.
├── api/
│   ├── src/
│   │   ├── main.py
│   │   └── services/
│   │       ├── rag_service.py
│   │       └── agent_validator.py
│   └── tests/
├── book_source/
│   ├── docs/
│   ├── src/
│   └── docusaurus.config.ts
├── specs/
│   └── 002-agent-validator/
└── history/
    └── prompts/
```

## Commands

- `uvicorn api.src.main:app --reload` - Start the FastAPI backend
- `npm start` - Start the Docusaurus frontend
- `pytest` - Run tests

## Code Style

- Use type hints for all functions
- Follow async/await pattern for FastAPI compatibility
- Implement graceful error handling
- Log all validation decisions for monitoring

## Recent Changes

- Agent Validator (002-agent-validator): Added question validation using OpenAI Agents SDK with Gemini backend, validates questions for relevance to Physical AI & Humanoid Robotics textbook content before RAG processing
- RAG Chatbot (001-rag-chatbot): Implemented RAG system with Qdrant vector database and FastAPI backend
- AI Textbook Generation (001-ai-textbook-gen): Created Docusaurus-based textbook with 4 modules on Physical AI & Humanoid Robotics

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->