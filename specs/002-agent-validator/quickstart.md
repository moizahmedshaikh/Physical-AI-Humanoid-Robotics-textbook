# Quickstart Guide: Agent Validator for RAG Chatbot

**Feature**: Agent Validator for RAG Chatbot
**Date**: 2025-12-15
**Branch**: 002-agent-validator

## Overview

This guide provides quick setup and usage instructions for the Agent Validator that validates user questions for relevance to Physical AI & Humanoid Robotics textbook content before RAG processing.

## Prerequisites

- Python 3.11+
- FastAPI
- OpenAI Python SDK
- Gemini API access
- Existing RAG infrastructure (Qdrant, FastAPI backend)

## Environment Setup

1. **Required Environment Variables**:
   ```bash
   export GEMINI_API_KEY="your-gemini-api-key"
   export QDRANT_URL="your-qdrant-url"
   export QDRANT_API_KEY="your-qdrant-api-key"
   ```

2. **Install Dependencies** (if not already installed):
   ```bash
   pip install openai fastapi
   ```

## Integration with Existing System

The Agent Validator integrates with the existing RAG system through the following modifications:

1. **New File**: `/api/src/services/agent_validator.py`
   - Contains the validation logic using OpenAI Agents SDK with Gemini backend
   - Implements the 2-second timeout requirement
   - Provides logging for all validation decisions

2. **Modified File**: `/api/src/main.py`
   - Adds validation step before RAG processing in the `/query` endpoint
   - Routes questions based on validation result (RELEVANT/IRRELEVANT)

## Usage Examples

### Validating a Question

```python
from api.src.services.agent_validator import validate_question

# Example: Valid question about ROS 2
result = await validate_question("How does ROS 2 handle message passing between nodes?")
# Returns: {"status": "RELEVANT", "confidence": 0.95, "processing_time_ms": 850}

# Example: Invalid question about cooking
result = await validate_question("What is the best recipe for chocolate cake?")
# Returns: {"status": "IRRELEVANT", "confidence": 0.98, "processing_time_ms": 720}
```

### API Integration

When a user submits a question to the `/query` endpoint:

1. Question is first sent to the Agent Validator
2. If validation returns `RELEVANT`, the question proceeds to RAG processing
3. If validation returns `IRRELEVANT`, the system returns the rejection message:
   > "I can only answer questions related to the Physical AI & Humanoid Robotics textbook. Please ask questions about topics covered in the book such as ROS 2, Gazebo, Isaac Sim, sensors, or humanoid robotics."

### Metrics and Monitoring

The system collects validation metrics that can be accessed:

```python
from api.src.services.agent_validator import get_validation_metrics, calculate_false_rejection_rate

metrics = get_validation_metrics()
# Returns: {"total_validations": 100, "relevant_count": 85, "irrelevant_count": 15, "timeout_count": 2, "error_count": 1}

false_rejection_rate = calculate_false_rejection_rate()
# Returns: 15.0 (percentage of questions incorrectly marked as irrelevant)
```

## Performance Requirements

- **Response Time**: Validation must complete within 2 seconds
- **Accuracy**: False rejection rate must be less than 5%
- **Logging**: All validation decisions are logged for monitoring

## Testing

Run the validation tests to ensure proper functionality:

```bash
pytest tests/unit/test_agent_validator.py
```

## Troubleshooting

1. **Validation taking too long**: Check Gemini API connectivity and response times
2. **High false rejection rate**: Review agent instructions and adjust if needed
3. **API errors**: Verify GEMINI_API_KEY is properly set and has sufficient quota