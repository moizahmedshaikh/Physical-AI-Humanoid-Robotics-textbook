"""
Agent Validator Service for RAG Chatbot

This module implements a validator using Gemini directly to check if user questions are relevant
to Physical AI & Humanoid Robotics textbook content before performing RAG retrieval.
"""

import logging
import os
import time
from datetime import datetime
from typing import Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum
import google.generativeai as genai
from pydantic import BaseModel, Field


def _is_greeting_or_pleasantry(text: str) -> bool:
    text_lower = text.strip().lower()
    greetings = ['hello', 'hi', 'hey', 'good morning', 'good afternoon', 'good evening', 'greetings', 'howdy', 'salutations']
    pleasantries = ['please', 'thank you', 'thanks', 'please and thank you', 'appreciate it', 'much appreciated', 'you\'re welcome', 'welcome']
    for greeting in greetings:
        if text_lower == greeting or text_lower.startswith(greeting + ' ') or text_lower.endswith(' ' + greeting) or ' ' + greeting + ' ' in text_lower:
            return True
    for pleasantry in pleasantries:
        if text_lower == pleasantry or text_lower.startswith(pleasantry + ' ') or text_lower.endswith(' ' + pleasantry) or ' ' + pleasantry + ' ' in text_lower:
            return True
    return False

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Metrics collection for monitoring
validation_metrics = {
    "total_validations": 0,
    "relevant_count": 0,
    "irrelevant_count": 0,
    "timeout_count": 0,
    "error_count": 0
}


class ValidationStatus(str, Enum):
    """Enum for validation status"""
    RELEVANT = "RELEVANT"
    IRRELEVANT = "IRRELEVANT"


@dataclass
class Question:
    """Data class representing a user question"""
    text: str
    timestamp: datetime = None
    user_id: Optional[str] = None

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now()


class ValidationResult(BaseModel):
    """Model for validation result"""
    status: ValidationStatus
    confidence: Optional[float] = Field(None, ge=0.0, le=1.0)
    reason: Optional[str] = None
    processing_time_ms: int
    timestamp: datetime = Field(default_factory=datetime.now)


SYSTEM_PROMPT = """You are a STRICT question relevance checker for the Physical AI & Humanoid Robotics textbook.

Your ONLY job is to determine if a question is RELEVANT or IRRELEVANT to these SPECIFIC topics:

✅ RELEVANT TOPICS (Physical AI & Humanoid Robotics Textbook):
- Physical AI and embodied intelligence
- Humanoid robotics systems and design
- ROS 2 (Robot Operating System 2) - NOT JavaScript frameworks
- Gazebo simulation environment - NOT web development
- NVIDIA Isaac Sim and Isaac ROS
- Robot sensor systems (LiDAR, cameras, IMU)
- URDF (Unified Robot Description Format) - robot models
- SLAM (Simultaneous Localization and Mapping)
- Robot navigation and path planning
- Physics simulation for robots
- Robot kinematics and control
- Robotics software development (Python, C++ for robots)

❌ IRRELEVANT TOPICS (Not in textbook):
- Web development (React, Next.js, Angular, Vue)
- General programming (unless robotics-specific)
- Cooking, recipes, food
- Sports, entertainment, movies
- History, geography (unless robotics history)
- General software frameworks (unless ROS 2)
- Mobile app development
- Database management (unless robotics-specific)
- Any non-robotics topics

CRITICAL RULES:
1. "Next.js" = IRRELEVANT (web framework, not robotics)
2. "React" = IRRELEVANT (web library, not ROS)
3. "What is [web technology]" = IRRELEVANT
4. "Python" alone = IRRELEVANT (unless robotics context)
5. Questions about robots, ROS 2, Gazebo, Isaac Sim = RELEVANT
6. Greetings ("Hello", "Hi", "Thanks") = RELEVANT
7. Short phrases like "ROS 2" or "SLAM" = RELEVANT if matching topics
8. ALWAYS respond with EXACTLY ONE WORD only: "RELEVANT" or "IRRELEVANT". No other text, no explanation!

Examples:
- "What is Next.js?" → IRRELEVANT
- "What is ROS 2?" → RELEVANT
- "How to use Gazebo?" → RELEVANT
- "What is React?" → IRRELEVANT
- "Hello" → RELEVANT
- "What is Python?" → IRRELEVANT
- "How to use Python in ROS 2?" → RELEVANT
- "How to cook food?" → IRRELEVANT
- "Tell me about football?" → IRRELEVANT
- "What is SLAM?" → RELEVANT
- "ROS 2" → RELEVANT
- "Physical AI & Humanoid Robotics" → RELEVANT
- "What is cooking?" → IRRELEVANT
- "Explain URDF" → RELEVANT
"""


async def validate_question(question_text: str, timeout_seconds: float = 5.0) -> ValidationResult:
    start_time = time.time()
    validation_metrics["total_validations"] += 1

    # Handle empty questions - treat as RELEVANT if context-only (for selected text)
    if not question_text or not question_text.strip():
        processing_time_ms = int((time.time() - start_time) * 1000)
        validation_metrics["relevant_count"] += 1  # Changed to relevant for context-only cases
        return ValidationResult(
            status=ValidationStatus.RELEVANT,
            confidence=0.6,
            processing_time_ms=processing_time_ms,
            reason="Empty query, assuming context-relevant"
        )

    # Handle long questions
    if len(question_text) > 1000:
        question_text = question_text[:1000]

    # Handle greetings
    if _is_greeting_or_pleasantry(question_text):
        processing_time_ms = int((time.time() - start_time) * 1000)
        validation_metrics["relevant_count"] += 1
        return ValidationResult(
            status=ValidationStatus.RELEVANT,
            confidence=0.8,
            processing_time_ms=processing_time_ms,
            reason="Greeting or pleasantry"
        )

    try:
        # Load Gemini API key
        GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
        genai.configure(api_key=GEMINI_API_KEY)
        model = genai.GenerativeModel('gemini-2.5-flash')  # Stable model

        # Generate response from Gemini
        full_prompt = f"{SYSTEM_PROMPT}\n\nQuestion: {question_text}"
        response = model.generate_content(full_prompt)
        agent_response = response.text.strip().upper()

        logger.info(f"Gemini raw response for '{question_text}': {agent_response}")

        if "IRRELEVANT" in agent_response:
            status = ValidationStatus.IRRELEVANT
            confidence = 0.9
            validation_metrics["irrelevant_count"] += 1
        elif "RELEVANT" in agent_response:
            status = ValidationStatus.RELEVANT
            confidence = 0.9
            validation_metrics["relevant_count"] += 1
        else:
            # Default to RELEVANT if unclear (lenient for robotics)
            status = ValidationStatus.RELEVANT
            confidence = 0.5
            validation_metrics["relevant_count"] += 1

        processing_time_ms = int((time.time() - start_time) * 1000)
        logger.info(f"Validation: {status.value} for '{question_text[:50]}...' (confidence: {confidence}, time: {processing_time_ms}ms)")

        return ValidationResult(
            status=status,
            confidence=confidence,
            processing_time_ms=processing_time_ms
        )

    except Exception as e:
        processing_time_ms = int((time.time() - start_time) * 1000)
        validation_metrics["error_count"] += 1
        validation_metrics["relevant_count"] += 1  # Default to RELEVANT on error
        logger.error(f"Validation error for '{question_text[:50]}...': {e}, defaulting to RELEVANT")
        return ValidationResult(
            status=ValidationStatus.RELEVANT,
            confidence=0.3,
            processing_time_ms=processing_time_ms,
            reason=f"Error: {str(e)}"
        )


# Standardized rejection message template (typo fixed)
REJECTION_MESSAGE = (
    "I can only answer questions related to the Physical AI & Humanoid Robotics textbook. "
    "Please ask questions about topics covered in the book such as ROS 2, Gazebo, Isaac Sim, "
    "sensors, or humanoid robotics."
)


def get_validation_metrics() -> Dict[str, int]:
    return validation_metrics.copy()


def calculate_false_rejection_rate() -> float:
    total_processed = validation_metrics["relevant_count"] + validation_metrics["irrelevant_count"]
    if total_processed == 0:
        return 0.0
    false_rejection_rate = (validation_metrics["irrelevant_count"] / total_processed) * 100
    return round(false_rejection_rate, 2)


def get_rejection_message() -> str:
    return REJECTION_MESSAGE


def format_rejection_response(validation_result: ValidationResult) -> dict:
    return {
        "message": REJECTION_MESSAGE,
        "validation": {
            "status": validation_result.status.value,
            "confidence": validation_result.confidence,
            "processing_time_ms": validation_result.processing_time_ms
        }
    }


def format_success_response(answer: str, source_chunks: list, validation_result: ValidationResult) -> dict:
    return {
        "answer": answer,
        "source_chunks": source_chunks,
        "validation": {
            "status": validation_result.status.value,
            "confidence": validation_result.confidence,
            "processing_time_ms": validation_result.processing_time_ms
        }
    }


if __name__ == "__main__":
    # Example usage
    import asyncio
    async def main():
        question = "How does ROS 2 handle message passing between nodes?"
        result = await validate_question(question)
        print(f"Question: {question}")
        print(f"Status: {result.status}")
        print(f"Confidence: {result.confidence}")
        print(f"Processing Time: {result.processing_time_ms}ms")

    asyncio.run(main())