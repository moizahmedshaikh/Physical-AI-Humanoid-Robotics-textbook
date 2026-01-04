#!/usr/bin/env python3
"""
Test script for the updated agent_validator module.
This script tests the updated validate_question function that uses OpenAI Agents SDK.
"""

import asyncio
import os
import sys

# Add the api/src directory to the path so we can import the agent_validator module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'api', 'src'))

from services.agent_validator import validate_question, ValidationResult, ValidationStatus


async def test_validate_question():
    """Test the updated validate_question function"""
    print("Testing the updated validate_question function with OpenAI Agents SDK...")

    # Test cases
    test_cases = [
        "What is ROS 2?",
        "How does SLAM work in robotics?",
        "Hello",
        "Thanks for your help!",
        "",  # Empty question
        "This is a very long question " + "word " * 100,  # Long question
        "What is the weather today?",  # Irrelevant question
        "Explain Isaac Sim in detail",  # Relevant question
    ]

    for i, question in enumerate(test_cases):
        print(f"\nTest {i+1}: '{question[:30]}{'...' if len(question) > 30 else ''}'")
        try:
            result = await validate_question(question, timeout_seconds=5.0)
            print(f"  Status: {result.status}")
            print(f"  Confidence: {result.confidence}")
            print(f"  Processing Time: {result.processing_time_ms}ms")
            print(f"  Reason: {result.reason if result.reason else 'N/A'}")
        except Exception as e:
            print(f"  Error: {e}")


if __name__ == "__main__":
    # Make sure GEMINI_API_KEY is set in environment
    if not os.getenv("GEMINI_API_KEY"):
        print("Warning: GEMINI_API_KEY environment variable not set.")
        print("Please set it before running the test: export GEMINI_API_KEY='your_api_key'")
        print("Using a mock for testing purposes...")
        os.environ["GEMINI_API_KEY"] = "mock-key-for-test"

    print("Running tests for agent_validator with OpenAI Agents SDK...")
    try:
        asyncio.run(test_validate_question())
        print("\nTest completed!")
    except Exception as e:
        print(f"\nTest failed with error: {e}")
        print("Note: This may be expected if GEMINI_API_KEY is not properly configured or network issues occur.")