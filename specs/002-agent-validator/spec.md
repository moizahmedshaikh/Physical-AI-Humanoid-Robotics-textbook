# Feature Specification: Agent Validator for RAG Chatbot

**Feature Branch**: `002-agent-validator`
**Created**: 2025-12-15
**Status**: Draft
**Input**: User description: "Implement Agent Validator for RAG Chatbot using OpenAI Agents SDK with Gemini backend. The validator should check if user questions are related to Physical AI & Humanoid Robotics textbook before performing RAG retrieval.

Technical Requirements:
- Use OpenAI Agents SDK with AsyncOpenAI client pointing to Gemini API endpoint
- Model: gemini-2.0-flash via OpenAI-compatible endpoint
- Create new file: /api/src/services/agent_validator.py
- Modify existing file: /api/src/main.py to integrate validation
- Validation happens BEFORE RAG process (Option A strategy)
- Return polite rejection message for irrelevant questions
- Validation must complete within 2 seconds
- False rejection rate target: <5%

Integration Flow:
1. User sends question to /query endpoint
2. Agent Validator checks relevance
3. If RELEVANT → proceed to existing RAG system (perform_rag function)
4. If IRRELEVANT → return rejection message without RAG

Agent Configuration:
- Agent Name: Question Validator
- Instructions: Determine if question relates to Physical AI, ROS 2, Gazebo, Isaac Sim, humanoid robotics, sensors, URDF, SLAM
- Response: Return 'RELEVANT' or 'IRRELEVANT' (single word)
- Special cases: Greetings/pleasantries treated as RELEVANT

Rejection Message Template:
'I can only answer questions related to the Physical AI & Humanoid Robotics textbook. Please ask questions about topics covered in the book such as ROS 2, Gazebo, Isaac Sim, sensors, or humanoid robotics.'

Existing Code to Preserve:
- /api/src/services/rag_service.py - Keep unchanged
- /api/src/main.py perform_rag() function - Keep unchanged
- Chatbot.js frontend - No changes needed
- Qdrant database schema - No changes needed

Dependencies:
- openai-agents-sdk (already installed via UV in /api/.venv/)
- Use existing GEMINI_API_KEY from .env

Quality Requirements:
- Log all validation decisions
- Handle edge cases (typos, unclear questions)
- Graceful error handling
- Type hints for all functions
- Async/await pattern for FastAPI compatibility

Generate complete spec.md with implementation details, API contracts, error handling strategy, and testing approach."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Validate Question Relevance (Priority: P1)

As a user of the Physical AI & Humanoid Robotics textbook chatbot, I want my questions to be validated for relevance before processing, so that I only receive answers related to the textbook content.

**Why this priority**: This is the core functionality that prevents the system from processing irrelevant questions and ensures users get appropriate responses for out-of-scope queries.

**Independent Test**: Can be fully tested by sending various relevant and irrelevant questions to the system and verifying that only relevant questions proceed to RAG processing while irrelevant ones receive the polite rejection message.

**Acceptance Scenarios**:

1. **Given** a user submits a question about ROS 2, **When** the validation system processes it, **Then** the system determines it as RELEVANT and proceeds with RAG retrieval
2. **Given** a user submits a question about cooking recipes, **When** the validation system processes it, **Then** the system determines it as IRRELEVANT and returns the rejection message
3. **Given** a user submits a greeting like "Hello", **When** the validation system processes it, **Then** the system treats it as RELEVANT and proceeds with RAG processing

---

### User Story 2 - Receive Appropriate Response for Invalid Questions (Priority: P1)

As a user asking off-topic questions, I want to receive a polite message explaining the scope of the system, so that I understand what types of questions are appropriate.

**Why this priority**: This ensures a good user experience when questions are out of scope, preventing confusion and guiding users to appropriate queries.

**Independent Test**: Can be fully tested by submitting various off-topic questions and verifying that the system returns the specific rejection message template.

**Acceptance Scenarios**:

1. **Given** a user submits a question about sports, **When** the validation determines it's irrelevant, **Then** the system returns the standardized rejection message
2. **Given** a user submits a question about movies, **When** the validation determines it's irrelevant, **Then** the system returns the standardized rejection message

---

### User Story 3 - Fast Validation Processing (Priority: P2)

As a user, I want the validation to complete quickly so that there's no noticeable delay in my conversation with the chatbot.

**Why this priority**: Performance is critical for maintaining a good user experience and ensuring the system feels responsive.

**Independent Test**: Can be tested by measuring the time taken for validation decisions and ensuring they complete within the specified 2-second limit.

**Acceptance Scenarios**:

1. **Given** a user submits a question, **When** the validation system processes it, **Then** the validation completes within 2 seconds
2. **Given** a complex question about robotics, **When** the validation system processes it, **Then** the validation completes within 2 seconds

---

### User Story 4 - Handle Edge Cases Gracefully (Priority: P2)

As a user, I want the system to handle unclear or malformed questions gracefully, so that I still get appropriate responses.

**Why this priority**: This ensures robustness and prevents system failures when users make typos or ask unclear questions.

**Independent Test**: Can be tested by submitting various edge cases like typos, unclear questions, or malformed input and verifying the system handles them appropriately.

**Acceptance Scenarios**:

1. **Given** a user submits a question with typos, **When** the validation system processes it, **Then** the system makes an appropriate relevance decision
2. **Given** a user submits a very short or unclear question, **When** the validation system processes it, **Then** the system handles it gracefully with appropriate fallback

---

### Edge Cases

- What happens when the validation API is temporarily unavailable?
- How does the system handle extremely long or malformed questions?
- What if the validation process times out after 2 seconds?
- How does the system handle questions that contain both relevant and irrelevant topics?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST validate user questions against the Physical AI & Humanoid Robotics textbook scope before RAG processing
- **FR-002**: System MUST use OpenAI Agents SDK with AsyncOpenAI client pointing to Gemini API endpoint for validation
- **FR-003**: System MUST return 'RELEVANT' or 'IRRELEVANT' as the validation response
- **FR-004**: System MUST return the specific rejection message template for irrelevant questions
- **FR-005**: System MUST treat greetings and pleasantries as RELEVANT questions
- **FR-006**: System MUST complete validation within 2 seconds
- **FR-007**: System MUST maintain a false rejection rate of less than 5%
- **FR-008**: System MUST log all validation decisions for monitoring and analysis
- **FR-009**: System MUST preserve existing RAG functionality when questions are deemed relevant
- **FR-010**: System MUST handle API errors gracefully with appropriate fallbacks
- **FR-011**: System MUST use type hints for all functions to ensure code quality

### Key Entities

- **Question**: A user input string that requires validation for relevance to Physical AI & Humanoid Robotics topics
- **Validation Result**: The outcome of the validation process, either 'RELEVANT' or 'IRRELEVANT' with associated confidence level
- **Rejection Message**: A standardized response provided to users when their question is determined to be out of scope

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users receive validation responses within 2 seconds for 95% of questions
- **SC-002**: System achieves a false rejection rate of less than 5% based on test question sets
- **SC-003**: 90% of relevant questions are correctly identified as relevant and proceed to RAG processing
- **SC-004**: 90% of irrelevant questions are correctly identified as irrelevant and receive appropriate rejection messages
- **SC-005**: Users report satisfaction with the system's ability to handle out-of-scope questions appropriately
- **SC-006**: System maintains existing RAG functionality performance for relevant questions