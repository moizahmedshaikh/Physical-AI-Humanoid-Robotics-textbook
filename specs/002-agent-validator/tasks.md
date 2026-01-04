# Implementation Tasks: Agent Validator for RAG Chatbot

**Feature**: Agent Validator for RAG Chatbot
**Branch**: `002-agent-validator`
**Created**: 2025-12-15
**Input**: Feature specification and plan from `/specs/002-agent-validator/`

## Implementation Strategy

Implement the Agent Validator feature in priority order, starting with the core validation functionality (User Story 1) which is the MVP. Each user story should be independently testable and deliver value on its own. The implementation follows the Phase structure: Setup → Foundational → User Stories → Polish.

## Dependencies

User stories are prioritized as follows:
1. **US1 (P1)**: Validate Question Relevance - Core functionality, highest priority
2. **US2 (P1)**: Receive Appropriate Response for Invalid Questions - Critical for UX
3. **US3 (P2)**: Fast Validation Processing - Performance requirement
4. **US4 (P2)**: Handle Edge Cases Gracefully - Robustness requirement

## Parallel Execution Examples

- **US1 & US2**: Can be developed in parallel since US2 is about the response format which can be implemented alongside validation logic
- **Testing tasks**: Can run in parallel with implementation tasks [P] marker
- **Logging & error handling**: Can be added in parallel with core functionality

---

## Phase 1: Setup

Setup tasks for project initialization and environment configuration.

- [x] T001 Create agent_validator.py file in api/src/services/ directory
- [x] T002 Verify OpenAI Agents SDK and AsyncOpenAI are available in api/.venv/
- [x] T003 Ensure GEMINI_API_KEY environment variable is accessible to the application

---

## Phase 2: Foundational

Foundational tasks that block all user stories - core validation infrastructure.

- [x] T004 [P] Create async function validate_question() in api/src/services/agent_validator.py with proper type hints
- [x] T005 [P] Implement OpenAI AsyncOpenAI client initialization pointing to Gemini API endpoint in agent_validator.py
- [x] T006 [P] Define ValidationConfig class with agent instructions for Physical AI, ROS 2, Gazebo, Isaac Sim, humanoid robotics, sensors, URDF, SLAM topics
- [x] T007 [P] Create Question and ValidationResult data models with type hints in agent_validator.py
- [x] T008 [P] Implement logging infrastructure for validation decisions in agent_validator.py
- [x] T009 [P] Create timeout mechanism to ensure validation completes within 2 seconds

---

## Phase 3: [US1] Validate Question Relevance (Priority: P1)

Implement the core functionality to validate if questions are relevant to Physical AI & Humanoid Robotics textbook content.

**Goal**: User questions are validated for relevance before RAG processing, with 'RELEVANT' or 'IRRELEVANT' responses.

**Independent Test**: Send various relevant and irrelevant questions to the system and verify that only relevant questions proceed to RAG processing while irrelevant ones receive the polite rejection message.

- [x] T010 [US1] Implement agent validation logic to return 'RELEVANT' or 'IRRELEVANT' based on question content
- [x] T011 [US1] Configure agent with instructions to determine if question relates to Physical AI, ROS 2, Gazebo, Isaac Sim, humanoid robotics, sensors, URDF, SLAM
- [x] T012 [US1] Handle special cases: treat greetings/pleasantries as RELEVANT
- [x] T013 [US1] Implement confidence scoring for validation decisions
- [x] T014 [US1] Add processing time measurement to validation results
- [x] T015 [US1] [P] Create unit tests for validate_question function with various relevant questions
- [x] T016 [US1] [P] Create unit tests for validate_question function with various irrelevant questions
- [x] T017 [US1] [P] Create unit tests for special case handling (greetings/pleasantries)

---

## Phase 4: [US2] Receive Appropriate Response for Invalid Questions (Priority: P1)

Implement the standardized rejection message for questions determined to be irrelevant.

**Goal**: Users asking off-topic questions receive a polite message explaining the scope of the system.

**Independent Test**: Submit various off-topic questions and verify that the system returns the specific rejection message template.

- [x] T018 [US2] Create constant for the standardized rejection message template
- [x] T019 [US2] Implement function to return rejection message when validation status is 'IRRELEVANT'
- [x] T020 [US2] Update API response structure to include rejection message for irrelevant questions
- [x] T021 [US2] [P] Create unit tests for rejection message functionality
- [x] T022 [US2] [P] Test various off-topic questions to ensure proper rejection message is returned

---

## Phase 5: [US3] Fast Validation Processing (Priority: P2)

Ensure validation completes within the required time limit.

**Goal**: Validation completes quickly to maintain good user experience with no noticeable delay.

**Independent Test**: Measure time taken for validation decisions and ensure they complete within the specified 2-second limit.

- [x] T023 [US3] Implement 2-second timeout for agent validation calls
- [x] T024 [US3] Add performance monitoring to validation function
- [x] T025 [US3] Create fallback logic for timeout scenarios (default to RELEVANT to avoid blocking valid questions)
- [x] T026 [US3] [P] Create performance tests to verify validation completes within 2 seconds
- [x] T027 [US3] [P] Add validation time logging for monitoring purposes

---

## Phase 6: [US4] Handle Edge Cases Gracefully (Priority: P2)

Implement robust handling of edge cases like typos and unclear questions.

**Goal**: System handles unclear or malformed questions gracefully to provide appropriate responses.

**Independent Test**: Submit various edge cases like typos, unclear questions, or malformed input and verify the system handles them appropriately.

- [x] T028 [US4] Implement error handling for API unavailability scenarios
- [x] T029 [US4] Handle extremely long question inputs gracefully
- [x] T030 [US4] Handle malformed or empty question inputs
- [x] T031 [US4] Handle questions with typos gracefully
- [x] T032 [US4] Handle questions containing both relevant and irrelevant topics
- [x] T033 [US4] [P] Create unit tests for edge case handling scenarios
- [x] T034 [US4] [P] Test error handling when validation API is unavailable

---

## Phase 7: Integration & API

Integrate the agent validator into the existing FastAPI application.

- [x] T035 Modify api/src/main.py to integrate validation before RAG processing
- [x] T036 Update the /query endpoint to call validate_question before perform_rag
- [x] T037 Implement routing logic: if RELEVANT → proceed to RAG, if IRRELEVANT → return rejection message
- [x] T038 Preserve existing perform_rag() function unchanged as required
- [x] T039 Ensure api/src/services/rag_service.py remains unchanged
- [x] T040 Add validation result metadata to successful RAG responses
- [x] T041 [P] Create integration tests for the complete validation and RAG flow
- [x] T042 [P] Create integration tests for the validation and rejection flow

---

## Phase 8: Polish & Cross-Cutting Concerns

Final implementation details and quality improvements.

- [x] T043 Add comprehensive logging for all validation decisions with timestamps
- [x] T044 Implement metrics collection for false rejection rate monitoring
- [x] T045 Add proper error handling with graceful fallbacks for all edge cases
- [x] T046 Verify type hints are present for all functions as required
- [x] T047 Ensure async/await pattern is used throughout for FastAPI compatibility
- [x] T048 Update documentation comments in agent_validator.py
- [x] T049 Run full test suite to verify all functionality works correctly
- [x] T050 Verify that existing RAG functionality is preserved for relevant questions
- [x] T051 Perform end-to-end testing with various question types
- [x] T052 Update README or quickstart documentation with validation feature details
