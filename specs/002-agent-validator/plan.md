# Implementation Plan: Agent Validator for RAG Chatbot

**Branch**: `002-agent-validator` | **Date**: 2025-12-15 | **Spec**: [specs/002-agent-validator/spec.md](specs/002-agent-validator/spec.md)
**Input**: Feature specification from `/specs/002-agent-validator/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of an Agent Validator that uses OpenAI Agents SDK with Gemini backend to validate user questions for relevance to Physical AI & Humanoid Robotics textbook content before RAG processing. The validator will check if questions are related to topics like ROS 2, Gazebo, Isaac Sim, humanoid robotics, sensors, URDF, and SLAM, returning 'RELEVANT' or 'IRRELEVANT' responses within 2 seconds.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, OpenAI Agents SDK, AsyncOpenAI, Gemini API, Qdrant
**Storage**: N/A (uses existing Qdrant vector database)
**Testing**: pytest
**Target Platform**: Linux server
**Project Type**: Web API service
**Performance Goals**: Validation must complete within 2 seconds
**Constraints**: False rejection rate < 5%, must preserve existing RAG functionality
**Scale/Scope**: Handles all user queries to the RAG chatbot

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

1. **RAG Chatbot Implementation**: The feature aligns with the constitution's RAG Chatbot requirements, specifically the Agent Validator section (lines 131-160) which mandates validation of questions related to Physical AI & Humanoid Robotics textbook content.

2. **Technology Stack Compliance**: The implementation uses OpenAI Agents SDK with Gemini API as specified in the constitution (line 120, 142-143), which is compliant with the constraint of not using paid APIs (line 181).

3. **Integration Point**: The validation happens at the FastAPI `/query` endpoint before RAG retrieval as specified in the constitution (line 144).

4. **Quality Requirements**: The implementation meets the 2-second response time requirement (line 157) and false rejection rate < 5% (line 158) as specified in the constitution.

5. **Rejection Message**: The system will return the exact rejection message specified in the constitution (line 153-154).

## Project Structure

### Documentation (this feature)

```text
specs/002-agent-validator/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
api/
├── src/
│   ├── main.py          # Modified to integrate validation
│   └── services/
│       ├── rag_service.py  # Preserved (no changes)
│       └── agent_validator.py  # New file for validation logic
└── tests/
    └── unit/
        └── test_agent_validator.py  # New tests for validation
```

**Structure Decision**: The implementation follows a Web API structure with modifications to the existing FastAPI backend. The agent validator service is added as a new module in the services directory, with integration into the main.py query endpoint. The existing rag_service.py is preserved as required by the feature specification.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |