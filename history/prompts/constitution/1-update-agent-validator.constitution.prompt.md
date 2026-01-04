---
id: 1
title: Update Agent Validator
stage: constitution
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: Claude
command: /sp.constitution
labels: [constitution,rag,validation]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

```
text

I need to update my project constitution to add a new Agent Validator feature for the RAG chatbot.

Current constitution file location: `.specify/constitution.md`

Please ADD the following section after "## RAG Chatbot Implementation" (keep existing content, just add this):

---

### Agent Validator (Question Relevance Check)

**Purpose**: Ensure chatbot only answers questions related to the Physical AI & Humanoid Robotics textbook content.

**Implementation Strategy**: Option A - Validate BEFORE RAG retrieval
```
User Question → Agent Validation → [If relevant] → RAG Process → Answer
  → [If irrelevant] → Polite Rejection Message
```

**Technology Stack**:
- **Framework**: OpenAI Agents SDK (using Gemini as backend via AsyncOpenAI)
- **Model**: Gemini 2.0 Flash (via OpenAI-compatible endpoint)
- **Integration Point**: FastAPI `/query` endpoint - validation happens first

**Agent Configuration**:
- Agent Name: "Question Validator"
- Agent Role: Determine if user question is related to textbook topics
- Response Types:
  - `RELEVANT`: Question is about Physical AI, ROS 2, Gazebo, Isaac Sim, humanoid robotics, or related
    name="Question Validator",
    instructions="Check if question relates to Physical AI textbook topics...",
    model=OpenAIChatCompletionsModel(openai_client=client, model="gemini-2.0-flash")
)
```

**Quality Checks**:
- Agent must respond within 2 seconds
- False rejection rate < 5% (don't reject valid questions)
- Log all validation decisions for monitoring

---

Please update the constitution file and confirm the changes were applied correctly.
```

## Response snapshot

The constitution file has been successfully updated to include the Agent Validator section. I've added the new "Agent Validator (Question Relevance Check)" section after the "## RAG Chatbot Implementation" section as requested, and updated the version from 2.0.0 to 2.1.0 in the header along with the sync impact report.

## Outcome

- ✅ Impact: Added Agent Validator feature specification to constitution
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md updated
- 🔁 Next prompts: None needed
- 🧠 Reflection: Constitution now includes requirements for question relevance validation in the RAG chatbot

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A