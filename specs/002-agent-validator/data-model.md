# Data Model: Agent Validator for RAG Chatbot

**Feature**: Agent Validator for RAG Chatbot
**Date**: 2025-12-15
**Branch**: 002-agent-validator

## Entities

### Question
- **Description**: A user input string that requires validation for relevance to Physical AI & Humanoid Robotics topics
- **Attributes**:
  - `text` (string): The user's question text
  - `timestamp` (datetime): When the question was submitted
  - `user_id` (string, optional): Identifier for the user (if available)
- **Validation rules**: Must not be empty or consist only of whitespace

### Validation Result
- **Description**: The outcome of the validation process, either 'RELEVANT' or 'IRRELEVANT' with associated metadata
- **Attributes**:
  - `status` (string): Either 'RELEVANT' or 'IRRELEVANT'
  - `confidence` (float, optional): Confidence level of the validation decision (0.0 to 1.0)
  - `reason` (string, optional): Brief explanation for the validation decision
  - `processing_time_ms` (int): Time taken for validation in milliseconds
  - `timestamp` (datetime): When the validation was completed
- **Validation rules**: Status must be either 'RELEVANT' or 'IRRELEVANT'; confidence must be between 0.0 and 1.0 if provided

### Rejection Message
- **Description**: A standardized response provided to users when their question is determined to be out of scope
- **Attributes**:
  - `message` (string): The standardized rejection message text
  - `suggested_topics` (list of strings): Topics the user might ask about instead
- **Validation rules**: Message must match the template specified in the feature requirements

## State Transitions

### Question Validation Flow
1. **Question Received**: User submits a question to the system
2. **Validation In Progress**: System sends question to the agent validator
3. **Validation Complete**: Agent returns validation result
   - If RELEVANT: Proceed to RAG processing
   - If IRRELEVANT: Return rejection message to user

## Relationships

- A `Question` generates one `Validation Result`
- A `Validation Result` with status 'IRRELEVANT' triggers a `Rejection Message`
- All entities are associated with a specific `timestamp` for audit and analysis

## Data Validation Requirements

1. **Performance**: Validation must complete within 2 seconds (2000ms)
2. **Accuracy**: False rejection rate must be less than 5%
3. **Logging**: All validation decisions must be logged for monitoring
4. **Error Handling**: System must handle API errors gracefully with appropriate fallbacks