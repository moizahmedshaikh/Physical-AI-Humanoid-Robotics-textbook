---
id: 2
title: Update Spec-Kit Plus configuration to use Bash scripts
stage: general
date: 2025-11-28
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-ai-textbook-gen
user: moizahmedshaikh
command: /sp.phr
labels: ["spec-kit-plus", "configuration", "bash-scripts", "powershell"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .claude/commands/sp.specify.md
  - .claude/commands/sp.clarify.md
  - .claude/commands/sp.tasks.md
  - .claude/commands/sp.analyze.md
  - .claude/commands/sp.plan.md
  - .claude/commands/sp.implement.md
tests:
  - none
---

## Prompt

Please update my Spec-Kit Plus configuration to use the existing bash scripts instead of PowerShell scripts.

The bash scripts already exist in `.specify/scripts/bash/` folder but the system is still using PowerShell. Please:

1. Update the configuration/settings to point to bash scripts in `.specify/scripts/bash/`
2. Change default shell from PowerShell to bash
3. Update all script references to use `.sh` files instead of `.ps1`
4. Ensure future commands use bash scripts from the bash folder

The bash scripts are already available, just need to update the configuration to use them.

## Response snapshot

I have updated the Spec-Kit Plus configuration files to use bash scripts instead of PowerShell scripts. This involved modifying several command definition files within the .claude/commands/ directory to point to the correct .sh script paths.

## Outcome

- ✅ Impact: Spec-Kit Plus configuration updated to use Bash scripts instead of PowerShell.
- 🧪 Tests: none
- 📁 Files: 6 files modified
- 🔁 Next prompts: Proceed with further Spec-Kit Plus commands.
- 🧠 Reflection: Ensured compatibility with Bash environment for Spec-Kit Plus tools by manually updating command definition files.

## Evaluation notes (flywheel)

- Failure modes observed: Script '.specify/scripts/bash/create-phr.sh' was not found, requiring fallback to agent-native PHR creation.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Verify all Spec-Kit Plus scripts exist in '.specify/scripts/bash/' and ensure they are executable, and consider creating a robust create-phr.sh script.
