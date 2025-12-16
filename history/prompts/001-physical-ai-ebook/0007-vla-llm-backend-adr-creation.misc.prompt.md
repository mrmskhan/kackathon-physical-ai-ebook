---
id: 0007
title: VLA LLM Backend ADR Creation
stage: misc
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-ebook
branch: 001-physical-ai-ebook
user: developer
command: /sp.adr Flexible LLM Backend for VLA Pipeline
labels: ["adr", "architecture", "vla", "llm", "whisper", "openai", "llama", "privacy", "cost"]
links:
  spec: specs/001-physical-ai-ebook/spec.md
  ticket: null
  adr: history/adr/0004-flexible-llm-backend-for-vla-pipeline.md
  pr: null
files:
  - history/adr/0004-flexible-llm-backend-for-vla-pipeline.md
tests:
  - Significance checklist validated (Impact: YES, Alternatives: YES, Scope: YES)
---

## Prompt

```text
/sp.adr Flexible LLM Backend for VLA Pipeline
```

## Response snapshot

Created ADR-0004 documenting the decision to use **Whisper (local) + flexible LLM backend** (OpenAI API or local Llama) → **ROS 2 Action Server**:

**Decision Rationale**:
- Voice-to-Text (Fixed): Whisper running locally (CPU or GPU) for all learners
- LLM Planning (Flexible): Dual-path documentation
  - Path A (Simple): OpenAI API (GPT-4) for plug-and-play experience
  - Path B (Privacy): Local Llama 3 (8B/70B) for offline, privacy-preserving operation
- Robot Execution (Fixed): ROS 2 Action Server pattern for long-running tasks

**Significance Validation** (all 3 criteria met):
1. **Impact**: Affects API key management, privacy model, Module 4 hardware requirements, capstone project
2. **Alternatives**: Evaluated OpenAI Whisper API + GPT-4 only (cloud-only), cloud LLM only (no privacy), direct LLM → topics (no Action Server)
3. **Scope**: Cross-cutting concern affecting Module 4 architecture, learner cost/privacy decisions, vendor lock-in prevention

**Positive Consequences**:
- Learner flexibility (cloud API vs. local inference based on priorities)
- Privacy options (local Llama for commercial/military on-premises AI)
- Cost transparency (OpenAI $0.01-0.03/call vs. local $0 inference)
- Progressive complexity (start OpenAI, migrate to Llama for optimization)
- Vendor independence (proprietary and open-source LLMs documented)

**Negative Consequences**:
- Documentation duplication (parallel setup guides for OpenAI vs. Llama)
- Hardware fragmentation (OpenAI any hardware, Llama 3 70B requires 40GB+ VRAM)
- Accuracy variance (GPT-4 very high vs. Llama 3 8B high)
- Support complexity (API errors vs. local inference issues)
- Latency unpredictability (OpenAI 500-2000ms network-dependent, Llama varies by model/hardware)

**Alternatives Rejected**:
1. **OpenAI Whisper API + GPT-4 Only**: Requires constant internet, accumulates API costs, excludes privacy use cases, Whisper API adds latency
2. **Cloud LLM Only**: Excludes restricted regions, forces API costs on students, prevents privacy-preserving patterns, limits offline robots
3. **Direct LLM → ROS 2 Topics**: Topics for streaming data not tasks, actions provide feedback/preemption/status, violates ROS 2 patterns

**Comparison Table Included**: Latency, accuracy, hardware load, cost, privacy across 5 VLA components

## Outcome

- ✅ Impact: Fourth ADR created, documents VLA architecture affecting Module 4 LLM backend flexibility and privacy model
- 🧪 Tests: Significance checklist passed (3/3 criteria validated - Impact/Alternatives/Scope)
- 📁 Files: Created ADR-0004 at history/adr/0004-flexible-llm-backend-for-vla-pipeline.md
- 🔁 Next prompts: Optional ADR for MDX Format decision, or `/sp.implement` to begin task execution
- 🧠 Reflection: ADR documents dual-path strategy balancing simplicity (OpenAI API 5-minute setup) with privacy/cost optimization (local Llama). Whisper fixed as local to avoid voice data cloud upload. ROS 2 Action Server pattern enforced for correct task execution architecture. Decision prioritizes learner choice (API vs. local) over single prescriptive approach.

## Evaluation notes (flywheel)

- Failure modes observed: None (ADR creation successful)
- Graders run and results (PASS/FAIL): Significance checklist PASS - all 3 criteria (Impact/Alternatives/Scope) met, decision cluster includes voice recognition (Whisper), LLM planning (dual-path), action decomposition, robot control (ROS 2 Actions)
- Prompt variant (if applicable): N/A (baseline)
- Next experiment (smallest change to try): During Module 4 implementation, validate that dual-path documentation clearly separates OpenAI and Llama setup to prevent learners from attempting hybrid configurations (e.g., OpenAI Whisper + local Llama, which adds unnecessary complexity)
