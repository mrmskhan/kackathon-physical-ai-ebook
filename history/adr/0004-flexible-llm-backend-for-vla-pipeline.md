# ADR-0004: Flexible LLM Backend for VLA Pipeline

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-13
- **Feature:** 001-physical-ai-ebook
- **Context:** Module 4 (VLA) requires LLM for voice command → robot action planning, but learners have varying privacy requirements, API budgets, and hardware capabilities

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - affects API key management, privacy model, Module 4 hardware requirements
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - OpenAI-only, cloud-only, direct topics evaluated
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects Module 4 architecture, capstone project, learner cost/privacy decisions
-->

## Decision

Use **Whisper (local) + flexible LLM backend** (OpenAI API or local Llama) → **ROS 2 Action Server** for VLA pipeline:

- **Voice-to-Text (Fixed)**: Whisper running locally (CPU or GPU) for all learners
- **LLM Planning (Flexible)**: Dual-path documentation
  - **Path A (Simple)**: OpenAI API (GPT-4) for plug-and-play experience with API key
  - **Path B (Privacy)**: Local Llama 3 (8B or 70B) for offline, privacy-preserving operation
- **Robot Execution (Fixed)**: ROS 2 Action Server pattern for long-running tasks (navigation, manipulation)

**VLA Architecture Components**:
- Voice Recognition: Whisper (local, supports CPU and GPU modes)
- Task Planning: OpenAI GPT-4 API OR local Llama 3 (8B/70B via Ollama)
- Action Decomposition: Python service mapping LLM output → ROS 2 action goals
- Robot Control: ROS 2 Action Servers (Nav2 for navigation, MoveIt for manipulation)
- Module 4 Structure: Shared VLA concepts → Path A (OpenAI) → Path B (Local Llama) → Capstone integration

## Consequences

### Positive

- **Learner flexibility**: Choose between cloud API (simpler setup, better accuracy) vs. local inference (privacy, offline, no recurring cost)
- **Privacy options**: Local Llama path enables commercial/military use cases requiring on-premises AI (no data sent to OpenAI)
- **Cost transparency**: OpenAI path clearly documents API costs ($0.01-0.03/call), local path has $0 inference cost after setup
- **Progressive complexity**: Start with OpenAI API (5-minute setup), optionally migrate to local Llama for privacy/cost optimization
- **Vendor independence**: Documentation covers both proprietary (OpenAI) and open-source (Llama 3) LLMs, preventing lock-in

### Negative

- **Documentation duplication**: Must maintain parallel setup guides, code examples, troubleshooting for OpenAI vs. Llama paths
- **Hardware fragmentation**: OpenAI path works on any hardware, Llama 3 70B requires 40GB+ VRAM (limits accessibility)
- **Accuracy variance**: GPT-4 (very high accuracy) vs. Llama 3 8B (high accuracy) may produce different action plans for same voice input
- **Support complexity**: Troubleshooting must account for OpenAI API errors vs. local Llama inference issues (VRAM, quantization, model loading)
- **Latency unpredictability**: OpenAI API latency varies (500-2000ms network-dependent), local Llama varies by model size and hardware

## Alternatives Considered

### Alternative A: OpenAI Whisper API + GPT-4 API Only (Cloud-Only Stack)
- **Stack**: OpenAI Whisper API for voice-to-text, GPT-4 API for planning, cloud-dependent architecture
- **Why rejected**:
  - Requires constant internet connectivity (excludes offline robot deployments)
  - Accumulates API costs for both Whisper and GPT-4 ($0.01-0.05/call combined)
  - Excludes privacy-sensitive use cases (military, healthcare, proprietary research)
  - Whisper API adds latency vs. local Whisper (network round-trip for audio upload)
- **Tradeoff**: Simplest setup (no local model management) vs. ongoing costs and privacy concerns

### Alternative B: Cloud LLM Only, No Local Option (No Privacy Path)
- **Stack**: Document OpenAI GPT-4 exclusively, omit local Llama 3 alternative
- **Why rejected**:
  - Excludes learners in regions with restricted OpenAI API access (China, Russia, some EU contexts)
  - Forces API costs on students and hobbyists ($10-50/month for experimentation)
  - Prevents teaching privacy-preserving AI deployment patterns (on-premises inference)
  - Limits to cloud-connected robots (cannot demonstrate offline autonomous operation)
- **Tradeoff**: Reduced documentation scope vs. accessibility and privacy exclusions

### Alternative C: Direct LLM → ROS 2 Topics (No Action Server)
- **Stack**: LLM outputs published directly to ROS 2 topics instead of Action Server pattern
- **Why rejected**:
  - ROS 2 topics are for streaming data (sensor readings, odometry), not task execution
  - Actions provide feedback (progress updates), preemption (cancellation), and goal status tracking
  - Navigation and manipulation require long-running tasks (5-30 seconds), ill-suited for topic pub/sub
  - Violates ROS 2 architectural patterns (actions are standard for goals, topics for data streams)
- **Tradeoff**: Simpler ROS 2 API (topics only) vs. incorrect pattern usage and loss of task management features

### Comparison Table

| Component | Latency | Accuracy | Hardware Load | Cost | Privacy |
|-----------|---------|----------|---------------|------|---------|
| Whisper (local, GPU) | ~100ms | High (95%+) | Medium (2GB VRAM) | $0 | Full |
| Whisper (local, CPU) | ~500ms | High (95%+) | Low | $0 | Full |
| OpenAI API (GPT-4) | 500-2000ms | Very High | None (cloud) | $0.01-0.03/call | Shared with OpenAI |
| Llama 3 (local, 8B) | 200-500ms | High | High (8GB+ VRAM) | $0 | Full |
| Llama 3 (local, 70B) | 1000-3000ms | Very High | Very High (40GB+ VRAM) | $0 | Full |

## References

- Feature Spec: `specs/001-physical-ai-ebook/spec.md` (User Story 4 - VLA Integration)
- Implementation Plan: `specs/001-physical-ai-ebook/plan.md`
- Research Document: `specs/001-physical-ai-ebook/research.md` (Decision 4: VLA Integration Architecture)
- Related ADRs:
  - ADR-0002 (Local RTX Primary with Cloud GPU) - Local Llama 3 70B requires RTX GPU (40GB+ VRAM), assumes local RTX workstation
- Evaluator Evidence: `history/prompts/001-physical-ai-ebook/0002-physical-ai-e-book-plan.plan.prompt.md` (Constitution check PASSED)
