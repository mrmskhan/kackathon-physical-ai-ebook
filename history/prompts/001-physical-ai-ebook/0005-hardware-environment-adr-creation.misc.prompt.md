---
id: 0005
title: Hardware Environment ADR Creation
stage: misc
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-ebook
branch: 001-physical-ai-ebook
user: developer
command: /sp.adr Local RTX Primary with Cloud GPU Fallback
labels: ["adr", "architecture", "hardware", "gpu", "cloud", "accessibility", "cost-model"]
links:
  spec: specs/001-physical-ai-ebook/spec.md
  ticket: null
  adr: history/adr/0002-local-rtx-primary-with-cloud-gpu-fallback.md
  pr: null
files:
  - history/adr/0002-local-rtx-primary-with-cloud-gpu-fallback.md
tests:
  - Significance checklist validated (Impact: YES, Alternatives: YES, Scope: YES)
---

## Prompt

```text
/sp.adr Local RTX Primary with Cloud GPU Fallback
```

## Response snapshot

Created ADR-0002 documenting the decision to use **Local RTX workstation as primary** with **Cloud GPU (AWS g5/g6) as documented alternative**:

**Decision Rationale**:
- Primary Path: Local RTX 3060+ (12GB VRAM) for optimal <10ms latency and full environment control
- Alternative Path: AWS g5.xlarge (NVIDIA A10G, 24GB VRAM) for learners without RTX hardware
- Progressive Requirements: Modules 1-2 CPU-only, Modules 3-4 (Isaac Sim, VLA) require GPU

**Significance Validation** (all 3 criteria met):
1. **Impact**: Affects learner environment setup, cost model (upfront vs. hourly), accessibility across Modules 3-4-5
2. **Alternatives**: Evaluated cloud-only, local-only, CPU-only with documented tradeoffs
3. **Scope**: Cross-cutting concern affecting hardware requirements documentation, setup guides, TCO comparisons

**Positive Consequences**:
- Optimal local experience (<10ms latency vs. 50-150ms cloud)
- Accessibility via pay-as-you-go cloud GPU ($1-2/hour)
- Progressive cost curve (free CPU-only Modules 1-2, then GPU investment decision)
- Data privacy (local workstation for commercial use cases)
- No vendor lock-in (instructions work for AWS/Azure/GCP)

**Negative Consequences**:
- Documentation overhead (two parallel setup paths for Modules 3-4)
- Cost barrier remains ($1500-3000 local upfront or $50-100/month cloud)
- Cloud latency impacts real-time simulation feedback
- Cloud persistence complexity (EBS volumes increase costs)
- Support fragmentation (local driver issues vs. cloud networking)

**Alternatives Rejected**:
1. **Cloud-Only**: Ongoing costs prohibitive, latency impacts iterative development, AWS dependency
2. **Local-Only**: Excludes learners without $1500-3000 upfront budget, global accessibility limits
3. **CPU-Only**: Isaac Sim requires RTX GPU, VLA inference impractical (10-30s vs. 500ms)

**Comparison Table Included**: Cost, latency, reliability, accessibility, data persistence, setup complexity

## Outcome

- ✅ Impact: Second ADR created, documents hardware environment strategy affecting Modules 3-4 accessibility and cost model
- 🧪 Tests: Significance checklist passed (3/3 criteria validated - Impact/Alternatives/Scope)
- 📁 Files: Created ADR-0002 at history/adr/0002-local-rtx-primary-with-cloud-gpu-fallback.md
- 🔁 Next prompts: Optional ADRs for remaining architecture decisions (Multi-Platform Robots, Flexible LLM Backend, MDX Format), or `/sp.implement` to begin task execution
- 🧠 Reflection: ADR documents hybrid strategy balancing optimal developer experience (local RTX) with accessibility (cloud GPU alternative). TCO transparency (upfront vs. hourly costs) helps learners make informed decisions. Progressive requirements (CPU → GPU) reduce friction for early modules while enabling advanced AI workflows in later modules.

## Evaluation notes (flywheel)

- Failure modes observed: None (ADR creation successful)
- Graders run and results (PASS/FAIL): Significance checklist PASS - all 3 criteria (Impact/Alternatives/Scope) met, decision cluster approach followed (hardware strategy includes local specs, cloud alternative, module annotations, cost transparency)
- Prompt variant (if applicable): N/A (baseline)
- Next experiment (smallest change to try): During Module 5 implementation, validate that TCO comparison includes hidden cloud costs (EBS volumes, data transfer) to prevent learner surprise when monthly bills arrive
