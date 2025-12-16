# ADR-0002: Local RTX Primary with Cloud GPU Fallback

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-13
- **Feature:** 001-physical-ai-ebook
- **Context:** Modules 3-4 (Isaac Sim, VLA) require GPU acceleration for practical learning, but requiring RTX GPU purchase creates accessibility barrier

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - affects learner environment setup, cost model, accessibility
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - cloud-only, local-only, CPU-only all evaluated
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects Modules 3-4-5, hardware requirements, setup documentation
-->

## Decision

Use **Local RTX workstation as primary recommendation** with **Cloud GPU (AWS g5/g6) as documented alternative**:

- **Primary Path (Local RTX)**: Recommend local RTX GPU workstation (RTX 3060+ with 12GB VRAM minimum) for Modules 3-4
- **Alternative Path (Cloud GPU)**: Provide AWS g5.xlarge (NVIDIA A10G, 24GB VRAM) setup instructions for learners without local RTX hardware
- **Progressive Requirements**: Modules 1-2 (ROS 2, Gazebo) run on CPU-only, Modules 3-4 (Isaac Sim, VLA) require GPU

**Hardware Strategy Components**:
- Local Workstation Specs: RTX 4070+ (12GB VRAM), 32GB RAM, Ubuntu 22.04 LTS
- Cloud Alternative: AWS g5.xlarge ($1.006/hour on-demand), pre-configured NVIDIA drivers
- Module Annotations: Each Isaac Sim/VLA lesson includes "Local RTX" vs. "Cloud GPU" configuration notes
- Cost Transparency: Module 5 includes TCO comparison (upfront vs. hourly costs)

## Consequences

### Positive

- **Optimal local experience**: <10ms latency for iterative simulation development, no network dependency, full environment control
- **Accessibility via cloud**: Learners without RTX hardware can still complete Modules 3-4 using pay-as-you-go cloud GPU
- **Progressive cost curve**: Modules 1-2 free (CPU-only), learners invest in GPU only after validating interest in Isaac/VLA workflows
- **Data privacy**: Local workstation keeps synthetic datasets and AI models on-premises (important for commercial use cases)
- **No vendor lock-in**: Cloud instructions work for AWS, Azure, GCP (g5/NVv4/T4 instances respectively)

### Negative

- **Documentation overhead**: Must maintain two parallel setup paths for Modules 3-4 (local vs. cloud configuration)
- **Cost barrier remains**: Local RTX workstation requires $1500-3000 upfront investment, cloud GPU costs $50-100/month for part-time learners
- **Cloud latency**: 50-150ms network latency impacts real-time simulation feedback compared to local <10ms
- **Cloud persistence complexity**: Requires EBS volumes for data persistence, increasing monthly costs and setup complexity
- **Support fragmentation**: Troubleshooting must account for local driver issues vs. cloud networking/provisioning issues

## Alternatives Considered

### Alternative A: Cloud-Only (No Local RTX Documentation)
- **Stack**: AWS g5/g6 instances exclusively, no local RTX workstation path
- **Why rejected**:
  - Ongoing costs prohibitive for long-term learners ($50-100/month adds up over months)
  - Network latency (50-150ms) impacts real-time simulation feedback and iterative development
  - Dependency on AWS uptime and internet connectivity creates reliability issues
  - Data transfer costs for large synthetic datasets (multi-GB Isaac Sim scenes)
- **Tradeoff**: Lower upfront cost barrier vs. worse developer experience and ongoing costs

### Alternative B: Local-Only (No Cloud Alternative)
- **Stack**: Require RTX GPU purchase, no cloud GPU documentation
- **Why rejected**:
  - Excludes learners without $1500-3000 upfront budget for RTX hardware
  - Limits global accessibility (GPU prices vary, import taxes in some regions)
  - Creates "pay to learn" barrier for students and hobbyists
  - Prevents corporate learners from using existing cloud infrastructure
- **Tradeoff**: Simplified documentation vs. reduced accessibility

### Alternative C: CPU-Only Approach
- **Stack**: Attempt to run Isaac Sim and VLA on CPU-only hardware
- **Why rejected**:
  - Isaac Sim officially requires RTX GPU (Omniverse platform requirement)
  - VLA inference (Whisper + Llama 3 70B) extremely slow on CPU (10-30 seconds vs. 500ms on GPU)
  - Photorealistic rendering and synthetic data generation impractical without GPU acceleration
  - Compromises learning experience with long wait times and limited dataset sizes
- **Tradeoff**: Universal accessibility vs. impractical performance and limited feature coverage

### Comparison Table

| Factor | Local RTX Workstation | Cloud GPU (AWS g5/g6) |
|--------|----------------------|----------------------|
| Cost | Upfront ($1500-3000 for RTX 4070+) | $1-2/hour (~$50-100/month part-time) |
| Latency | <10ms (no network) | 50-150ms (network dependent) |
| Reliability | Full control | Dependent on AWS uptime |
| Accessibility | Requires hardware purchase | Pay-as-you-go, no upfront cost |
| Data Persistence | Local storage (free) | EBS volumes ($0.08/GB-month) |
| Setup Complexity | Driver install only | AWS account, VPC, security groups, SSH |

## References

- Feature Spec: `specs/001-physical-ai-ebook/spec.md`
- Implementation Plan: `specs/001-physical-ai-ebook/plan.md`
- Research Document: `specs/001-physical-ai-ebook/research.md` (Decision 2: Hardware Environment Strategy)
- Related ADRs: ADR-0001 (Multi-Simulator Architecture) - Isaac Sim is one of three simulators requiring GPU
- Evaluator Evidence: `history/prompts/001-physical-ai-ebook/0002-physical-ai-e-book-plan.plan.prompt.md` (Constitution check PASSED - all 6 principles validated)
