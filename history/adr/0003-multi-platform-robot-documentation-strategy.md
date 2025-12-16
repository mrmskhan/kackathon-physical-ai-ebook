# ADR-0003: Multi-Platform Robot Documentation Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-13
- **Feature:** 001-physical-ai-ebook
- **Context:** E-book needs to serve learners with varied budgets ($0 simulation-only to $16k+ humanoid purchase) while maintaining transferable robotics knowledge

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - defines Module 5 hardware content, sim-to-real coverage, accessibility
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - G1-only, simulation-only, single platform all evaluated
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects hardware setup docs, Module 5 structure, learning path flexibility
-->

## Decision

Use **multi-platform documentation** covering Unitree Go2 quadruped, ROBOTIS OP3 humanoid, and generic simulation proxies:

- **Primary Learning Environment**: Simulation (Gazebo, Unity, Isaac Sim) with generic robot proxies (arms, quadrupeds, humanoids)
- **Optional Hardware Platforms**:
  - Unitree Go2 ($1600 quadruped) - affordable entry point for locomotion and ROS 2 fundamentals
  - ROBOTIS OP3 ($10k+ humanoid) - open-source reference for bipedal walking and humanoid research
  - Unitree G1 ($16k+ humanoid) - mentioned for completeness, not primary focus
- **Module 5 Structure**: Comparison table → platform-agnostic concepts → per-platform setup guides → sim-to-real validation

**Documentation Strategy Components**:
- Platform Comparison Matrix: Cost, humanoid fidelity, accessibility, ROS 2 support
- Simulation-First Approach: All modules completable without hardware purchase
- Hardware as Validation: Optional real-world testing of simulation-developed skills
- Transferable Knowledge: Emphasize ROS 2 APIs and patterns common across platforms

## Consequences

### Positive

- **Universal accessibility**: Learners can complete 100% of content with $0 hardware investment (simulation-only path)
- **Progressive investment**: Clear cost tiers ($0 sim → $1600 Go2 → $10k+ OP3) enable budget-conscious learners to start without commitment
- **Platform flexibility**: Knowledge transfers across quadrupeds, humanoids, manipulators (ROS 2 abstractions are platform-agnostic)
- **Sim-to-real coverage**: Explicitly addresses gap between simulation and hardware deployment (Module 5 focus)
- **Research relevance**: OP3 as open-source humanoid aligns with academic robotics research practices

### Negative

- **Documentation breadth**: Requires setup guides, troubleshooting, and examples for 3+ platforms vs. single-platform simplicity
- **No deep platform expertise**: Surface-level coverage of each platform vs. comprehensive Unitree-only or OP3-only guide
- **Hardware validation limited**: Cannot guarantee examples work on ALL platforms (testing requires physical access to Go2, OP3, G1)
- **Confusion risk**: Learners may struggle deciding "which platform is right for me?" without expert guidance
- **Maintenance burden**: Platform firmware updates, ROS 2 driver changes across multiple ecosystems

## Alternatives Considered

### Alternative A: Unitree G1 Humanoid Only (Premium Focus)
- **Stack**: Unitree G1 ($16k+) as exclusive hardware platform, all examples G1-specific
- **Why rejected**:
  - Cost prohibitive for students, hobbyists, and educators ($16k+ vs. $1600 Go2)
  - Limited global availability (supply chain, import restrictions in some regions)
  - Less accessible than quadruped for locomotion fundamentals (walking harder than four-legged gait)
  - Excludes learners interested in quadrupeds, manipulators, or non-humanoid form factors
- **Tradeoff**: Cutting-edge humanoid fidelity vs. severe accessibility and cost barriers

### Alternative B: Simulation-Only (No Real Hardware)
- **Stack**: Gazebo, Unity, Isaac Sim proxies exclusively, no physical robot documentation
- **Why rejected**:
  - Fails to address sim-to-real gap (sensor noise, actuator delays, contact physics mismatches)
  - Limits practical deployment learning (flashing firmware, wiring sensors, debugging hardware)
  - Reduces credibility for industry professionals requiring real-world validation skills
  - Misses opportunity to teach hardware integration workflows (ROS 2 drivers, URDF calibration)
- **Tradeoff**: Universal accessibility ($0 cost) vs. incomplete robotics education (sim-to-real is critical)

### Alternative C: Single Platform Focus (Unitree Go2 Only)
- **Stack**: Unitree Go2 ($1600) as exclusive hardware platform, deep dive into Go2 ecosystem
- **Why rejected**:
  - Reduces knowledge transferability (Go2-specific APIs vs. ROS 2 abstractions)
  - Excludes humanoid learners (bipedal walking algorithms, manipulation workflows)
  - Limits to quadruped locomotion (no arm manipulation, no biped balancing)
  - Platform vendor lock-in (Unitree SDK vs. platform-agnostic ROS 2 patterns)
- **Tradeoff**: Deep platform expertise vs. reduced transferability and humanoid exclusion

### Comparison Table

| Platform | Cost | Humanoid Fidelity | Accessibility | ROS 2 Support |
|----------|------|-------------------|---------------|---------------|
| Unitree Go2 | $1600 | Low (quadruped) | High | Excellent |
| Unitree G1 | $16000 | Very High | Low | Good |
| ROBOTIS OP3 | $10000+ | High | Medium | Excellent |
| Proxy Robots (sim) | $0 | Configurable | Very High | Excellent |

## References

- Feature Spec: `specs/001-physical-ai-ebook/spec.md` (User Story 5 - Hardware Deployment)
- Implementation Plan: `specs/001-physical-ai-ebook/plan.md`
- Research Document: `specs/001-physical-ai-ebook/research.md` (Decision 3: Robot Platform Coverage)
- Related ADRs:
  - ADR-0001 (Multi-Simulator Architecture) - Simulation proxies are primary learning environment
  - ADR-0002 (Local RTX Primary with Cloud GPU) - Hardware platforms assume local RTX workstation for Isaac Sim integration
- Evaluator Evidence: `history/prompts/001-physical-ai-ebook/0002-physical-ai-e-book-plan.plan.prompt.md` (Constitution check PASSED)
