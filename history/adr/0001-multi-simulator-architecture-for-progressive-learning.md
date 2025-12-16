# ADR-0001: Multi-Simulator Architecture for Progressive Learning

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-13
- **Feature:** 001-physical-ai-ebook
- **Context:** E-book needs simulation stack that serves beginners (CPU-only) through advanced AI practitioners (RTX GPU)

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - defines entire simulation learning path
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - Gazebo-only, Isaac-only, Unity-only all evaluated
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects Modules 2-3, hardware requirements, learner accessibility
-->

## Decision

Use **Gazebo Garden + Unity + Isaac Sim** in progressive sequence across modules:

- **Module 2 (Foundation)**: Start with Gazebo Garden for accessible, lightweight ROS 2 physics simulation (CPU sufficient)
- **Module 2 (Visual Fidelity)**: Add Unity 2022.3 LTS for high-fidelity visualization and human-robot interaction (HRI)
- **Module 3 (AI Workflows)**: Transition to Isaac Sim 2023.1+ for photorealistic simulation, synthetic data generation, and AI perception pipelines (RTX GPU required)

**Technology Stack Components**:
- Simulator 1: Gazebo Garden (native ROS 2, free, open-source)
- Simulator 2: Unity 2022.3 LTS (ROS-TCP-Connector bridge, free for personal use)
- Simulator 3: NVIDIA Isaac Sim 2023.1+ (Isaac ROS native, free with NVIDIA account)
- Integration: ROS 2 Humble as common interface across all three

## Consequences

### Positive

- **Progressive accessibility**: Learners can start with CPU-only hardware (Gazebo), upgrade to GPU for visual fidelity (Unity), then RTX GPU for AI workflows (Isaac Sim)
- **Full spectrum coverage**: From basic physics (Gazebo) → high-fidelity HRI (Unity) → photorealistic AI datasets (Isaac)
- **Industry relevance**: All three simulators widely used in robotics industry (Gazebo for prototyping, Unity for HRI, Isaac for AI training)
- **Modular independence**: Module 2 (Gazebo + Unity) completable without Isaac Sim, enabling learners to opt out at hardware barriers
- **Prevents vendor lock-in**: Multi-simulator expertise transfers across different robotics ecosystems

### Negative

- **Maintenance complexity**: Three separate simulation environments to document, update, and troubleshoot
- **Learning curve multiplication**: Learners must understand Gazebo SDF, Unity scenes, and Isaac USD workflows
- **Integration overhead**: Each simulator has different ROS 2 integration patterns (native Gazebo, ROS-TCP-Connector for Unity, Isaac ROS)
- **Content scope inflation**: Requires 3x more examples, screenshots, troubleshooting guides compared to single-simulator approach
- **Hardware barrier remains**: Isaac Sim modules still require RTX GPU, excluding CPU-only learners from Modules 3-4 (mitigated by cloud GPU alternative)

## Alternatives Considered

### Alternative A: Gazebo Garden Only (Lightweight Approach)
- **Stack**: Gazebo Garden exclusively for all simulation needs
- **Why rejected**:
  - Limited visual fidelity for HRI demonstrations
  - No native support for photorealistic synthetic data generation required for AI perception training
  - Gazebo Classic deprecated, Garden still maturing (fewer community examples vs. Isaac Sim for AI workflows)
- **Tradeoff**: Simplicity and accessibility vs. insufficient coverage of AI perception workflows

### Alternative B: Isaac Sim Only (AI-First Approach)
- **Stack**: NVIDIA Isaac Sim 2023.1+ for all modules
- **Why rejected**:
  - Steep learning curve for beginners (Omniverse, USD format, Isaac ROS)
  - RTX GPU mandatory from Module 1, excluding CPU-only learners (significant accessibility barrier)
  - Overkill for basic ROS 2 concepts (publisher/subscriber, TF2) - heavyweight tool for lightweight needs
- **Tradeoff**: Cutting-edge AI capabilities vs. inaccessibility and excessive complexity for foundational topics

### Alternative C: Unity Only (Game Developer Bridge)
- **Stack**: Unity 2022.3 LTS with ROS-TCP-Connector for all simulation
- **Why rejected**:
  - Requires ROS-TCP-Connector bridge (adds network latency, complexity vs. native Gazebo ROS 2)
  - Lacks native physics simulation optimizations for robotics (Unity physics engine general-purpose, not robotics-specific)
  - Photorealistic rendering less optimized than Isaac Sim for synthetic dataset generation
- **Tradeoff**: Game developer familiarity vs. non-native ROS 2 integration and physics limitations

### Comparison Table

| Aspect | Gazebo Garden | Unity | Isaac Sim |
|--------|---------------|-------|-----------|
| Realism | Medium | High (visual) | Very High (physics + visual) |
| Performance | Lightweight | Medium | GPU-intensive (RTX required) |
| Hardware Requirements | CPU sufficient | GPU recommended | RTX GPU mandatory |
| ROS 2 Integration | Native | Bridge (ROS-TCP-Connector) | Native (Isaac ROS) |
| Learning Curve | Low | Medium | High |
| Cost | Free | Free (Personal) | Free (with NVIDIA account) |

## References

- Feature Spec: `specs/001-physical-ai-ebook/spec.md`
- Implementation Plan: `specs/001-physical-ai-ebook/plan.md`
- Research Document: `specs/001-physical-ai-ebook/research.md` (Decision 1: Simulation Stack Selection)
- Related ADRs: None (first ADR)
- Evaluator Evidence: `history/prompts/001-physical-ai-ebook/0002-physical-ai-e-book-plan.plan.prompt.md` (Constitution check PASSED - all 6 principles validated)
