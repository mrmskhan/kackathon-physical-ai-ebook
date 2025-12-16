# ADR-0005: MDX for Enhanced Learner Interactivity

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-13
- **Feature:** 001-physical-ai-ebook
- **Context:** E-book needs interactive learning features (tabbed code blocks, collapsible sections, callouts) while maintaining contributor-friendly Markdown writing experience

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - affects contributor workflow, content authoring patterns, Docusaurus config
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - pure Markdown, full React components evaluated
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects all 5 modules, contributor guide, component library
-->

## Decision

Use **MDX (Markdown + JSX components)** for all e-book content:

- **Base Format**: MDX files (.mdx extension) with Markdown-first authoring
- **Interactive Components**:
  - Tabbed code blocks (multi-language examples: Python, C++, YAML)
  - Collapsible sections (advanced topics, troubleshooting FAQs)
  - Custom callouts ("Try It Yourself", "Prerequisites", "Version Warning")
  - Mermaid diagrams (flowcharts, sequence diagrams, architecture ERDs)
- **Backward Compatibility**: Plain Markdown (.md) supported, gradual MDX adoption
- **Contributor Workflow**: MDX templates provided for common patterns, syntax documented in quickstart.md

**Content Authoring Stack**:
- Format: MDX (Markdown with JSX)
- Diagrams: Mermaid.js (native Docusaurus support)
- Components: Custom React components for callouts, tabs, collapsibles
- Validation: Docusaurus build checks MDX syntax, ESLint for JSX code
- Templates: Pre-built MDX patterns (lesson template, code example template, troubleshooting template)

## Consequences

### Positive

- **Rich learning experience**: Tabbed code blocks enable side-by-side Python/C++ examples, collapsible sections hide advanced content for beginners
- **Native Docusaurus support**: MDX is Docusaurus's native format, zero additional plugins required for JSX components
- **Gradual complexity**: Contributors start with plain Markdown, adopt JSX components only when needed for interactivity
- **Mermaid integration**: Architecture diagrams, flowcharts, ERDs render natively without external image hosting
- **Reusable components**: "Prerequisites" callout component standardizes format across all lessons, reducing copy-paste errors

### Negative

- **Contributor learning curve**: MDX syntax (JSX in Markdown) requires understanding component props vs. pure Markdown simplicity
- **Build complexity**: MDX parsing adds build time vs. plain Markdown (estimated +10-20 seconds for full site build)
- **Debugging difficulty**: MDX syntax errors harder to diagnose than Markdown (JSX component errors show cryptic stack traces)
- **Version lock-in**: Docusaurus MDX version upgrades may break custom components (requires migration testing)
- **IDE support fragmentation**: Not all Markdown editors support MDX syntax highlighting (VS Code requires MDX extension)

## Alternatives Considered

### Alternative A: Pure Markdown Only (Simplicity Focus)
- **Stack**: Standard Markdown (.md files), no JSX components, static code blocks
- **Why rejected**:
  - Limits interactivity (no tabs for multi-language examples, no collapsible advanced sections)
  - Harder to create rich learning experiences (prerequisites must be plain text, not styled callouts)
  - Mermaid diagrams require plugins or external image hosting (vs. native MDX support)
  - No component reuse (every "Try It Yourself" callout copy-pasted, inconsistent formatting)
- **Tradeoff**: Lowest contributor barrier vs. limited learner engagement and content consistency

### Alternative B: Full React Components (Maximum Interactivity)
- **Stack**: React .tsx components for all lessons, no Markdown, pure JSX/TSX
- **Why rejected**:
  - Too complex for content contributors (requires React knowledge, JSX syntax, component lifecycle)
  - Defeats purpose of Markdown-first writing (content buried in JSX tags, poor readability)
  - High barrier to entry (contributors must know React to fix typos or add citations)
  - Over-engineering for text content (ROS 2 installation instructions don't need component state management)
- **Tradeoff**: Maximum interactivity and customization vs. prohibitive contributor complexity

### Comparison Table

| Format | Simplicity | Interactivity | Contributor Barrier | Docusaurus Support |
|--------|-----------|---------------|---------------------|-------------------|
| Pure Markdown | Very High | Low | Very Low | Full |
| MDX | High | High | Low-Medium | Full (native) |
| React Components | Low | Very High | High | Full |

## References

- Feature Spec: `specs/001-physical-ai-ebook/spec.md`
- Implementation Plan: `specs/001-physical-ai-ebook/plan.md`
- Research Document: `specs/001-physical-ai-ebook/research.md` (Decision 5: Documentation Format)
- Related ADRs: None (content format is orthogonal to simulation, hardware, VLA decisions)
- Evaluator Evidence: `history/prompts/001-physical-ai-ebook/0002-physical-ai-e-book-plan.plan.prompt.md` (Constitution check PASSED)
