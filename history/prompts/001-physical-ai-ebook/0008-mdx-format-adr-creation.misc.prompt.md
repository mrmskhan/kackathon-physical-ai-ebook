---
id: 0008
title: MDX Format ADR Creation
stage: misc
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-ebook
branch: 001-physical-ai-ebook
user: developer
command: /sp.adr MDX for Enhanced Learner Interactivity
labels: ["adr", "architecture", "mdx", "markdown", "docusaurus", "interactivity", "contributor-experience"]
links:
  spec: specs/001-physical-ai-ebook/spec.md
  ticket: null
  adr: history/adr/0005-mdx-for-enhanced-learner-interactivity.md
  pr: null
files:
  - history/adr/0005-mdx-for-enhanced-learner-interactivity.md
tests:
  - Significance checklist validated (Impact: YES, Alternatives: YES, Scope: YES)
---

## Prompt

```text
/sp.adr MDX for Enhanced Learner Interactivity
```

## Response snapshot

Created ADR-0005 documenting the decision to use **MDX (Markdown + JSX components)** for all e-book content:

**Decision Rationale**:
- Base Format: MDX files (.mdx) with Markdown-first authoring
- Interactive Components: Tabbed code blocks, collapsible sections, custom callouts, Mermaid diagrams
- Backward Compatibility: Plain Markdown (.md) supported for gradual MDX adoption
- Contributor Workflow: MDX templates provided for common patterns, syntax documented in quickstart.md

**Significance Validation** (all 3 criteria met):
1. **Impact**: Affects contributor workflow, content authoring patterns, Docusaurus configuration, component library across all 5 modules
2. **Alternatives**: Evaluated pure Markdown (simplicity focus), full React components (maximum interactivity)
3. **Scope**: Cross-cutting concern affecting contributor guide, lesson templates, build process

**Positive Consequences**:
- Rich learning experience (tabbed code blocks for Python/C++ side-by-side, collapsible advanced sections)
- Native Docusaurus support (MDX is native format, zero additional plugins)
- Gradual complexity (start Markdown, adopt JSX components only when needed)
- Mermaid integration (architecture diagrams render natively)
- Reusable components (standardized callouts reduce copy-paste errors)

**Negative Consequences**:
- Contributor learning curve (MDX syntax requires understanding JSX props)
- Build complexity (MDX parsing adds +10-20 seconds to build time)
- Debugging difficulty (MDX syntax errors show cryptic JSX stack traces)
- Version lock-in (Docusaurus MDX upgrades may break custom components)
- IDE support fragmentation (not all Markdown editors support MDX highlighting)

**Alternatives Rejected**:
1. **Pure Markdown Only**: Limits interactivity (no tabs, no collapsible sections), Mermaid requires plugins, no component reuse
2. **Full React Components**: Too complex for contributors (requires React knowledge), defeats Markdown-first purpose, over-engineering for text content

**Comparison Table Included**: Simplicity, interactivity, contributor barrier, Docusaurus support across 3 formats

## Outcome

- ✅ Impact: Fifth ADR created, documents content authoring format affecting all modules and contributor workflow
- 🧪 Tests: Significance checklist passed (3/3 criteria validated - Impact/Alternatives/Scope)
- 📁 Files: Created ADR-0005 at history/adr/0005-mdx-for-enhanced-learner-interactivity.md
- 🔁 Next prompts: All 5 architecture decisions from research.md now documented as ADRs, proceed with `/sp.implement` to begin task execution
- 🧠 Reflection: ADR documents MDX as sweet spot between simplicity (Markdown-first) and interactivity (JSX components). Backward compatibility with .md files reduces risk. Templates and contributor guide lower MDX learning curve. Decision prioritizes learner engagement (rich interactive components) while maintaining contributor accessibility (gradual JSX adoption).

## Evaluation notes (flywheel)

- Failure modes observed: None (ADR creation successful)
- Graders run and results (PASS/FAIL): Significance checklist PASS - all 3 criteria (Impact/Alternatives/Scope) met, decision cluster includes format (MDX), diagrams (Mermaid), components (custom React), validation (Docusaurus build + ESLint), templates (pre-built patterns)
- Prompt variant (if applicable): N/A (baseline)
- Next experiment (smallest change to try): During implementation, validate that MDX templates (lesson template, code example template) include inline comments explaining JSX component usage to accelerate contributor onboarding
