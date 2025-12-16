# AI/Spec-Driven E-Book on Modern Robotics & AI Tools Constitution

<!--
SYNC IMPACT REPORT:
Version change: INITIAL → 1.0.0
Modified principles: N/A (initial creation)
Added sections:
  - Core Principles (6 principles defined)
  - Content Standards
  - Development Workflow
  - Governance
Templates requiring updates:
  ✅ plan-template.md - Constitution Check section aligns with principles
  ✅ spec-template.md - Requirements align with verification and modularity principles
  ✅ tasks-template.md - Task organization reflects build verification and modular structure
Follow-up TODOs: None
-->

## Core Principles

### I. Verified Information Only (NON-NEGOTIABLE)

All content MUST be sourced from official documentation or verified technical sources. No hallucinated APIs, fabricated code examples, or unverified claims are permitted.

**Rules:**
- Every code snippet must be tested and reproducible on Ubuntu
- API references must link to official documentation
- All technical claims require citation (APA format)
- Minimum 10 official sources + 5 technical sources per major topic
- Context7 MCP must be used to fetch current official documentation

**Rationale:** Accuracy is paramount for a technical e-book. Readers must trust that every code example works and every API reference is current. Unverified information damages credibility and wastes reader time debugging non-existent features.

### II. Build-First Verification

Every chapter and code example MUST pass Docusaurus build without errors before being considered complete.

**Rules:**
- Docusaurus build (`npm run build`) must succeed with zero errors
- All MDX syntax must be valid
- All links (internal and external to official docs) must resolve
- Code blocks must use correct language tags for syntax highlighting
- No broken image references or missing assets

**Rationale:** A broken build means broken content delivery. If the e-book can't deploy, it can't serve readers. Build verification is the definitive test that content is production-ready.

### III. Clear Writing for Target Audience

All content must be written at grade 9-12 reading level, optimized for intermediate developers (1-3 years experience).

**Rules:**
- Assume reader knows programming basics (variables, functions, loops)
- Explain domain-specific concepts (ROS nodes, AI model architectures)
- Use active voice and concrete examples
- Define acronyms on first use
- Include "Why this matters" context for complex topics
- Avoid jargon without explanation; prefer plain language

**Rationale:** Technical precision without clarity fails readers. The sweet spot is explaining advanced concepts using accessible language, not dumbing down but not gatekeeping either.

### IV. Strict Modular Structure

All content development follows the SpecKit-Plus workflow: constitution → spec → plan → tasks → implementation.

**Rules:**
- Every chapter starts with a spec defining learning objectives and success criteria
- Plan documents must outline structure, sources, and dependencies before writing
- Tasks break writing into discrete, testable units (sections/subsections)
- No content created outside this workflow
- Each module (chapter) must be independently buildable and testable

**Rationale:** Modular structure enables parallel development, clear acceptance criteria, and systematic quality control. It prevents scope creep and ensures every piece of content has a defined purpose and measurable outcome.

### V. Reproducible Code Standards

All code examples must be copy-paste executable on a standard Ubuntu environment.

**Rules:**
- Include setup instructions (dependencies, environment variables)
- Use standard package managers (apt, pip, npm)
- Specify exact versions for critical dependencies
- Provide sample data/configuration files where needed
- Test on clean Ubuntu install (via container/VM)
- Include expected output for each example

**Rationale:** Non-reproducible examples frustrate readers and erode trust. If code doesn't work as shown, the e-book becomes a source of confusion rather than learning.

### VI. Citation and Attribution Discipline

All external sources must be properly cited using APA format with traceable links.

**Rules:**
- Cite official documentation for all API/library references
- Cite research papers for algorithmic concepts
- Cite tutorials/guides when adapting examples (with modification notes)
- Maintain `sources.md` in each chapter spec directory
- Link citations to version-specific documentation (e.g., ROS2 Humble, TensorFlow 2.x)
- No generic "from the internet" or undocumented copy-paste

**Rationale:** Academic integrity and legal compliance. Readers deserve to trace claims to authoritative sources. Proper attribution respects original authors and enables readers to explore topics deeper.

## Content Standards

### Technical Accuracy
- All robotics concepts verified against official ROS/ROS2 documentation
- All AI/ML examples tested with specified framework versions (TensorFlow, PyTorch, etc.)
- All tool usage validated against official tool documentation (Claude, Context7, SpecKit-Plus)

### Documentation Format
- Markdown/MDX only (Docusaurus-compatible)
- Frontmatter metadata required for every document
- Consistent heading hierarchy (H1 = chapter, H2 = section, H3 = subsection)
- Code blocks must specify language for syntax highlighting

### Source Requirements
- Official documentation: framework docs, RFC specifications, IEEE papers
- Technical sources: peer-reviewed papers, established technical blogs (with author credentials)
- Prohibited: Wikipedia as sole source, uncited Stack Overflow snippets, abandoned GitHub repos

## Development Workflow

### Feature Lifecycle (SpecKit-Plus Compliance)
1. **Constitution**: This document defines non-negotiable principles
2. **Specification** (`/sp.specify`): Define chapter learning objectives, audience, success criteria
3. **Planning** (`/sp.plan`): Research sources, outline structure, identify dependencies
4. **Tasks** (`/sp.tasks`): Break into discrete writing/coding units with acceptance tests
5. **Implementation**: Write content, validate build, verify sources
6. **Review**: Cross-reference constitution compliance, peer review for clarity

### Quality Gates
- **Gate 1 (Spec)**: Learning objectives measurable, target audience clear, success criteria defined
- **Gate 2 (Plan)**: Minimum sources identified, structure logical, no gaps in coverage
- **Gate 3 (Tasks)**: Each task has acceptance criteria, dependencies mapped
- **Gate 4 (Implementation)**: Build passes, code runs, citations complete
- **Gate 5 (Review)**: Constitution compliance verified, readability confirmed

### Tooling Requirements
- **Docusaurus**: Official setup only, no custom plugins without documented justification
- **Context7 MCP**: Primary source for official documentation retrieval
- **SpecKit-Plus**: Workflow orchestration and artifact generation
- **Claude Code**: AI-assisted content generation with human verification
- **GitHub Pages**: Deployment target

### Version Control
- Feature branches for each chapter: `feat/chapter-<number>-<slug>`
- Atomic commits per task completion
- Pull request requires: build passing, peer review, constitution checklist

## Governance

### Constitution Authority
This constitution supersedes all other project documentation. When conflicts arise between this document and other guidance, constitution principles take precedence.

### Amendment Process
1. Propose amendment via GitHub issue tagged `constitution-amendment`
2. Justify need: what problem does current principle create? What does amendment solve?
3. Review impact: which existing content violates new principle? Migration plan?
4. Approval: Requires explicit consensus from all active contributors
5. Version bump: MAJOR for breaking changes, MINOR for new principles, PATCH for clarifications
6. Update dependent templates: plan-template, spec-template, tasks-template must reflect changes
7. Communicate: Announce in project README and commit message

### Compliance Review
- Every pull request must include constitution compliance checklist
- Monthly audit of merged content against current constitution version
- Non-compliant content flagged for remediation or removal
- Persistent non-compliance blocks further feature development

### Exception Handling
Exceptions to constitutional principles require:
1. Documented justification in `specs/<feature>/complexity.md`
2. Alternative approaches considered and rejected (with reasons)
3. Explicit time-bound scope (exception does not set new precedent)
4. Review and approval from all contributors

### Living Document
- Constitution is versioned with the project
- Changes tracked in git history with detailed commit messages
- Breaking changes communicated proactively to all contributors
- Regular review (quarterly) to ensure principles still serve project goals

**Version**: 1.0.0 | **Ratified**: 2025-12-13 | **Last Amended**: 2025-12-13
