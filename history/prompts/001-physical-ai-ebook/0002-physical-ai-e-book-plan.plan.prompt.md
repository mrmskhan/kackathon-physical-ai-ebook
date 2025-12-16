---
id: 0002
title: Physical AI E-Book Plan
stage: plan
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-ebook
branch: 001-physical-ai-ebook
user: developer
command: /sp.plan
labels: ["plan", "architecture", "robotics", "ros2", "simulation", "vla", "docusaurus"]
links:
  spec: specs/001-physical-ai-ebook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-ebook/plan.md
  - specs/001-physical-ai-ebook/research.md
  - specs/001-physical-ai-ebook/data-model.md
  - specs/001-physical-ai-ebook/quickstart.md
  - specs/001-physical-ai-ebook/contracts/module-schema.json
  - specs/001-physical-ai-ebook/contracts/code-example-schema.json
  - specs/001-physical-ai-ebook/contracts/citation-schema.json
  - CLAUDE.md
tests:
  - none
---

## Prompt

```text
AI/Spec-Driven E-Book on Physical AI & Humanoid Robotics

Create:
- Architecture sketch for full E-Book structure (Docusaurus layout, modules, build flow)
- Section hierarchy based on 4 learning modules + hardware + capstone
- Research approach aligned with robotics documentation (ROS 2, Gazebo, Unity, NVIDIA Isaac, Whisper, VLA)
- Quality validation checklist (accuracy, reproducibility, APA citations, Docusaurus build success)

Decisions needing documentation:
1. **Simulation stack choice**
   - Options: Gazebo Classic | Gazebo Garden | Unity | Isaac Sim
   - Tradeoffs: realism, performance, hardware requirements

2. **Hardware environment**
   - Local RTX workstation vs. Cloud GPU (AWS g5/g6)
   - Tradeoffs: cost, latency, reliability

3. **Robot platform**
   - Unitree Go2 | Unitree G1 | OP3 | Proxy robots (arms, quadrupeds)
   - Tradeoffs: cost vs. humanoid fidelity

4. **VLA integration path**
   - Whisper + LLM planning → ROS 2 actions
   - Options: OpenAI, local models, cloud inference
   - Tradeoffs: latency, accuracy, hardware load

5. **Documentation structure**
   - MDX vs. pure Markdown
   - Tradeoffs: interactivity vs. simplicity

Testing strategy:
- Validate commands by running them on Ubuntu 22.04
- Verify each ROS 2, Gazebo, Unity, and Isaac workflow reproduces exact results
- Check Docusaurus compiles without errors (`npm run start`)
- Confirm diagrams, code blocks, and MDX components render correctly
- Ensure APA citation compliance from Constitution
- Run consistency checks for module ordering and learning outcomes
- End-to-end test: Capstone pipeline must function conceptually (Voice → Whisper → LLM → ROS 2 → Navigation → Perception → Manipulation)

Technical details:
- Use **research-concurrent writing** (research while writing, not upfront)
- Follow APA citation style defined in Constitution
- Organize the writing plan by phases:
  1. **Research**
  2. **Foundation**
  3. **Analysis**
  4. **Synthesis**
```

## Response snapshot

Created comprehensive implementation plan for Physical AI & Humanoid Robotics E-Book including:

**Architecture Decisions (research.md)**:
1. Multi-simulator approach: Gazebo Garden (accessible) + Unity (visual fidelity) + Isaac Sim (AI workflows)
2. Local RTX workstation primary, cloud GPU (AWS g5/g6) documented alternative
3. Multi-platform coverage: Unitree Go2 ($1600 quadruped), ROBOTIS OP3 ($10k+ humanoid), simulation proxies
4. VLA: Whisper (local) + flexible LLM (OpenAI API or local Llama) → ROS 2 action servers
5. MDX format for interactivity (tabs, Mermaid, custom components) while maintaining Markdown simplicity

**Data Model (data-model.md)**:
- 7 entities: Module, LearningObjective, Lesson, CodeExample, Citation, Diagram, TroubleshootingEntry
- Validation rules enforcing constitution compliance (Flesch-Kincaid 9-12, min citations, reproducible code)
- Entity relationships diagram (Mermaid ERD)

**Content Structure (plan.md)**:
- Docusaurus site in `my-website/docs/` with 5 modules + capstone
- Each module: index.md, lessons (01-*.md), examples/, sources.md (APA citations)
- Constitution check: ✅ ALL 6 PRINCIPLES PASSED

**Contracts (JSON schemas)**:
- module-schema.json: Validates module structure, learning objectives, citations (min 15: 10 official + 5 technical)
- code-example-schema.json: Ensures reproducible code with prerequisites, setup, execution, expected output
- citation-schema.json: Enforces APA 7th edition format, version-specific URLs

**Quickstart Guide (quickstart.md)**:
- Contributor setup: Node.js, Docusaurus local dev server, content structure
- Learner environment setup: ROS 2 Humble, Gazebo Garden, Unity 2022.3 LTS, Isaac Sim, Whisper
- Quality checklist: Build validation, code testing, readability, citations

**Technology Stack**:
- Markdown/MDX, Python 3.10+, Node.js 18+, Docusaurus 3.x
- ROS 2 Humble, Gazebo Garden, Unity 2022.3 LTS, Isaac Sim 2023.1+, Whisper, Mermaid.js
- GitHub Pages deployment, Ubuntu 22.04 LTS target platform

## Outcome

- ✅ Impact: Complete implementation plan with architecture decisions, data models, contracts, and quickstart guide for 5-module e-book
- 🧪 Tests: Constitution check PASSED (all 6 principles validated), JSON schemas enforce quality gates
- 📁 Files: Created plan.md, research.md, data-model.md, quickstart.md, 3 JSON schema contracts
- 🔁 Next prompts: `/sp.tasks` to generate discrete writing tasks per module with acceptance criteria
- 🧠 Reflection: Plan balances technical rigor (verified sources, reproducible code, APA citations) with accessibility (multi-simulator approach, cloud GPU alternative, grade 9-12 writing). Research decisions documented with rationale and tradeoffs for transparency.

## Evaluation notes (flywheel)

- Failure modes observed: None (planning phase successful)
- Graders run and results (PASS/FAIL): Constitution check PASSED - all 6 principles align with plan architecture
- Prompt variant (if applicable): N/A (baseline)
- Next experiment (smallest change to try): During tasks generation, validate that Module 1 (P1) can be completed independently without Module 2-5 dependencies, ensuring true modular structure
