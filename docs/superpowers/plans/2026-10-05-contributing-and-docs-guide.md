# Autonomy Binder Contributing & RoveSoDocs Integration Guide Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create Chapter 16 of the Autonomy Software Binder (`Contributing_and_Docs_Integration.md`) providing an authoritative guide on contributing to the Autonomy Binder and a turnkey blueprint for integrating any MRDT repository into `docs.themrdt.org`.

**Architecture:** A comprehensive Markdown document placed in `docs/autonomy_binder/16_Contributing_and_Docs_Guide/` adhering to the binder's formatting and Pandoc build rules. The binder's master Table of Contents (`00_Table_of_Contents.md`), Jekyll index (`index.md`), and sidebar navigation (`_config.yml`) will be updated to link and index the new chapter.

**Tech Stack:** Markdown (GFM), Jekyll 3.10 / GitHub Pages, MathJax 3 (SVG engine), Pandoc, Bash, GitHub Actions CI, VitePress.

## Global Constraints
- Target repository: `Autonomy_Software` on branch `docs/autonomy-binder`.
- No placeholders, TODOs, or incomplete sections.
- Strict adherence to Pandoc PDF compiler compatibility (leading blank lines before bullet lists, no raw HTML linebreaks).
- Mathematical formulas formatted for MathJax 3 with proper backtick protection for shell variables.

---

### Task 1: Create Chapter 16 (`Contributing_and_Docs_Integration.md`)

**Files:**
- Create: `docs/autonomy_binder/16_Contributing_and_Docs_Guide/Contributing_and_Docs_Integration.md`

**Interfaces:**
- Consumes: Design spec from `docs/superpowers/specs/2026-10-05-contributing-and-docs-guide-design.md`.
- Produces: Complete, self-contained documentation chapter covering:
  - Philosophy & Dual-Docset Architecture (`_d` vs `_j`).
  - Autonomy Binder Authoring & Local Workflow (branches, directory structure, Markdown/MathJax standards, local Jekyll serve, Pandoc single MD & PDF compilation).
  - Generalized RoveSoDocs Integration Playbook (Source repo setup, `deploy.yml` workflow, `NotFoundContent.vue` SPA routing, `build-minisearch-index.mjs` indexing, `index.md` homepage cards, MathJax/CSS grid layout setup).

- [ ] **Step 1: Create directory and write the comprehensive Chapter 16 markdown file**

Create `docs/autonomy_binder/16_Contributing_and_Docs_Guide/Contributing_and_Docs_Integration.md` with:
- Detailed introduction and overview.
- Section 1: The MRDT Documentation Architecture (Dual-Docset model).
- Section 2: Contributing to the Autonomy Binder (Branch strategy, folder layout, style guide, LaTeX math formatting, local Jekyll testing, Pandoc compile commands).
- Section 3: Generalized RoveSoDocs Hub Integration Playbook (6-step recipe for onboarding any repository into `docs.themrdt.org`).
- Section 4: Troubleshooting & Best Practices (Jekyll gem template parsing bug prevention via `exclude:`, CSS grid scaling, SPA 404 cache busting).

- [ ] **Step 2: Verify file existence and non-empty size**

Run: `ls -la docs/autonomy_binder/16_Contributing_and_Docs_Guide/Contributing_and_Docs_Integration.md`
Expected: File exists with > 10,000 bytes.

- [ ] **Step 3: Commit the new chapter file**

```bash
git add docs/autonomy_binder/16_Contributing_and_Docs_Guide/Contributing_and_Docs_Integration.md
git commit -m "Add Chapter 16: Contributing and Docs Integration guide"
```

---

### Task 2: Update Table of Contents & Web Index

**Files:**
- Modify: `docs/autonomy_binder/00_Table_of_Contents.md:65-70`
- Modify: `docs/autonomy_binder/index.md:65-75`

**Interfaces:**
- Consumes: Path `16_Contributing_and_Docs_Guide/Contributing_and_Docs_Integration.md`.
- Produces: Updated TOC that `compile_binder_pandoc.sh` parses to append Chapter 16 to the single PDF.

- [ ] **Step 1: Update `00_Table_of_Contents.md`**

Add section:
```markdown
### Contributing & Docs Integration
- [Contributing & Docs Integration Guide](16_Contributing_and_Docs_Guide/Contributing_and_Docs_Integration.md)
```

- [ ] **Step 2: Update `index.md`**

Add Chapter 16 card / bullet under Contributing & Docs Integration on the Jekyll home page.

- [ ] **Step 3: Commit updates to `00_Table_of_Contents.md` and `index.md`**

```bash
git add docs/autonomy_binder/00_Table_of_Contents.md docs/autonomy_binder/index.md
git commit -m "Index Chapter 16 in Table of Contents and landing page"
```

---

### Task 3: Update Jekyll Sidebar Navigation

**Files:**
- Modify: `docs/autonomy_binder/_config.yml:15-30`

**Interfaces:**
- Consumes: Chapter 16 HTML path `/16_Contributing_and_Docs_Guide/Contributing_and_Docs_Integration.html`.
- Produces: Direct sidebar navigation item for Chapter 16 on the web site.

- [ ] **Step 1: Add Chapter 16 to `navigation` list in `_config.yml`**

Add:
```yaml
  - link: "/16_Contributing_and_Docs_Guide/Contributing_and_Docs_Integration.html"
    name: "Contributing & Docs Guide"
```

- [ ] **Step 2: Commit `_config.yml`**

```bash
git add docs/autonomy_binder/_config.yml
git commit -m "Add Chapter 16 to Jekyll sidebar navigation"
```

---

### Task 4: Verify Pandoc Compilation & Git State

**Files:**
- Test: `tools/compile_binder_pandoc.sh`

- [ ] **Step 1: Run Pandoc compilation script**

Run: `bash tools/compile_binder_pandoc.sh`
Expected: Script discovers Chapter 16 in `00_Table_of_Contents.md`, outputs `Appending 16_Contributing_and_Docs_Guide/Contributing_and_Docs_Integration.md...`, and updates `Autonomy_Binder_Pandoc.md` (and `Autonomy_Binder_Pandoc.pdf` if Pandoc/LaTeX is present).

- [ ] **Step 2: Verify `git status`**

Run: `git status`
Expected: Clean working tree on `docs/autonomy-binder`.
