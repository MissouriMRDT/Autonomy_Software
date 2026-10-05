# Contributing to Autonomy Binder & RoveSoDocs Integration Design Specification

- **Date**: 2026-10-05
- **Branch**: `docs/autonomy-binder`
- **Target File**: `docs/autonomy_binder/16_Contributing_and_Docs_Guide/Contributing_and_Docs_Integration.md`
- **Companion Updates**:
  - `docs/autonomy_binder/00_Table_of_Contents.md`
  - `docs/autonomy_binder/index.md`
  - `docs/autonomy_binder/_config.yml`

---

## 1. Executive Summary

This specification defines the creation of Chapter 16 of the Autonomy Software Binder: **Contributing & Documentation Integration Guide**. This document serves two core purposes:
1. **Internal Autonomy Contributor Guide**: Details how team members write, format, test, and compile the Autonomy Software Binder into HTML and PDF.
2. **Generalized MRDT Docs Hub Playbook**: Provides an end-to-end blueprint for connecting any Missouri S&T Mars Rover Design Team repository (Autonomy, Simulator, Embedded, Science, Arm, etc.) to the central documentation portal at `https://docs.themrdt.org` (managed via `MissouriMRDT/RoveSoDocs`).

---

## 2. Document Architecture & Outline

The chapter will be saved to:
`docs/autonomy_binder/16_Contributing_and_Docs_Guide/Contributing_and_Docs_Integration.md`

### Section 1: Overview & Documentation Philosophy
- Role of the Autonomy Binder as the engineering source of truth and operations manual.
- Dual-docset model: Doxygen (`_d/`) for low-level C++ API interfaces; Jekyll Binder (`_j/`) for high-level system architecture, algorithms, and operations manuals.

### Section 2: Autonomy Binder Authoring & Local Workflow
- **Branching Model**:
  - Maintenance on dedicated `docs/autonomy-binder` branch to isolate large build artifacts (PDFs, Pandoc scripts, Jekyll assets) from the active C++ `development` branch.
  - Periodic synchronization (`git merge development`) to incorporate new code features into documentation.
- **Directory Hierarchy & File Conventions**:
  - Numeric prefixes (`00_` through `16_`) for deterministic order in the Table of Contents and Pandoc PDF concatenation.
  - Subsystem deep dives (`04_Subsystems_Deep_Dive/`), board drivers (`07_Board_Drivers/`), handlers (`08_Handlers/`), etc.
  - Markdown standards: GFM (GitHub Flavored Markdown), heading hierarchy (`#`, `##`, `###`), standard tables, and bullet lists with leading blank lines.
- **Mathematical Formulations (LaTeX & MathJax 3)**:
  - Inline equations: `$ ... $` or `\( ... \)`.
  - Display equations: `$$ ... $$` or `\[ ... \]`.
  - Escaping rules and code protection (always enclosing shell variables like `$(nproc)` in backticks to prevent MathJax parsing).
- **Local Compilation & Preview**:
  - Building the web documentation locally with Jekyll / Bundler:
    ```bash
    bundle install --path vendor/bundle
    bundle exec jekyll serve
    ```
  - Compiling the unified Markdown and single PDF using Pandoc:
    ```bash
    bash tools/compile_binder_pandoc.sh
    # or
    bash tools/compile_binder_pdf.sh
    ```
  - Dependencies: Pandoc and `pdflatex` (TeX Live / MiKTeX).

### Section 3: Generalized RoveSoDocs Integration Playbook (For Any MRDT Repo)
A comprehensive playbook outlining the exact 6-step procedure for adding any team repository to `docs.themrdt.org`:

1. **Step 1: Source Repository Preparation**:
   - Creating a dedicated documentation branch (e.g., `docs/autonomy-binder` or `github-pages`).
   - Adding a minimal `Gemfile`:
     ```ruby
     source "https://rubygems.org"
     gem "github-pages", group: :jekyll_plugins
     ```
   - Configuring `_config.yml` with essential settings:
     - `baseurl: "/<repo>/_j"` and `url: "https://docs.themrdt.org"`
     - `remote_theme: "godalming123/minimal"`
     - `plugins:` including `jekyll-remote-theme` and `jekyll-optional-front-matter`
     - **Critical Exclusions**: Explicitly excluding `vendor`, `vendor/*`, `.bundle`, `.sass-cache`, `Gemfile`, and `*.pdf` so Jekyll 3.x does not attempt to parse raw gem ERB templates.
     - Layout defaults: `defaults: [{ scope: { path: "" }, values: { layout: "default" } }]`.
   - Layout overrides (`_layouts/default.html` and `_includes/navigation.html`) for sidebar menu navigation, "Return to RoveSoDocs" link, and responsive CSS grid widening.
   - Injecting MathJax 3 in `_includes/head-custom.html`.

2. **Step 2: Deployment Workflow Setup in `RoveSoDocs` (`.github/workflows/deploy.yml`)**:
   - Creating a dedicated build job: `build_<repo>_jekyll` or `build_<repo>_doxygen`.
   - Fetching the specific documentation branch via `actions/checkout@v4`.
   - Installing Ruby gems into `vendor/bundle` without cache to avoid corrupt state.
   - Building the site into `out/<repo>/_j` using `bundle exec jekyll build --trace --baseurl "/<repo>/_j" -d "$GITHUB_WORKSPACE/out/<repo>/_j"`.
   - Uploading artifacts via `actions/upload-artifact@v4`.

3. **Step 3: Merging Artifacts in the `assemble` Job**:
   - Adding the new build job to `assemble.needs`.
   - Downloading artifact into `parts/<repo>-jekyll-out/`.
   - Overlaying into `dist/` with `rsync -a "parts/<repo>-jekyll-out/" dist/`.
   - Ensuring `touch dist/.nojekyll` is present to disable GitHub Pages root Jekyll processing.

4. **Step 4: Client-Side SPA Routing & 404 Refresh (`.vitepress/theme/NotFoundContent.vue`)**:
   - Adding `/<repo>/_j/` and `/<repo>/_d/` to `REFRESH_PREFIXES` array to trigger browser hard reload for client-side deep links.
   - Adding convenient direct quicklinks to the 404 page.

5. **Step 5: Unified Search Indexing (`tools/build-minisearch-index.mjs`)**:
   - Updating `sectionFromUrl(url)` with matching regex/prefix rules so the documentation pages are indexed under clean human-readable section titles (e.g., `"<Repo> Documentation (Binder)"`).

6. **Step 6: VitePress Homepage Feature Cards (`index.md`)**:
   - Adding feature cards in `index.md` with appropriate emojis/icons, titles, descriptions, and direct links.
   - Maintaining balance in the 3-column / 2-column responsive card grid.

---

## 3. Cross-Linking & Navigation Integration

1. **`docs/autonomy_binder/00_Table_of_Contents.md`**:
   - Add new section:
     ```markdown
     ### Contributing & Docs Integration
     - [Contributing & Docs Integration Guide](16_Contributing_and_Docs_Guide/Contributing_and_Docs_Integration.md)
     ```
   - This ensures `compile_binder_pandoc.sh` automatically reads and appends Chapter 16 when compiling the PDF.

2. **`docs/autonomy_binder/index.md`**:
   - Add Chapter 16 to the Table of Contents feature list on the Jekyll homepage.

3. **`docs/autonomy_binder/_config.yml`**:
   - Add navigation entry in `site.navigation`:
     ```yaml
     - link: "/16_Contributing_and_Docs_Guide/Contributing_and_Docs_Integration.html"
       name: "Contributing & Docs Guide"
     ```

---

## 4. Verification & Validation Plan

1. **Pandoc Script Validation**:
   - Run `bash tools/compile_binder_pandoc.sh` in Dev Container/Linux or verify Markdown generation:
     - Ensure Chapter 16 is picked up by `grep -o '([0-9a-zA-Z_/]*\.md)'`.
     - Ensure no broken anchors or invalid unescaped characters disrupt compilation.
2. **Jekyll Link Validation**:
   - Verify that all relative links resolve to existing files.
   - Check `navigation.html` rendering.
3. **Git Cleanliness**:
   - Verify all files are tracked and committed cleanly to `docs/autonomy-binder`.
