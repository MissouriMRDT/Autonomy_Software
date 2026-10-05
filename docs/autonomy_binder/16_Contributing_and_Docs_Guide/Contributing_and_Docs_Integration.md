# Contributing & Documentation Integration Guide

This guide details the end-to-end workflow for contributing to the **Autonomy Software Binder**, as well as the **Generalized MRDT Docs Hub Playbook** used to integrate any Missouri S&T Mars Rover Design Team repository into the centralized documentation portal at [docs.themrdt.org](https://docs.themrdt.org/).

---

## 1. Documentation Philosophy & Architecture

MRDT Software employs a **Dual-Docset Architecture** to serve different engineering needs without cluttering codebases or confusing users:

| Documentation Type | Engine | Route Subpath | Primary Audience & Purpose |
| :--- | :--- | :--- | :--- |
| **API Reference (Doxygen)** | Doxygen + Graphviz | `/<repo>/_d/` | Developers requiring low-level C++ class declarations, function signatures, inheritance graphs, and inline source comments. |
| **Engineering Manual (Binder)** | Jekyll / GitHub Pages / Pandoc | `/<repo>/_j/` | System architects, field operators, and new members requiring high-level subsystem deep dives, state machine workflows, tuning guides, and deployment checklists. |
| **Documentation Hub** | VitePress | `/` | Central landing portal, global search index, and directory of all MRDT software projects. |

---

## 2. Contributing to the Autonomy Binder

### A. Repository & Branch Strategy

To ensure that heavy build artifacts (compiled PDFs, LaTeX packages, pandoc build scripts, and Jekyll web templates) do not bloat the core C++ codebase, documentation is maintained on an isolated branch:

* **Production Code**: `development` branch (contains autonomy algorithms, drivers, state machines, and unit tests).
* **Documentation**: `docs/autonomy-binder` branch (contains `docs/autonomy_binder/` markdown files, `Gemfile`, `_config.yml`, and Pandoc compilation scripts).

#### Workflow for Documentation Updates:
1. Check out the documentation branch:
   ```bash
   git checkout docs/autonomy-binder
   ```
2. When new features or major refactors land in `development`, sync the branch:
   ```bash
   git merge development
   ```
3. Make your documentation additions or edits inside `docs/autonomy_binder/`.
4. Commit your changes and open a pull request targeting `docs/autonomy-binder`.

---

### B. Directory Structure & File Naming Conventions

All binder documentation lives in `docs/autonomy_binder/`. Chapters use a **two-digit numeric prefix** (`00_` to `16_`) to guarantee deterministic ordering across both the web navigation and the compiled Pandoc PDF:

```text
docs/autonomy_binder/
├── 00_Table_of_Contents.md                      # Master TOC parsed by Pandoc compile script
├── 01_Quick_Start_and_Ops.md                    # Environment setup, build instructions, hotkeys
├── 02_Architecture_Overview.md                  # High-level architecture & pipeline diagrams
├── 03_The_State_Machine.md                      # Autonomy State Machine specification
├── 04_Subsystems_Deep_Dive/                     # Subsystem engineering specifications
│   ├── Control_and_Actuation.md
│   ├── Path_Planning.md
│   └── Perception.md
├── 05_Configuration_and_Tuning/                 # Autonomy constants and CMake build flags
│   ├── Autonomy_Constants.md
│   └── CMake_Options.md
├── 06_Troubleshooting_Guide.md                  # Field diagnostics & fault recovery
├── 07_Board_Drivers/                            # Drive, Nav, and Multimedia board drivers
├── 08_Handlers/                                 # Subsystem handlers (Camera, LiDAR, Waypoints)
├── 09_Threading/                                # Thread pools & AutonomyThread interface
├── 10_Networking/                               # RoveComm UDP/TCP protocol specification
├── 11_Logs_and_Data/                            # Quill logging, video recording, visualization
├── 12_Vision/                                   # Cameras, ArUco detection, YOLO object detector
├── 13_Controllers/                              # PID, Pure Pursuit, and Predictive Stanley
├── 14_URC_Rules/                                # URC 2027 Autonomy Rules specification
├── 15_Checklists/                               # Field checklists and release procedure
├── 16_Contributing_and_Docs_Guide/              # This contribution & integration guide
│   └── Contributing_and_Docs_Integration.md
├── _includes/                                   # Jekyll layout components
│   ├── head-custom.html                         # MathJax 3 and responsive CSS grid styling
│   └── navigation.html                          # Sidebar chapter navigation template
├── _layouts/                                    # Jekyll base layouts
│   └── default.html                             # Theme wrapper based on godalming123/minimal
├── _config.yml                                  # Jekyll configuration & exclusions
├── Gemfile                                      # Ruby gems (github-pages)
└── index.md                                     # Web landing page for /autonomy/_j/
```

---

### C. Formatting Standards

To ensure flawless rendering across both GitHub Pages (HTML) and Pandoc (`pdflatex` PDF generation):

1. **Heading Hierarchy**:
   * Use a single `# Level 1 Heading` for the chapter title.
   * Use `## Level 2 Heading` for primary sections.
   * Use `### Level 3 Heading` for subsections.
2. **Lists**:
   * Always place a blank empty line before any bulleted (`-`) or numbered (`1.`) list. Pandoc will clump lists together if a preceding blank line is missing.
3. **Internal Relative Hyperlinks**:
   * Always use relative Markdown links with the `.md` extension:
     ```markdown
     [Predictive Stanley Controller](../13_Controllers/Predictive_Stanley_Controller.md)
     ```
   * The `jekyll-relative-links` plugin in GitHub Pages automatically translates `.md` to `.html` in web builds, while Pandoc preserves the cross-document anchors.
4. **LaTeX Mathematical Formulations**:
   * Equations are rendered via **MathJax 3** on the web and native LaTeX in Pandoc.
   * **Inline Equations**: Wrap math in single dollar signs `$ ... $` or `\( ... \)`:
     ```markdown
     Rover rotational effort is bounded by $u \in [-1.0, 1.0]$.
     ```
   * **Display Equations**: Wrap standalone block math in double dollar signs `$$ ... $$` or `\[ ... \]`:
     ```markdown
     $$\delta(t) = \theta_e(t) + \arctan\left(\frac{k \cdot e(t)}{v(t)}\right)$$
     ```
   * **Shell Variable Escaping**: Always enclose bash variables and shell commands like `$(nproc)` in backticks (`` `make -j$(nproc)` ``) so MathJax does not interpret them as math delimiters.

---

### D. Local Testing & Compilation

#### 1. Previewing Web Docs Locally (Jekyll)
Install Ruby (version 3.2+) and Bundler, then run inside `docs/autonomy_binder/`:
```bash
cd docs/autonomy_binder
bundle install --path vendor/bundle
bundle exec jekyll serve
```
Open `http://localhost:4000/autonomy/_j/` in your browser.

#### 2. Compiling the Autonomy Binder to a Single Markdown & PDF (Pandoc)
The Autonomy repository includes automated shell scripts that concatenate all chapters in the exact order declared in `00_Table_of_Contents.md`:

```bash
# From repository root:
bash tools/compile_binder_pandoc.sh
```

* **Output Markdown**: `docs/autonomy_binder/Autonomy_Binder_Pandoc.md`
* **Output PDF**: `docs/autonomy_binder/Autonomy_Binder_Pandoc.pdf`

> [!NOTE]
> PDF generation requires `pandoc` and `pdflatex` (e.g., via TeX Live or MiKTeX). If running inside the official Autonomy Dev Container, all TeX packages and Pandoc binaries are pre-installed.

---

## 3. Generalized RoveSoDocs Integration Playbook

This playbook provides the exact 6-step blueprint for connecting **any new or existing MRDT repository** (e.g., `RoveSoSimulator`, `Embedded_Docs`, `Science_Software`, etc.) to the central documentation hub at [docs.themrdt.org](https://docs.themrdt.org/).

### Architecture Overview

```
[Target Repo (docs branch)]
   ├── Jekyll or Doxygen files
   └── Pushed to GitHub
             │
             ▼
[RoveSoDocs (.github/workflows/deploy.yml)]
   ├── 1. build_<repo>_jekyll (outputs to out/<repo>/_j)
   ├── 2. build_<repo>_doxygen (outputs to out/<repo>/_d)
   ├── 3. assemble (merges all out/ artifacts into dist/)
   ├── 4. build-minisearch-index.mjs (indexes dist/)
   └── 5. deploy (publishes to GitHub Pages via Cloudflare)
```

---

### Step 1: Prepare the Source Repository

1. **Create a Dedicated Docs Branch**:
   Create a clean branch in your repository (e.g., `docs/binder` or `github-pages`).
2. **Add a `Gemfile`**:
   In your documentation root (e.g., `docs/binder/Gemfile`):
   ```ruby
   source "https://rubygems.org"
   gem "github-pages", group: :jekyll_plugins
   ```
3. **Configure `_config.yml`**:
   Create `_config.yml` with the following essential configuration:
   ```yaml
   title: Your Subsystem Name
   description: Engineering manual and operations reference.
   url: "https://docs.themrdt.org"
   baseurl: "/<repo>/_j"

   remote_theme: "godalming123/minimal"
   plugins:
     - jekyll-remote-theme
     - jekyll-optional-front-matter

   repository: "MissouriMRDT/<Your_Repo>"

   defaults:
     - scope:
         path: ""
       values:
         layout: "default"

   # CRITICAL: In Jekyll 3.x, specifying exclude overrides defaults!
   # You MUST explicitly exclude vendor/bundle to prevent gem ERB parsing crashes.
   exclude:
     - vendor
     - vendor/*
     - .bundle
     - .sass-cache
     - .jekyll-cache
     - gemfiles
     - Gemfile
     - Gemfile.lock
     - node_modules
     - "*.pdf"
     - "*_Pandoc.md"
   ```
4. **Create Layout Overrides**:
   * `_layouts/default.html`: Wrap the layout with `<header>` (containing title, "Return to RoveSoDocs" link, and navigation) and `<section>` (containing `{{ content }}`).
   * `_includes/navigation.html`: Iterate through `site.navigation` using `relative_url` for internal links and raw URLs for external links.
   * `_includes/head-custom.html`: Embed MathJax 3 and responsive CSS grid styles:
     ```html
     <!-- MathJax 3 SVG vector rendering -->
     <script>
       window.MathJax = {
         tex: {
           inlineMath: [['$', '$'], ['\\(', '\\)']],
           displayMath: [['$$', '$$'], ['\\[', '\\]']],
           processEscapes: true
         },
         svg: { fontCache: 'global' },
         options: {
           skipHtmlTags: ['script', 'noscript', 'style', 'textarea', 'pre', 'code']
         }
       };
     </script>
     <script type="text/javascript" id="MathJax-script" async
       src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js">
     </script>

     <style>
       @media screen and (min-width: 961px) {
         .wrapper {
           width: 92%;
           max-width: 1450px;
           margin: 0 auto;
           display: grid;
           grid-template-columns: 290px minmax(0, 1fr);
           column-gap: 55px;
         }
         header { position: sticky; top: 40px; }
         section { width: 100%; max-width: none; }
       }
       section table { display: block; width: 100%; overflow-x: auto; }
       section pre { max-width: 100%; overflow-x: auto; }
     </style>
     ```

---

### Step 2: Add Build Job to `RoveSoDocs` Workflow

In `MissouriMRDT/RoveSoDocs`, open `.github/workflows/deploy.yml` and add a new build job:

```yaml
  # -------------------------
  # Build <Repo> (Jekyll)
  # -------------------------
  build_<repo>_jekyll:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout <Repo> (Jekyll)
        uses: actions/checkout@v4
        with:
          repository: MissouriMRDT/<Your_Repo>
          ref: docs/binder
          path: src/<Your_Repo>
          fetch-depth: 1
          submodules: false
          token: ${{ secrets.DOCS_BOT_TOKEN || github.token }}

      - name: Setup Ruby (for Jekyll) - no cache
        uses: ruby/setup-ruby@v1
        with:
          ruby-version: "3.2.4"
          bundler-cache: false
          working-directory: src/<Your_Repo>/docs/binder

      - name: Install gems (fresh, no cache)
        working-directory: src/<Your_Repo>/docs/binder
        env:
          BUNDLE_WITHOUT: "development:test"
        run: |
          set -euo pipefail
          rm -rf .bundle vendor/bundle Gemfile.lock
          gem --version
          bundler --version || gem install bundler
          bundle config set --local path 'vendor/bundle'
          bundle config set --local clean 'true'
          bundle config set --local without 'development:test'
          bundle install --jobs 4 --retry 3

      - name: Build site into out/
        working-directory: src/<Your_Repo>/docs/binder
        env:
          JEKYLL_ENV: production
          PAGES_REPO_NWO: MissouriMRDT/<Your_Repo>
          JEKYLL_GITHUB_TOKEN: ${{ secrets.DOCS_BOT_TOKEN || github.token }}
        run: |
          set -euo pipefail
          mkdir -p "$GITHUB_WORKSPACE/out/<repo>/_j"
          bundle exec jekyll build \
            --trace \
            --baseurl "/<repo>/_j" \
            -d "$GITHUB_WORKSPACE/out/<repo>/_j"

      - name: Upload jekyll artifact
        uses: actions/upload-artifact@v4
        with:
          name: <repo>-jekyll-out
          path: out
```

---

### Step 3: Merge Artifacts in the `assemble` Job

In `deploy.yml`, update the `assemble` job:

1. **Add to `needs`**:
   ```yaml
   assemble:
     runs-on: ubuntu-latest
     needs:
       - build_hub
       - build_<repo>_jekyll
       # ... other build jobs
   ```
2. **Merge into `dist/`**:
   ```yaml
       - name: Merge artifacts into dist/
         run: |
           set -euo pipefail
           mkdir -p dist
           rsync -a "parts/hub-out/" dist/
           rsync -a "parts/<repo>-jekyll-out/" dist/
           # ... other artifacts
           touch dist/.nojekyll
   ```

---

### Step 4: Configure SPA Routing & 404 Refresh

VitePress is a Single Page Application (SPA). To prevent client-side routing from hijacking requests to sub-sites and returning a 404:

In `RoveSoDocs/.vitepress/theme/NotFoundContent.vue`:
1. Add the path prefix to `REFRESH_PREFIXES`:
   ```javascript
   const REFRESH_PREFIXES = [
     '/autonomy/_d/',
     '/autonomy/_j/',
     '/<repo>/_j/',
     '/<repo>/_d/',
     // ...
   ];
   ```
2. Add quicklinks to `quickLinks`:
   ```javascript
   { text: '<Repo> Docs (Binder)', href: '/<repo>/_j/' },
   ```

---

### Step 5: Update the Unified Search Index

In `RoveSoDocs/tools/build-minisearch-index.mjs`:

Update `sectionFromUrl(url)` with regular expressions for your docset:

```javascript
function sectionFromUrl(url) {
  if (url.startsWith('/<repo>/_j/')) return '<Repo> Documentation (Binder)';
  if (url.startsWith('/<repo>/_d/')) return '<Repo> Documentation (Doxygen)';
  // ...
}
```

This ensures search results in the top navbar (`Ctrl+K`) categorize entries under clean, human-readable subsystem headers.

---

### Step 6: Add Homepage Feature Cards

In `RoveSoDocs/index.md`:

Add a feature card under `features:`:

```yaml
  - icon: "📖"
    title: "<Repo> Documentation (Binder)"
    details: "High-level subsystem engineering manual, architecture guide, and operational instructions."
    link: /<repo>/_j/
    linkText: "Open <Repo> Docs"
```

> [!TIP]
> Maintain clean grid balance on the homepage. MRDT's homepage layout is optimized for a **3-column grid** on desktop screens (`@media (min-width: 1400px)`). Aim for multiple-of-3 card counts (6, 9, or 12 cards) for a visually symmetrical presentation.

---

## 4. Common Pitfalls & Troubleshooting Checklist

| Issue / Symptom | Root Cause | Resolution |
| :--- | :--- | :--- |
| **`Invalid date '<%= Time.now...' in welcome-to-jekyll.markdown.erb`** | Jekyll 3.x `exclude:` list completely overrides default exclusions. When `vendor/bundle` was created by bundler, Jekyll tried to compile gem templates. | Add `vendor`, `vendor/*`, and `.bundle` explicitly to `exclude:` in `_config.yml`. |
| **Narrow 500px Content Column on Desktop** | The default minimal theme hardcodes `section { width: 500px; float: right; }`. | Implement the responsive CSS Grid override in `_includes/head-custom.html`. |
| **Raw LaTeX (`$...$`) Rendering as Plain Text** | Jekyll Markdown engine does not convert math unless a client-side renderer is loaded. | Load MathJax 3 `tex-svg.js` in `_includes/head-custom.html`. |
| **Missing Bullet Lists in Compiled Pandoc PDF** | Pandoc requires a blank line preceding any bullet or numbered list. | Ensure an empty line separates all paragraphs from bullet points. |
| **Old 5-Card Layout Showing After Deploy** | VitePress client-side PWA caching. | Perform a hard refresh in the browser (<kbd>Ctrl</kbd> + <kbd>F5</kbd> or <kbd>Cmd</kbd> + <kbd>Shift</kbd> + <kbd>R</kbd>). |
