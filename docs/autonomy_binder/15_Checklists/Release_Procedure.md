# Autonomy Software Release Procedure & Changelog Guide

This guide defines the end-to-end standard operating procedure for preparing, auditing, drafting, and publishing official GitHub releases for the MRDT Autonomy Software repository (`MissouriMRDT/Autonomy_Software`).

---

## 1. Overview & Release Philosophy

In the Mars Rover Design Team, Autonomy Software releases mark major engineering milestones:
- **Competition Releases** (e.g., `v24.5.0`, `v26.0.0`): Field-validated, competition-proven software used during official University Rover Challenge (URC) runs.
- **Sprint & Pre-Competition Releases** (e.g., `v25.0.0`, `v25.1.0`): Significant platform upgrades (such as JetPack, CUDA, or major architecture rewrites) ready for team integration and field trials.
- **Hotfix Releases** (e.g., `v24.3.1`): Targeted fixes deployed for critical bugs identified during testing sessions.

### Versioning Convention
Releases follow semantic versioning adapted to the rover competition cycle:
`v<Year>.<Minor>.<Patch>`
- **Year**: The competition year the code targets (e.g., `v26` for 2026).
- **Minor**: Milestone index or pre-competition sprint index (e.g., `0` for Pre-Comp #1, `1` for Pre-Comp #2, `5` or `0` for major competition release).
- **Patch**: Urgent bug fixes or minor patches.

---

## 2. Prerequisites & Tooling

Before creating a release, ensure you have:
1. **GitHub CLI (`gh`)**:
   - Installed and authenticated with repository write access:
     ```bash
     gh auth status
     ```
   - Must show logged in with `repo` scope.
2. **Git Client**:
   - Clean working copy on the default branch (`development`).
3. **Python 3 & `jq`**:
   - Used for parsing PR JSON metadata and generating the Full Development Log.

---

## 3. Step-by-Step Release Workflow

### Step 1: Branch Verification & Sync
All releases are cut from the default `development` branch. Do not cut releases from unmerged feature branches.

```bash
# 1. Switch to development branch
git checkout development

# 2. Fetch and pull latest remote changes
git fetch origin
git pull origin development

# 3. Verify working directory is clean
git status
```

Confirm the output reports:
`Your branch is up to date with 'origin/development'` and `nothing to commit, working tree clean`.

---

### Step 2: Determine Previous Release Tag
Locate the most recent published release tag to establish the diff baseline:

```bash
gh release list --limit 5
```

Alternatively, query git tags:
```bash
git tag -l -n1 --sort=-creatordate
```

Example output:
`v25.1.0` (published previously). All commits and PRs between `v25.1.0` and `HEAD` will comprise the new release.

---

### Step 3: Extract Merged Pull Requests
Use the GitHub CLI to pull structured JSON data for all merged pull requests since the last release tag:

```bash
gh pr list --state merged --base development --limit 100 --json number,title,author,mergedAt,url,labels,body
```

You can use the GitHub Releases API to automatically preview the changelog and discover first-time contributors:

```bash
gh api repos/MissouriMRDT/Autonomy_Software/releases/generate-notes \
  -f tag_name=<NEW_TAG> \
  -f previous_tag_name=<PREV_TAG> \
  -f target_commitish=development
```

Example:
```bash
gh api repos/MissouriMRDT/Autonomy_Software/releases/generate-notes \
  -f tag_name=v26.0.0 \
  -f previous_tag_name=v25.1.0 \
  -f target_commitish=development
```

---

### Step 4: Audit Commit History & Identify New Contributors
Check git log to verify that no direct merge commits or branch contributions are omitted:

```bash
# View all commit titles since the previous tag
git log <PREV_TAG>..HEAD --oneline

# View all authors who contributed commits in this cycle
git log <PREV_TAG>..HEAD --format="%an <%ae>" | sort -u
```

Identify first-time contributors by checking authors who had never merged commits before `<PREV_TAG>`:
```bash
python3 -c "
import subprocess
def get_authors(rev):
    return set(subprocess.check_output(['git', 'log', rev, '--format=%aN <%aE>'], text=True).splitlines())
brand_new = get_authors('<PREV_TAG>..HEAD') - get_authors('<PREV_TAG>')
for a in sorted(brand_new):
    print('-', a)
"
```

---

### Step 5: Draft the Release Notes (`RELEASE_NOTES.md`)

Create a local `RELEASE_NOTES.md` file in the workspace root. Structure the document according to the following mandatory sections:

#### Required Sections & Format

```markdown
### **Autonomy Software <VERSION> – <TITLE>: <SUBTITLE>**

<Executive summary paragraph describing the release context, competition performance, testing score (e.g., 86/100 at URC), and core themes.>

> **Breaking Changes**: <High-level warning outlining major backward-incompatible changes, removed states, or new submodule/toolchain requirements.>

---

### Breaking Changes

#### <Subsystem 1 Name>
- <Bullet points detailing breaking API changes, removed classes, or new database formats.>

#### <Subsystem 2 Name>
- <Details on deprecated workflows, build flag adjustments, or configuration file removals.>

---

### Key Updates

#### <Subsystem Area: e.g., Terrain-Aware Path Planning (GeoPlanner & LiDARHandler)>
- <Technical details of implementation, algorithms (KD-trees, R-Trees), performance improvements.>

#### <Subsystem Area: e.g., Predictive Stanley Controller & Tank-Drive Kinematics>
- <Details on controller redesigns, physics models (UnicycleModel), and deceleration logic.>

#### <Subsystem Area: e.g., Multi-Class Object Detection>
- <Models trained, target classes (Mallet, Bottle, Rock Pick), and dataset collection.>

#### <Subsystem Area: e.g., Approaching & Verifying Marker/Object States>
- <State machine enhancements, bounding box verification image capture, and HTTP REST serving.>

#### <Subsystem Area: e.g., Autonomy Live 3D Visualizer Tool>
- <Interactive browser dashboard features, telemetry feeds, and spatial deduplication.>

#### <Subsystem Area: e.g., Search Patterns & Stuck-State Path Splicing>
- <Search trajectories (Snake, double-spiral), stuck recovery detour splicing, and path persistence.>

#### <Subsystem Area: e.g., Simulation Parity & WebRTC Streaming>
- <Simulation stability, WebRTC connection improvements, and asynchronous camera APIs.>

---

### Validated On

#### Hardware & Software Stack
- **Compute**: <e.g., NVIDIA Jetson AGX Orin Developer Kit>
- **Operating System**: <e.g., JetPack 6.2 (L4T 36.4.0)>
- **CUDA Toolchain**: <e.g., CUDA 12.6.3, TensorRT, LibTorch>
- **Perception Hardware**: <e.g., Dual Stereolabs ZED 2i Cameras (Front & Rear)>
- **Container Environment**: <e.g., Ubuntu 22.04 LTS (Jammy) Docker>

#### Field Validation & Testing Grounds
- **URC Competition**: <Mission scores, location, conditions (e.g., MDRS Hanksville, UT - 86/100)>
- **Field Testing**: <Trials in Tucumcari, Rolla, or other testing sites>
- **Simulation**: <Validation in RoveSoSimulator (e.g., 100/100 score on path splicing)>

---

### What’s Next
- <Bullet point 1: Next milestone feature>
- <Bullet point 2: Multi-camera fusion / sensor upgrade>
- <Bullet point 3: Autonomous state expansions>

---

**Huge congratulations and thanks to the entire Autonomy subteam and MRDT members who worked tirelessly to bring this software to competition!**

---

### Full Development Log
* <PR 1 Title> by @<Author> in https://github.com/MissouriMRDT/Autonomy_Software/pull/<PR_NUMBER>
* <PR 2 Title> by @<Author> in https://github.com/MissouriMRDT/Autonomy_Software/pull/<PR_NUMBER>
...

## New Contributors
* @<Contributor1> made their first contribution in https://github.com/MissouriMRDT/Autonomy_Software/pull/<PR_NUMBER>
* @<Contributor2> made their first contribution in https://github.com/MissouriMRDT/Autonomy_Software/pull/<PR_NUMBER>

Special thanks to all team members contributing commits, models, datasets, and testing hours to this release cycle: <List of all active contributors>.

**Full Changelog**: https://github.com/MissouriMRDT/Autonomy_Software/compare/<PREV_TAG>...<NEW_TAG>
```

---

### Step 6: Create Draft Release on GitHub
Always create the release as a **Draft** first. This permits the team and leads to verify the formatting, links, and text directly on GitHub without triggering webhook deployments or alerting subscribers prematurely.

Run the `gh release create` command:

```bash
gh release create <NEW_TAG> \
  --title "Autonomy Software <NEW_TAG> - <TITLE>" \
  --notes-file RELEASE_NOTES.md \
  --target development \
  --draft
```

Example:
```bash
gh release create v26.0.0 \
  --title "URC 2026 - Competition Release" \
  --notes-file RELEASE_NOTES.md \
  --target development \
  --draft
```

The command will output the draft URL:
`https://github.com/MissouriMRDT/Autonomy_Software/releases/tag/untagged-...`

---

### Step 7: Review and Publish

1. **Verify Draft in Web Browser**:
   Open the draft release link returned by `gh`. Ensure that:
   - All PR links navigate to the correct PRs.
   - Code blocks and backticks render cleanly.
   - There are zero stray characters or emojis.
   - The target branch is set to `development`.

2. **Publish the Release**:
   - **Option A (Web UI)**: Click the **Publish release** button on the GitHub release edit page.
   - **Option B (GitHub CLI)**:
     ```bash
     gh release edit <DRAFT_ID_OR_TAG> --draft=false --tag <NEW_TAG>
     ```

3. **Synchronize Local Tags**:
   After publishing, pull down the new git tag to your local environment:
   ```bash
   git fetch --tags origin
   ```

---

## 4. Common Pitfalls & Best Practices

| Pitfall | Impact | Prevention |
| :--- | :--- | :--- |
| **Direct shell script backtick evaluation** | Backticks in release notes get interpreted by bash as command substitutions (e.g., `` `UnicycleModel` `` tries to run command `UnicycleModel`). | When using bash scripts or python to generate markdown files, use quoted heredocs (`cat <<'EOF'`) or read directly from files. |
| **Omitting submodule updates** | Submodules like `data/LiDAR` or `submodules/RoveComm_CPP` may be on different commits than what the release expects. | Document submodule init commands (`git submodule update --init --recursive`) under Breaking Changes. |
| **Cutting from feature branches** | Release tags will not point to master/development HEAD, causing merge conflicts or dangling commits. | Always ensure `git status` confirms you are on `development` up to date with `origin/development`. |
| **Publishing before review** | Missing PRs or formatting bugs cannot be cleanly retracted once tags are pushed to remote clones. | Always use `--draft` flag on `gh release create` and preview before publishing. |
| **Emojis in release notes** | Clutters terminal release feeds and breaks automated changelog parsers. | Keep release text strictly alphanumeric and standard markdown typography. |

