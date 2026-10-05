# RoveComm Comprehensive Documentation & RoveSoDocs Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Author an authoritative, multi-chapter engineering guide for the RoveComm protocol and its ecosystem on a dedicated documentation branch in `RoveComm_Base`, and integrate it seamlessly into `docs.themrdt.org` via `RoveSoDocs`.

**Architecture:** A comprehensive Jekyll- and Pandoc-compatible technical manual housed under `docs/` in `MissouriMRDT/RoveComm_Base` on branch `docs/rovecomm`. It covers protocol wire specifications, the `manifest.json` schema and automated CI synchronization pipelines, multi-language implementations (C++, C#, Python, Arduino), the diagnostic tester software (`RoveComm_Tester_Software`), and full integration into the centralized documentation hub `docs.themrdt.org`.

**Tech Stack:** Markdown (GFM), Jekyll 3.10 / GitHub Pages, MathJax 3 (SVG engine), Pandoc, LaTeX (pdflatex), Python 3.12, C++20, C# (.NET 8), Bash, GitHub Actions CI, VitePress.

---

## Global Constraints
- Target repository for docs content: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base` on dedicated branch `docs/rovecomm`.
- Documentation hub repository: `C:\Users\ltkli\Documents\GitHub\RoveSoDocs` on branch `development`.
- Subpath convention: `/rovecomm/_j/` for the Jekyll guide, coexisting with `/rovecomm/_cpp/` for C++ Doxygen.
- Strict ASCII tree formatting in code blocks (no unicode box-drawing characters like `├`, `│`, `└`) to ensure 100% clean Pandoc `pdflatex` compilation.
- Complete, turnkey documentation with zero placeholders, `TODO` markers, or truncated explanations.
- MathJax 3 SVG rendering compatibility with backtick shielding for shell variables.

---

### Task 1: Scaffolding and Jekyll Environment Setup in `RoveComm_Base`

**Files:**
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\_config.yml`
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\Gemfile`
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\_includes\head-custom.html`
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\_includes\navigation.html`
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\_layouts\default.html`
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\tools\compile_rovecomm_pandoc.sh`

**Interfaces:**
- Produces: Complete static-site and PDF build infrastructure for `docs/` in `RoveComm_Base`.
- Consumes: `godalming123/minimal` remote theme and MathJax 3 CDN scripts.

- [x] **Step 1: Create `docs/_config.yml`**

Write `_config.yml` with:
```yaml
title: RoveComm Protocol Guide
description: Central engineering manual and specification for the Missouri S&T Mars Rover Design Team RoveComm protocol.

url: "https://docs.themrdt.org"
baseurl: "/rovecomm/_j"

remote_theme: "godalming123/minimal"
plugins:
  - jekyll-remote-theme
  - jekyll-optional-front-matter

repository: "MissouriMRDT/RoveComm_Base"

navigation:
  - link: "/"
    name: "Home"
  - link: "/01_Protocol_Specification.html"
    name: "Protocol Spec"
  - link: "/02_Transport_Layers.html"
    name: "Transport Layers"
  - link: "/03_Manifest_Ecosystem.html"
    name: "Manifest Ecosystem"
  - link: "/04_CPP_Implementation.html"
    name: "C++ Guide"
  - link: "/05_CSharp_Implementation.html"
    name: "C# Guide"
  - link: "/06_Python_Implementation.html"
    name: "Python Guide"
  - link: "/07_Embedded_Implementation.html"
    name: "Embedded Guide"
  - link: "/08_Tester_Software.html"
    name: "Tester Software"
  - link: "/09_Docs_Integration.html"
    name: "Docs Integration"
  - link: "https://docs.themrdt.org/rovecomm/_cpp/"
    name: "C++ Doxygen API"
  - link: "https://docs.themrdt.org"
    name: "Return to RoveSoDocs"

markdown: kramdown
kramdown:
  input: GFM
  hard_wrap: false

defaults:
  - scope:
      path: ""
    values:
      layout: "default"

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
  - "RoveComm_Guide_Pandoc.md"
```

- [x] **Step 2: Create `docs/Gemfile`**

```ruby
source "https://rubygems.org"

gem "github-pages", group: :jekyll_plugins
gem "jekyll-remote-theme"
gem "jekyll-optional-front-matter"
gem "webrick"
```

- [x] **Step 3: Create `docs/_includes/head-custom.html` and `_layouts/default.html`**

Set up MathJax 3 SVG engine script and custom CSS to provide clean mobile/desktop typography and wide table scaling.

- [x] **Step 4: Create `tools/compile_rovecomm_pandoc.sh`**

Script that reads `docs/00_Table_of_Contents.md`, concatenates all chapters into `docs/RoveComm_Guide_Pandoc.md`, and runs `pandoc` with `--pdf-engine=pdflatex` to output `docs/RoveComm_Manual.pdf`.

- [x] **Step 5: Verify build script line endings (LF) and commit scaffolding**

```bash
git add docs/ tools/
git commit -m "Initialize RoveComm documentation scaffolding and Pandoc compiler"
```

---

### Task 2: Author Core Protocol Architecture & Specification (Chapters 00, 01, 02)

**Files:**
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\00_Table_of_Contents.md`
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\index.md`
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\01_Protocol_Specification.md`
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\02_Transport_Layers.md`

**Interfaces:**
- Produces: Foundational protocol specification covering wire headers, data types, endianness, system packets, and transport layer tradeoffs.
- Consumes: Protocol definitions from `RoveComm_CPP/src/RoveComm/RoveCommConsts.h` and `manifest.json`.

- [x] **Step 1: Write `docs/01_Protocol_Specification.md`**
  - Section 1: Overview & History of RoveComm v3.
  - Section 2: Wire Format & Packet Framing:
    - 6-byte header: `[Version (1B)] [DataID (2B)] [DataCount (2B)] [DataType (1B)]`
    - ASCII bit diagram of header fields.
    - Network byte order rules (Big Endian) and conversion primitives (`htons`, `htonl`, `htonll`, `ntohs`, `ntohl`, `ntohll`).
  - Section 3: Data Types Reference Table (IDs 0-8: `INT8_T`, `UINT8_T`, `INT16_T`, `UINT16_T`, `INT32_T`, `UINT32_T`, `FLOAT_T`, `DOUBLE_T`, `CHAR`, with byte sizes and C++/C#/Python equivalents).
  - Section 4: System Reserved Packets (IDs 1-6):
    - `PING` (1) & `PING_REPLY` (2)
    - `SUBSCRIBE` (3) & `UNSUBSCRIBE` (4)
    - `INVALID_VERSION` (5)
    - `NO_DATA` (6)
  - Section 5: Network IP Addressing Architecture:
    - Port conventions: UDP `11000`, TCP `12000`.
    - Subnets: Rover Subnet (`192.168.2.x`), Aux Subnet (`192.168.3.x`), Drone Subnet (`192.168.100.x`), Ubiquiti Rocket links (`10.0.0.x`).

- [x] **Step 2: Write `docs/02_Transport_Layers.md`**
  - Section 1: UDP vs TCP Architectural Decision Matrix.
  - Section 2: UDP Transport:
    - Connectionless datagram mechanics, performance profile, and low-latency considerations.
    - UDP Subscription Model: How clients register (`SUBSCRIBE`) to receive continuous telemetry, subscriber capacity limits (`ROVECOMM_ETHERNET_UDP_MAX_SUBSCRIBERS`), and timeout/keep-alive handling.
    - UDP Packet Delivery Guarantees and mitigation of packet loss.
  - Section 3: TCP Transport:
    - Connection-oriented streaming mechanics.
    - Packet boundary delimitation and stream buffering over TCP.
    - Reconnection strategies, socket failure detection, and reliable command delivery (E-Stop, Bus status).

- [x] **Step 3: Commit Chapters 01 and 02**

```bash
git add docs/01_Protocol_Specification.md docs/02_Transport_Layers.md
git commit -m "Add Chapter 01 (Protocol Spec) and Chapter 02 (Transport Layers)"
```

---

### Task 3: Author Manifest Ecosystem & Automated CI Pipeline (Chapter 03)

**Files:**
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\03_Manifest_Ecosystem.md`

**Interfaces:**
- Produces: Detailed documentation of the central manifest file, CI triggers, GitHub Actions webhooks, and automatic language-binding generators.
- Consumes: `.github/workflows/readme-generate.yml`, `.github/scripts/readme-generate.py`, `manifest.json`.

- [x] **Step 1: Write `docs/03_Manifest_Ecosystem.md`**
  - Section 1: Role of `manifest.json` as Single Source of Truth for MRDT.
  - Section 2: Manifest JSON Schema Anatomy:
    - Root metadata: `ManifestSpecVersion`, `DataTypes`, `SystemPackets`, `ethernetUDPPort`, `ethernetTCPPort`.
    - Board definitions: `Core`, `PMS`, `Nav`, `Multimedia`, `Arm`, `Science`, `Raman`, `DroneGPS`, `RoveSoSimulator`.
    - Command, Telemetry, and Error blocks: `dataId`, `dataType`, `dataCount`, `comments`.
    - Enumerations (`Motors`, `DisplayState`, `VESCFaultCode`, etc.).
  - Section 3: Automated Human-Readable Documentation Generation:
    - Workflow: `.github/workflows/readme-generate.yml`
    - Script: `.github/scripts/readme-generate.py`
  - Section 4: Webhook Dispatch Pipeline:
    - Cross-repository synchronization via `repository_dispatch` (`event_type: rovecomm_sync`).
    - Flowchart of event distribution from `RoveComm_Base` to `RoveComm_CPP`, `RoveComm_CSharp`, and `RoveComm_Python`.
  - Section 5: Downstream Code Binding Generation:
    - C++: Submodule update in `data/RoveComm` and parser script `tools/RoveComm/parser.py` generating `src/RoveComm/RoveCommManifest.h`.
    - C#: Scripts `tools/generate_boards.py` and `tools/parser.py` generating `RoveCommBoards.cs` and `RoveCommManifest.cs`.

- [x] **Step 2: Commit Chapter 03**

```bash
git add docs/03_Manifest_Ecosystem.md
git commit -m "Add Chapter 03: Manifest Ecosystem and Automated CI Synchronization"
```

---

### Task 4: Author Multi-Language Implementation Guides (Chapters 04, 05, 06, 07)

**Files:**
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\04_CPP_Implementation.md`
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\05_CSharp_Implementation.md`
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\06_Python_Implementation.md`
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\07_Embedded_Implementation.md`

**Interfaces:**
- Produces: Concrete, production-tested implementation guides with copy-paste code examples across C++, C#, Python, and Arduino.
- Consumes: Code patterns from `RoveComm_CPP`, `RoveComm_CSharp`, `RoveComm_Tester_Software`, and `RoveComm_Arduino`.

- [x] **Step 1: Write `docs/04_CPP_Implementation.md`**
  - Architecture in `Autonomy_Software` and `RoveSoSimulator`.
  - Threading architecture: `AutonomyThread`, worker pools, Windows IOCP vs Linux POSIX sockets.
  - Core classes: `RoveCommPacket<T>`, `RoveCommUDP`, `RoveCommTCP`.
  - Comprehensive code examples:
    - Initializing UDP socket and registering callbacks for specific `dataId`.
    - Sending vector telemetry and commands.
    - Synchronous vs asynchronous callback handling.

- [x] **Step 2: Write `docs/05_CSharp_Implementation.md`**
  - Architecture in BaseStation .NET Blazor.
  - Dependency injection and service lifetime (`RoveCommService`).
  - Asynchronous async/await socket models and `TaskCompletionSource`.
  - Comprehensive code examples:
    - Setting up `RoveCommService` in `Program.cs`.
    - Subscribing to telemetry events in Razor components.
    - Sending commands from UI buttons and gamepads.

- [x] **Step 3: Write `docs/06_Python_Implementation.md`**
  - Architecture of `rovecomm.py`.
  - Python socket implementation using `select.select()` and background daemon threads.
  - Struct packing with `struct.pack(">BHHB...", ...)` and type map dictionaries.
  - Standalone script examples:
    - Sending telemetry and parsing incoming messages in automated testing scripts.
    - Differential GPS integration on NavBoard.

- [x] **Step 4: Write `docs/07_Embedded_Implementation.md`**
  - Architecture of `RoveComm_Arduino` for microcontrollers (Teensy 4.1, STM32, SAM).
  - Microcontroller constraints: static memory buffers, zero dynamic allocation, polling loop (`rovecomm.read()`).
  - Hardware PHY integration (NativeEthernet, W5500).
  - Firmware dispatch loop example: reading packet, switching on `dataId`, commanding motor controllers.

- [x] **Step 5: Commit Chapters 04 through 07**

```bash
git add docs/04_CPP_Implementation.md docs/05_CSharp_Implementation.md docs/06_Python_Implementation.md docs/07_Embedded_Implementation.md
git commit -m "Add Chapters 04-07: C++, C#, Python, and Embedded implementation guides"
```

---

### Task 5: Author Diagnostic Tooling & Tester Software Guide (Chapter 08)

**Files:**
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\08_Tester_Software.md`

**Interfaces:**
- Produces: Complete operations guide for the RoveComm Tester GUI application.
- Consumes: Codebase at `C:\Users\ltkli\Documents\GitHub\RoveComm_Tester_Software`.

- [x] **Step 1: Write `docs/08_Tester_Software.md`**
  - Section 1: Overview and Purpose of `RoveComm_Tester_Software`.
  - Section 2: Installation and Environment Setup (Python dependencies, PyQt5).
  - Section 3: GUI Modules Deep Dive:
    - `QtSender.py`: Packet crafting, selecting board and command from manifest, sending UDP/TCP.
    - `QtReciever.py`: Real-time packet sniffer, telemetry monitor, filtering by DataID and source IP.
  - Section 4: Hardware-in-the-Loop Testing with Gamepads:
    - `XboxController.py` axis and button mappings.
    - Teleoperation drive testing via RoveComm without BaseStation.
  - Section 5: Configuration Presets (`1-Configs/Autonomy.json`, `DriveConfig.json`).
  - Section 6: Standard Testing Procedures:
    - Testing board responsiveness.
    - Simulating motor telemetry for Autonomy debugging.
    - Stress-testing network packet throughput.

- [x] **Step 2: Commit Chapter 08**

```bash
git add docs/08_Tester_Software.md
git commit -m "Add Chapter 08: RoveComm Tester Software diagnostic guide"
```

---

### Task 6: Author RoveSoDocs Integration Playbook, Table of Contents, and Web Index (Chapters 09, 10, TOC, Index)

**Files:**
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\09_Docs_Integration.md`
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\10_Pandoc_PDF_Build.md`
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\00_Table_of_Contents.md`
- Create: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\docs\index.md`

**Interfaces:**
- Produces: Master navigation index, landing page, and documentation integration playbook following Chapter 16 of the Autonomy Binder.
- Consumes: `RoveSoDocs` deployment specification.

- [x] **Step 1: Write `docs/09_Docs_Integration.md`**
  - Complete 6-step integration playbook tailored for `RoveComm_Base` into `RoveSoDocs`:
    1. Duplicating Jekyll workflow section in `RoveSoDocs/.github/workflows/deploy.yml` (`build_rovecomm_jekyll`).
    2. Setting destination path to `out/rovecomm/_j/` (preserving `out/rovecomm/_cpp/` for Doxygen).
    3. Adding `/rovecomm/_j/` to `.vitepress/theme/NotFoundContent.vue`.
    4. Adding `/rovecomm/_j/` to `tools/build-minisearch-index.mjs`.
    5. Updating `RoveSoDocs/index.md` feature cards for both RoveComm Guide and C++ Doxygen.

- [x] **Step 2: Write `docs/10_Pandoc_PDF_Build.md`**
  - Instructions for compiling the single PDF using `tools/compile_rovecomm_pandoc.sh`.
  - Dependencies required (`pandoc`, `texlive-latex-base`, `texlive-fonts-recommended`).

- [x] **Step 3: Write `docs/00_Table_of_Contents.md` and `docs/index.md`**
  - Full master TOC linking Chapters 01 through 10.
  - Jekyll landing page with card grid linking to each section.

- [x] **Step 4: Commit Chapters 09, 10, TOC, and landing page**

```bash
git add docs/09_Docs_Integration.md docs/10_Pandoc_PDF_Build.md docs/00_Table_of_Contents.md docs/index.md
git commit -m "Add Chapters 09-10, master Table of Contents, and landing page"
```

---

### Task 7: Update `MissouriMRDT/RoveSoDocs` Configuration

**Files:**
- Modify: `C:\Users\ltkli\Documents\GitHub\RoveSoDocs\.github\workflows\deploy.yml`
- Modify: `C:\Users\ltkli\Documents\GitHub\RoveSoDocs\.vitepress\theme\NotFoundContent.vue`
- Modify: `C:\Users\ltkli\Documents\GitHub\RoveSoDocs\tools\build-minisearch-index.mjs`
- Modify: `C:\Users\ltkli\Documents\GitHub\RoveSoDocs\index.md`

**Interfaces:**
- Produces: Live deployment configuration in `RoveSoDocs` for pulling and serving the RoveComm Guide at `docs.themrdt.org/rovecomm/_j/`.

- [x] **Step 1: Add `build_rovecomm_jekyll` to `deploy.yml` in `RoveSoDocs`**
  - Clone `MissouriMRDT/RoveComm_Base` on branch `docs/rovecomm`.
  - Build Jekyll site into `out/rovecomm/_j/`.
  - Include artifact in `assemble` and `deploy` jobs.

- [x] **Step 2: Update `.vitepress/theme/NotFoundContent.vue`**
  - Add `/rovecomm/_j/` to `refreshPrefixes`.
  - Add quicklink for RoveComm Guide.

- [x] **Step 3: Update `tools/build-minisearch-index.mjs`**
  - Add `if (url.startsWith("/rovecomm/_j/")) return "RoveComm Protocol Guide";` to `sectionFromUrl()`.

- [x] **Step 4: Update `index.md` in `RoveSoDocs`**
  - Add feature card for "RoveComm Protocol Guide (Binder)" linking to `/rovecomm/_j/`.
  - Maintain balanced card layout on the homepage.

- [x] **Step 5: Verify and commit changes in `RoveSoDocs`**

```bash
git add .github/workflows/deploy.yml .vitepress/theme/NotFoundContent.vue tools/build-minisearch-index.mjs index.md
git commit -m "Integrate RoveComm Protocol Guide into RoveSoDocs hub"
```

---

### Task 8: Verification & Compilation

**Files:**
- Test: `C:\Users\ltkli\Documents\GitHub\RoveComm_Base\tools\compile_rovecomm_pandoc.sh`

- [x] **Step 1: Run Pandoc compilation script in `RoveComm_Base`**

```bash
cd C:\Users\ltkli\Documents\GitHub\RoveComm_Base
bash tools/compile_rovecomm_pandoc.sh
```
Expected: Clean compilation of `docs/RoveComm_Manual.pdf` with no LaTeX errors.

- [x] **Step 2: Verify git status and branches across both repos**

Run `git status` in `RoveComm_Base` and `RoveSoDocs` to ensure all branches are clean and ready.
