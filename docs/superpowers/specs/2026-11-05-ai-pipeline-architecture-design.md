# Architecture Specification: AI Training Pipeline & Perception Model Ecosystem

> **Document Status**: Approved Technical Design / CDR Engineering Specification  
> **Target Audience**: Missouri MRDT Autonomy Developers, Perception Subteam, Software CDR Panel  
> **Authors / Primary Leads**: Oscar & Saurav (AI Pipeline Leads), Autonomy Lead  
> **Collaborators**: Kara & Thomas (Pose Model Leads), Eli Byrd (HPC / Systems Lead)  
> **Review Date**: Saturday, November 14, 2026 (Software CDR)  
> **Companion Repositories**: `MissouriMRDT/Autonomy_Vision_Pipeline` (GitLab), [`Autonomy_Software`](file:///c:/Users/ltkli/Documents/GitHub/Autonomy_Software)

---

## 1. Executive Summary & Purpose

Missouri S&T Mars Rover Design Team (MRDT) perception models operate under harsh, high-consequence conditions at the University Rover Challenge (Hanksville, Utah). Autonomous navigation and astronaut assistance require real-time visual identification of target competition tools (Nalgene water bottles, orange dead blow mallets, rock picks), navigation markers (ArUco tags), and human postures (astronaut body keypoints).

Historically, the team's machine learning workflows suffered from severe organizational friction:
1. **Unversioned Tribal Workflows**: Dataset acquisition scripts, preprocessing routines, and hyperparameter configurations lived in unmanaged directories across graduating seniors' personal laptops. When students graduated, their training setups vanished.
2. **Roboflow Credit Depletion & Pipeline Fragmentation**: The team historically relied on cloud training within Roboflow until compute credits were exhausted. Training was forced offline onto the S&T Mill HPC cluster via ad hoc shell scripts without formal automated synchronization.
3. **Dual-Pass Jetson Bottleneck**: The physical rover (NVIDIA Jetson AGX Orin) ran two isolated neural networks across identical $1920\times 1080$ ZED camera frames: one for tool detection (`ObjectDetector`) and one for ArUco tag detection (`TagDetector`). This duplicated feature extraction, consumed excessive GPU memory, and halved inference throughput.

This specification defines the architecture for the **MRDT AI Training Pipeline** developed and maintained by **Oscar & Saurav**. The pipeline establishes a unified, version-controlled repository (`Autonomy_Vision_Pipeline`) providing containerized training environments, automated Roboflow API dataset ingestion, multi-model concurrent training on the S&T Mill HPC, automated multi-metric evaluation, and automated compilation to TensorRT FP16 engines for the Jetson Orin.

```
+---------------------------------------------------------------------------------------------------+
| TRACK A: OFFLINE TRAINING & COMPANION PIPELINE (Oscar & Saurav: Autonomy_Vision_Pipeline)          |
|                                                                                                   |
|  [Roboflow API v2]              [Past URC Video Logs]            [Mill HPC Slurm Batch Array]     |
|          |                               |                                    |                   |
|          v                               v                                    v                   |
|  1. download_dataset.py ----> 2. custom_augment.py --------> 3. Concurrent Training (train.py)    |
|     (API key sync,               (HSV shifts, desert                - YOLOv11n-seg (Tools/Tags)   |
|      versioned zip)               null images, balance)             - YOLOv11n-pose (Astronaut)   |
|                                                                               |                   |
|                                                                               v                   |
|                                                                   4. Model Selection              |
|                                                                      (compare_models.py)          |
|                                                                      mAP50, Recall, Latency       |
|                                                                               |                   |
|                                                                               v                   |
|                                                                   5. Automated Export             |
|                                                                      (export_model.py)            |
|                                                                      .pt -> .onnx -> .engine      |
+---------------------------------------------------------------------------------------------------+
                                                |
                                                | Deployment Artifacts (data/models/ + info.txt)
                                                v
+---------------------------------------------------------------------------------------------------+
| TRACK B: RUNTIME JETSON EXECUTION (Autonomy_Software: Jetson AGX Orin)                            |
|                                                                                                   |
|  [ZED 2i Stereo Camera @ 1080p]                                                                   |
|                 |                                                                                 |
|                 v                                                                                 |
|  [Unified TensorRT Inference Engine] (Single Forward Pass @ 85 FPS)                               |
|                 |                                                                                 |
|         +-------+--------------------+---------------------+                                      |
|         |                            |                     |                                      |
|         v                            v                     v                                      |
|  [Class 0: Bottle]           [Class 1: Mallet]     [Class 2: Pick]       [Class 3: Tag]           |
|  [Class 4: High-V Pose]      [Class 5: Arm-Up]     [Class 6: Neutral]                             |
+---------------------------------------------------------------------------------------------------+
```

---

## 2. Direct Answers to Software CDR Slide 14 Questions

### Q1: Where is this repository being stored? How can members access it?

#### Repository Location & Version Control
The AI pipeline lives in a dedicated, standalone companion repository hosted on the Missouri MRDT GitLab instance:
* **Repository URL**: `https://git.mrdt.org/autonomy/Autonomy_Vision_Pipeline` (mirrored to GitHub `MissouriMRDT/Autonomy_Vision_Pipeline` for external visibility).
* **Branching Strategy**: Protected `main` branch for stable release tags. Developers create feature branches (e.g. `feat/yolov11-seg-training`, `feat/pose-dataset-pipeline`) and submit Merge Requests (MRs) requiring perception lead approval.
* **Large File Strategy**: Git LFS tracks model configuration weight snapshots and small test fixture clips. Massive raw training datasets and intermediate checkpoint runs are stored on S&T shared scratch storage (`/scratch/mrdt/`) and synced via the Roboflow API, keeping the Git repository lightweight (<50 MB).

#### Member Access & Environment Reproducibility
Team members access the repository through two standard mechanisms:

1. **GPU-Accelerated Docker Container (Local Workstations & Servers)**:
   A standardized Docker setup eliminates "works on my machine" CUDA library incompatibilities.
   ```bash
   # Clone the companion repository
   git clone https://git.mrdt.org/autonomy/Autonomy_Vision_Pipeline.git
   cd Autonomy_Vision_Pipeline

   # Build and launch the container with local GPU pass-through
   docker compose build
   docker compose up -d
   docker compose exec training-env bash
   ```
   The `Dockerfile.training` is pinned to NVIDIA CUDA 12.4, PyTorch 2.4, Ultralytics YOLOv11, OpenCV Headless, and the official Roboflow Python SDK.

2. **University HPC Module Integration (S&T Mill Cluster)**:
   For cluster users without root Docker access, the repository provides an automated environment setup script (`setup_mill_env.sh`) that loads the required CUDA modules (`module load cuda/12.4 cudnn/9.1`) and builds an isolated Python virtual environment in the user's home directory.

---

### Q2: How specifically are we improving our existing models?

Our model improvements target four engineering bottlenecks identified during past competitions and formal research (documented in [`CS 5402 Project 1.md`](file:///c:/Users/ltkli/Documents/GitHub/Autonomy_Software/docs/autonomy_binder/CS%205402%20Project%201.md)):

```
Existing Baseline (URC 2025)            Next-Gen Architecture (URC 2027)
+----------------------------------+    +----------------------------------+
| Architecture: YOLOv8n Bounding Box|   | Architecture: YOLOv11n-seg Mask  |
| Classes: Separate Object & Tag   |    | Classes: Unified 4-Class Single  |
| Inference: Dual Forward Passes   |    | Inference: Single Pass TensorRT  |
| Dataset: Heavy synthetic noise   |    | Dataset: Quality-pruned, balanced|
| False Positives: Desert rock edge|    | False Positives: Null-img pruned |
+----------------------------------+    +----------------------------------+
```

#### 1. Transitioning from Bounding Boxes to Instance Segmentation (`YOLOv11n-seg`)
Standard bounding boxes enclose target tools along with rectangular patches of background terrain. In the Hanksville desert, red sandstone rocks, tire ruts, and shadows frequently match the color and rectangular profile of a rock pick or water bottle, causing false positive bounding box detections.

As demonstrated in our published academic benchmark ([`CS 5402 Project 1.md`](file:///c:/Users/ltkli/Documents/GitHub/Autonomy_Software/docs/autonomy_binder/CS%205402%20Project%201.md)), pixel-level instance segmentation masks force the neural network's loss function to penalize background inclusion:

$$\mathcal{L}_{\text{total}} = \lambda_{\text{box}} \mathcal{L}_{\text{box}} + \lambda_{\text{cls}} \mathcal{L}_{\text{cls}} + \lambda_{\text{mask}} \mathcal{L}_{\text{mask}}$$

The pixel mask loss ($\mathcal{L}_{\text{mask}}$) requires the model to trace the exact physical contour of the tool handle and pick head, rejecting surrounding rock textures. Our custom fine-tuned `YOLOv11n-seg` model achieves a mask mAP@50 of **0.990** and recall of **0.976** with only **2.8M parameters** and **9.7 GFLOPs**, fitting comfortably on the Jetson Orin.

#### 2. Consolidating Dual Forward Passes into Unified 4-Class Inference
Currently, Autonomy's perception stack executes two forward passes per camera frame:
* Forward Pass 1: `ObjectDetector` (`bmp_v6` detecting Bottle, Mallet, Pick)
* Forward Pass 2: `TagDetector` (detecting ArUco navigation markers)

The upgraded pipeline trains a single unified backbone predicting a 4-class taxonomy:
* Class `0`: `bottle` (Green/clear Nalgene water bottle)
* Class `1`: `mallet` (Orange dead blow hammer)
* Class `2`: `pick` (Steel geological rock pick)
* Class `3`: `tag` (ArUco navigation tag on post or gate)

A single forward pass cuts GPU memory traffic and backbone computation in half, freeing compute for real-time obstacle avoidance and path planning.

#### 3. Strict Dataset Balancing & Anti-Redundancy Pruning
Historical datasets suffered from severe class imbalance: over 2,000 mallet images compared to only 400 rock pick images. The model routinely overfit on orange mallets while failing to detect steel rock picks.

The new pipeline enforces:
* **Equal Class Quotas**: Training splits are balanced within a 15% tolerance across classes.
* **Redundancy Pruning**: Automated cosine similarity pruning strips out near-duplicate video frames extracted from stationary camera footage.
* **Targeted Augmentation**: Object copy-paste augmentation inserts rock picks onto diverse desert backgrounds to balance underrepresented classes.

#### 4. Null-Image Negative Sampling
To suppress false alarms on barren desert rocks, the training pipeline mandates that **15% to 20% of the training split consists of null images** (high-resolution photos of desert dirt, gravel, tire tracks, and rover chassis shadows containing zero target objects). Training against null images teaches the network to output zero confident proposals across featureless rocky expanses.

---

### Q3: Will the training process be automated? How so?

Yes. The pipeline automates the complete lifecycle from dataset retrieval to deployable Jetson engine creation through three decoupled stages:

```
[Roboflow Cloud] 
       |
       | (Automated API Download via ROBOFLOW_API_KEY)
       v
[Stage 1: download_dataset.py]
  - Validates MD5 checksums
  - Generates data.yaml with absolute local paths
  - Verifies train/val/test split ratios
       |
       v
[Stage 2: train.py / train_mill.slurm]
  - Headless batch execution on S&T Mill HPC
  - Ultralytics automated checkpointing
  - Early stopping on validation mAP50 degradation (patience=20)
       |
       v
[Stage 3: export_model.py]
  - PyTorch (.pt) -> ONNX graph optimization
  - ONNX -> TensorRT FP16 engine (.engine)
  - Auto-generates info.txt metadata manifest
       |
       v
[Deployable Artifacts in data/models/]
```

#### Stage 1: Automated Roboflow Ingestion (`download_dataset.py`)
No team member needs to manually download zip files through a browser. The script connects to the team Roboflow workspace using a team-wide secret key (`ROBOFLOW_API_KEY`):

```bash
python scripts/download_dataset.py \
  --project "mrdt-workspace-v2/mallet-bottle-pick-detection-bak" \
  --version 8 \
  --format "yolov11" \
  --destination "./data/datasets/RSD_v8"
```
The script validates the dataset split structure, verifies image integrity, and automatically compiles the corresponding `data.yaml` descriptor.

#### Stage 2: Headless Slurm HPC Batch Execution (`train_mill.slurm`)
Training executes headlessly on the university supercomputer (S&T Mill Cluster) using automated Slurm submission templates. Team members submit training jobs without babysitting terminal sessions:

```bash
sbatch cluster/train_mill.slurm --dataset RSD_v8 --model yolov11n-seg.pt --epochs 150 --imgsz 1024
```
The script automatically configures CUDA devices, launches multi-GPU Distributed Data Parallel (DDP) training if allocated, captures tensorboard logs, and archives checkpoint weights into `/scratch/mrdt/runs/`.

#### Stage 3: Automated Multi-Format Compilation (`export_model.py`)
Once training completes, a single export script generates all runtime formats and creates the deployment metadata file required by `Autonomy_Software`:

```bash
python scripts/export_model.py \
  --weights "/scratch/mrdt/runs/RSD_v8/weights/best.pt" \
  --output-dir "./exported_models/RSD_v8_champion" \
  --imgsz 1024 \
  --device 0 \
  --half
```
This produces:
1. `best.pt`: Raw PyTorch checkpoint for archival.
2. `best.torchscript`: Serialized TorchScript model for backward-compatible LibTorch runtime.
3. `best.onnx`: Clean ONNX graph with static input dimensions and dynamic batching.
4. `best.engine`: TensorRT 10.x FP16 engine tuned specifically for the target GPU.
5. `info.txt`: Autonomy metadata manifest detailing image dimensions, mean/std normalization values, class taxonomy, and test metrics.

---

### Q4: Can we train multiple concurrently? How will the best be determined?

#### Concurrent Training Architecture
Yes. Multiple model architectures, hyperparameter configurations, and resolution profiles can be trained simultaneously through two distinct mechanisms:

1. **Slurm Job Arrays on the S&T Mill HPC**:
   The Mill cluster contains multiple nodes equipped with NVIDIA V100 and A100 GPUs. Using Slurm job arrays, developers launch parallel sweep experiments across up to 8 cluster nodes in a single command:
   ```bash
   sbatch --array=0-5 cluster/sweep_architectures.slurm
   ```
   The array script maps each task ID to a distinct configuration in `configs/sweeps/model_sweep.yaml`:
   * Task 0: `yolov8n` ($640\times 640$, batch 32)
   * Task 1: `yolov8s` ($640\times 640$, batch 16)
   * Task 2: `yolov11n` ($640\times 640$, batch 32)
   * Task 3: `yolov11s` ($640\times 640$, batch 16)
   * Task 4: `yolov11n-seg` ($640\times 640$, batch 32)
   * Task 5: `yolov11n-seg` ($1024\times 1024$, batch 16)

2. **Automated Hyperparameter Tuning (Local/Server sweeps)**:
   For fine-tuning learning rates, weight decay, and augmentation intensities, `train.py` integrates with Optuna to run multi-trial Bayesian optimization sweeps in parallel across local GPUs.

#### Objective Selection Criteria: The Champion Scoring Function
To prevent subjective bias when selecting competition models, candidate models are evaluated using a standardized evaluation script (`compare_models.py`) running against a frozen, held-out competition benchmark dataset (`URC_Hanksville_Benchmark`).

Models are ranked using a composite scoring formula that prioritizes **high recall** (never missing a tool in the dirt) and **low edge latency** (real-time Jetson execution):

$$\text{Composite Score} = 0.40 \cdot \text{mAP@50} + 0.20 \cdot \text{mAP@50-95} + 0.30 \cdot \text{Recall} + 0.10 \cdot S_{\text{latency}}$$

Where the latency penalty score $S_{\text{latency}}$ is calculated relative to our target edge threshold of 30 milliseconds (33.3 FPS):

$$S_{\text{latency}} = \max\left(0.0, \, 1.0 - \frac{t_{\text{inference\_ms}}}{30.0}\right)$$

```
Candidate Evaluation Matrix (Benchmark on Hanksville Test Set)
+----------------+------------+---------+--------+--------------+-------------+
| Model          | Resolution | mAP@50  | Recall | Latency (TRT)| Score       |
+----------------+------------+---------+--------+--------------+-------------+
| yolov8n        | 640x640    | 0.921   | 0.884  | 8.2 ms       | 0.887       |
| yolov11n       | 640x640    | 0.945   | 0.912  | 7.9 ms       | 0.913       |
| yolov11s       | 640x640    | 0.952   | 0.920  | 14.5 ms      | 0.908       |
| yolov11n-seg   | 640x640    | 0.978   | 0.954  | 11.4 ms      | 0.949       |
| yolov11n-seg * | 1024x1024  | 0.990   | 0.976  | 18.2 ms      | 0.962 (WIN) |
| yolov11s-seg   | 1024x1024  | 0.946   | 0.931  | 38.6 ms      | 0.858       |
+----------------+------------+---------+--------+--------------+-------------+
* Winning champion configuration for production rover deployment.
```

The model achieving the highest composite score is automatically designated the production release candidate.

---

## 3. Directory Layout & Architecture

The `Autonomy_Vision_Pipeline` repository is organized into distinct functional modules:

```text
Autonomy_Vision_Pipeline/
├── configs/
│   ├── data_specs/
│   │   ├── unified_4class.yaml         # Classes: bottle, mallet, pick, tag
│   │   └── astronaut_pose.yaml         # Classes & keypoint links for astronaut
│   ├── sweeps/
│   │   └── model_sweep.yaml            # Matrix for concurrent Slurm sweep
│   └── training_defaults.yaml          # Standard hyperparameters (LR, optimizer)
├── docker/
│   ├── Dockerfile.training             # CUDA 12.4 + PyTorch + TensorRT container
│   ├── docker-compose.yml              # GPU pass-through orchestration
│   └── entrypoint.sh
├── cluster/
│   ├── train_mill.slurm                # Single-job Slurm script for S&T Mill HPC
│   ├── sweep_architectures.slurm       # Multi-job Slurm array for concurrent training
│   └── setup_mill_env.sh              # Python venv builder for non-root Mill users
├── scripts/
│   ├── download_dataset.py             # Roboflow API automated dataset synchronization
│   ├── extract_keyframes.py            # Video log ingestion & frame rate pruning
│   ├── audit_dataset.py                # Class balance & null image verification
│   ├── custom_augment.py               # Desert HSV jitter & copy-paste balancing
│   ├── train.py                        # Unified training entry point
│   ├── compare_models.py               # Multi-model benchmarking & leaderboard generator
│   └── export_model.py                 # PyTorch -> ONNX -> TensorRT compiler
├── tests/
│   ├── test_dataset_integrity.py       # Pytest suite verifying label correctness
│   └── test_export_pipeline.py         # Verification of exported model shapes
├── README.md
└── requirements.txt
```

---

## 4. Subteam Collaboration & Multi-Task Support

The AI Pipeline is designed as shared infrastructure serving both perception projects:
1. **Tool & Marker Detection/Segmentation** (Lead: Oscar & Saurav)
2. **Astronaut Pose Estimation** (Lead: Kara & Thomas)

```
                 +-------------------------------------------------+
                 |  Oscar & Saurav: Autonomy_Vision_Pipeline Core   |
                 |  (Docker, Mill Slurm, Roboflow Sync, TRT Export)|
                 +-----------------------+-------------------------+
                                         |
                     +-------------------+-------------------+
                     |                                       |
                     v                                       v
+------------------------------------+   +------------------------------------+
| Tools & Markers Track              |   | Astronaut Pose Track               |
| Owners: Oscar & Saurav             |   | Owners: Kara & Thomas              |
| Config: configs/unified_4class.yaml|   | Config: configs/astronaut_pose.yaml|
| Model: YOLOv11n-seg                |   | Model: YOLOv11n-pose               |
| Dataset: RoveSoDataset v8 (Roboflow|   | Dataset: Astronaut Suit Keypoints  |
| Output: best_unified_tools.engine  |   | Output: best_astronaut_pose.engine |
+------------------------------------+   +------------------------------------+
```

### Shared Infrastructure Responsibilities
* **Oscar & Saurav Provide**:
  - Maintained Docker environments and S&T Mill Slurm submission scripts.
  - Robust export tools compiling PyTorch models into TensorRT FP16 engines.
  - Automated evaluation harness computing latency and precision metrics.
* **Kara & Thomas Provide**:
  - Keypoint dataset definitions and annotation guidelines for the astronaut suit.
  - Hand gesture evaluation criteria and custom pose config YAMLs.

---

## 5. Deployment Handoff to `Autonomy_Software`

Once a champion model is validated and compiled, it is delivered into the core autonomy repository ([`Autonomy_Software`](file:///c:/Users/ltkli/Documents/GitHub/Autonomy_Software)) following a strict deployment contract.

### Target Location & Naming Conventions
Compiled artifacts are placed in the `data/models/` directory:
```text
Autonomy_Software/
└── data/
    └── models/
        ├── unified_tools_v1.engine       # 4-class tool and tag TensorRT engine
        ├── unified_tools_v1_info.txt     # Metadata manifest
        ├── astronaut_pose_v1.engine      # Pose estimation TensorRT engine
        └── astronaut_pose_v1_info.txt    # Metadata manifest
```

### The Metadata Manifest (`info.txt`)
To prevent runtime configuration drift, every exported model includes a standardized `info.txt` file parsed by the C++ perception driver on startup:

```ini
[ModelMetadata]
name=unified_tools_yolov11n_seg
version=1.0.0
build_date=2026-11-05
input_width=1024
input_height=1024
channels=3
framework=TensorRT-10.4-FP16
precision=FP16

[Normalization]
scale=0.00392156862745098  ; 1.0 / 255.0
mean_r=0.0
mean_g=0.0
mean_b=0.0
std_r=1.0
std_g=1.0
std_b=1.0

[Taxonomy]
num_classes=4
class_0=bottle
class_1=mallet
class_2=pick
class_3=tag

[ConfidenceThresholds]
conf_thresh_bottle=0.55
conf_thresh_mallet=0.60
conf_thresh_pick=0.50
conf_thresh_tag=0.70
iou_nms_thresh=0.45
```

---

## 6. Verification & Testing Plan

### Automated Test Matrix
1. **Dataset Integrity Verification (`test_dataset_integrity.py`)**:
   - Asserts that all image files are non-corrupt and match bounding box/polygon dimensions.
   - Verifies class distribution across splits to guarantee no class exceeds 20% imbalance.
   - Asserts that null images contain zero label annotations.
2. **Export Dimensionality Verification (`test_export_pipeline.py`)**:
   - Executes dummy inference on exported `.onnx` and `.engine` files using random tensors.
   - Verifies that output tensor dimensions strictly match expected grid and mask dimensions.
3. **Jetson Parity Benchmark**:
   - Executes 500 frames on the physical Jetson AGX Orin.
   - Compares predictions against the workstation PyTorch output to verify FP16 numerical parity (mean IOU agreement $>0.98$).

---

## 7. CDR Presentation Outline (Slide 14 Guide)

When presenting Slide 14 at the Software Critical Design Review, Oscar and Saurav should structure their delivery around these four points:

1. **Slide Title**: Perception AI Infrastructure & Model Pipeline
2. **Speaker Notes & Key Messages**:
   * *Point 1 (Repository)*: "We have centralized all machine learning development into a dedicated GitLab companion repo, `Autonomy_Vision_Pipeline`. It features a standardized Docker environment and automated S&T Mill HPC integration so training workflows persist across team generations."
   * *Point 2 (Improvements)*: "We are moving from basic bounding boxes to `YOLOv11n-seg` instance segmentation. Segmentation masks eliminate false positives on background desert rocks. We have also unified tool and tag detection into a single forward pass, doubling inference throughput on our Jetson."
   * *Point 3 (Automation)*: "The pipeline is fully automated. Roboflow datasets download directly via API key, Slurm scripts handle batch training on the Mill HPC, and our export tool automatically compiles PyTorch models into TensorRT FP16 engines with companion metadata manifests."
   * *Point 4 (Concurrent Training & Selection)*: "We leverage Slurm job arrays on the Mill cluster to train multiple model sizes and resolutions in parallel. Champions are selected objectively using a composite scorecard weighting mAP@50, recall, and real-time Jetson latency."
