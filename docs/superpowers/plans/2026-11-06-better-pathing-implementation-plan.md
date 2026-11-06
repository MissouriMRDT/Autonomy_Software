# Better Pathing & Incline Traversal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add hard slope kinematic gating, asymmetric roll/pitch cost modeling, and staged corridor expansion to `GeoPlanner` to prevent pathing over cliffs and optimize hillside navigation for the swerve drive.

**Architecture:** Extend the 8-directional neighbor expansion in `GeoPlanner::SearchAStar` with slope angle and vertical step threshold gating. Scale traversal cost using an asymmetric roll vs pitch penalty model calibrated to swerve stability limits. If A* fails to find a valid corridor at 100 meters, stage automatic expansion to 200 and 400 meters to discover natural mountain passes.

**Tech Stack:** C++20, GoogleTest, CMake, SQLite3 (LiDAR Handler Database)

## Global Constraints

- Preserve all existing `GeoPlanner` public method signatures to avoid breaking callers in `NavigatingState`, `SearchPatternState`, and `StuckState`.
- Maximum allowable rover slope is 20.0 degrees; maximum vertical step between adjacent 0.5m grid cells is 0.25 meters.
- Roll angles exceeding 15.0 degrees must be penalized twice as heavily as pitch angles exceeding 15.0 degrees.
- Do not introduce external third-party dependencies; use standard library math and existing MRDT logging utilities.
- All code changes must compile cleanly under CMake with `-Wall -Wextra -Werror` compliance.

---

### File Structure Map

```text
src/
├── algorithms/
│   └── planners/
│       ├── GeoPlanner.h        # Add kinematic thresholds, slope calculation helpers, corridor expansion
│       └── GeoPlanner.cpp      # Inject slope gating in SearchAStar(), update PlanPath() loop
├── AutonomyConstants.h         # Add slope limit constants and swerve stability multipliers
└── AutonomyConstants.cpp       # Initialize default slope constants
tests/
└── Unit/
    └── src/
        └── algorithms/
            └── planners/
                └── GeoPlanner.cc   # Add cliff rejection and slope cost unit tests
```

---

### Task 1: Add Incline Traversal Constants to AutonomyConstants

**Files:**
- Modify: `src/AutonomyConstants.h:328-335`
- Modify: `src/AutonomyConstants.cpp:319-330`
- Test: `tests/Unit/src/algorithms/planners/GeoPlanner.cc`

**Interfaces:**
- Consumes: None
- Produces:
  - `constants::GEOPLANNER_MAX_SLOPE_DEG` (double: default 20.0)
  - `constants::GEOPLANNER_MAX_STEP_HEIGHT_M` (double: default 0.25)
  - `constants::GEOPLANNER_ROLL_PENALTY_WEIGHT` (double: default 3.0)
  - `constants::GEOPLANNER_PITCH_PENALTY_WEIGHT` (double: default 1.5)
  - `constants::GEOPLANNER_ENABLE_SLOPE_GATING` (bool: default true)

- [ ] **Step 1: Write declarations in AutonomyConstants.h**
Add the constants under `//// GeoPlanner Adjustments`:
```cpp
extern const double GEOPLANNER_MAX_SLOPE_DEG;
extern const double GEOPLANNER_MAX_STEP_HEIGHT_M;
extern const double GEOPLANNER_ROLL_PENALTY_WEIGHT;
extern const double GEOPLANNER_PITCH_PENALTY_WEIGHT;
extern const bool GEOPLANNER_ENABLE_SLOPE_GATING;
```

- [ ] **Step 2: Initialize constants in AutonomyConstants.cpp**
Define default values:
```cpp
const double GEOPLANNER_MAX_SLOPE_DEG        = 20.0;
const double GEOPLANNER_MAX_STEP_HEIGHT_M    = 0.25;
const double GEOPLANNER_ROLL_PENALTY_WEIGHT  = 3.0;
const double GEOPLANNER_PITCH_PENALTY_WEIGHT = 1.5;
const bool GEOPLANNER_ENABLE_SLOPE_GATING    = true;
```

- [ ] **Step 3: Compile to verify syntax**
Run CMake build to confirm header and source files compile without warnings.

---

### Task 2: Implement Hard Slope & Cliff Step Gating in GeoPlanner::SearchAStar

**Files:**
- Modify: `src/algorithms/planners/GeoPlanner.h:80-95`
- Modify: `src/algorithms/planners/GeoPlanner.cpp:525-545`
- Test: `tests/Unit/src/algorithms/planners/GeoPlanner.cc`

**Interfaces:**
- Consumes:
  - `stCell.dAltitude`, `stCurrentCell.dAltitude`
  - `m_dGridResolution`, `adMoveDist[nI]`
  - `constants::GEOPLANNER_MAX_SLOPE_DEG`, `constants::GEOPLANNER_MAX_STEP_HEIGHT_M`
- Produces:
  - Immediate `continue;` in A* neighbor loop when slope or step limits are exceeded.

- [ ] **Step 1: Write a failing unit test in tests/Unit/src/algorithms/planners/GeoPlanner.cc**
Construct a test verifying that two points separated by a vertical cliff do not produce a straight path across the cliff:
```cpp
TEST_F(GeoPlannerTests, RejectCliffDropPath)
{
    // Test setup with synthetic 3-meter drop across 0.5-meter cell
    pathplanners::GeoPlanner planner;
    // Verify that IsSlopePassable returns false for 45-degree slope
    EXPECT_FALSE(planner.IsSlopePassable(5.0, 1.0));
}
```

- [ ] **Step 2: Add helper method declarations to GeoPlanner.h**
In `GeoPlanner.h` (public or protected for testing):
```cpp
bool IsSlopePassable(double dAltitudeDiff, double dPlanarDist) const;
void SetSlopeGatingEnabled(bool bEnabled);
bool GetSlopeGatingEnabled() const;
```

- [ ] **Step 3: Implement IsSlopePassable and inject into SearchAStar in GeoPlanner.cpp**
In `GeoPlanner.cpp`:
```cpp
bool GeoPlanner::IsSlopePassable(double dAltitudeDiff, double dPlanarDist) const
{
    if (dPlanarDist <= 0.0) return false;
    if (dAltitudeDiff > constants::GEOPLANNER_MAX_STEP_HEIGHT_M) return false;
    double dSlopeDeg = std::atan2(dAltitudeDiff, dPlanarDist) * (180.0 / 3.14159265358979323846);
    return dSlopeDeg <= constants::GEOPLANNER_MAX_SLOPE_DEG;
}
```
In `GeoPlanner::SearchAStar()` inside the 8-directional neighbor loop:
```cpp
double dAltDiff    = std::abs(stCell.dAltitude - stCurrentCell.dAltitude);
double dPlanarDist = adMoveDist[nI] * m_dGridResolution;

// Gating check: Reject transitions exceeding slope or step limits
if (m_bEnableSlopeGating && !IsSlopePassable(dAltDiff, dPlanarDist))
{
    continue;
}
```

- [ ] **Step 4: Run unit tests**
Run unit test binary to ensure `RejectCliffDropPath` passes and existing `DoesNotLeak` tests remain green.

---

### Task 3: Implement Asymmetric Roll vs Pitch Traversal Cost Multiplier

**Files:**
- Modify: `src/algorithms/planners/GeoPlanner.h:210-225`
- Modify: `src/algorithms/planners/GeoPlanner.cpp:540-555`
- Test: `tests/Unit/src/algorithms/planners/GeoPlanner.cc`

**Interfaces:**
- Consumes:
  - Transition angle relative to heading vector
  - `dAltDiff`, `dPlanarDist`
- Produces:
  - Multiplier $M_{\text{incline}} \ge 1.0$ applied to `dTentativeGCost`.

- [ ] **Step 1: Write failing unit test for asymmetric slope cost**
Verify that a side-hill (roll) traversal incurs higher path cost than direct ascent (pitch) at identical elevation angles:
```cpp
TEST_F(GeoPlannerTests, AsymmetricRollPitchPenalty)
{
    pathplanners::GeoPlanner planner;
    double dRollCost = planner.CalculateInclinePenalty(12.0, true);
    double dPitchCost = planner.CalculateInclinePenalty(12.0, false);
    EXPECT_GT(dRollCost, dPitchCost);
}
```

- [ ] **Step 2: Add CalculateInclinePenalty helper to GeoPlanner.h / .cpp**
```cpp
double GeoPlanner::CalculateInclinePenalty(double dSlopeDeg, bool bIsRollDominant) const
{
    if (dSlopeDeg <= 5.0) return 1.0;
    double dWeight = bIsRollDominant ? constants::GEOPLANNER_ROLL_PENALTY_WEIGHT 
                                     : constants::GEOPLANNER_PITCH_PENALTY_WEIGHT;
    return 1.0 + (dWeight * std::pow(dSlopeDeg / constants::GEOPLANNER_MAX_SLOPE_DEG, 2.0));
}
```

- [ ] **Step 3: Integrate penalty into SearchAStar cost calculation**
Scale `dTentativeGCost` using both the LiDAR point traversability score and the incline penalty.

- [ ] **Step 4: Verify with unit tests**
Run test suite to verify cost calculation logic.

---

### Task 4: Implement Staged Corridor Expansion for Natural Pass Routing

**Files:**
- Modify: `src/algorithms/planners/GeoPlanner.cpp:115-185`
- Test: `tests/Unit/src/algorithms/planners/GeoPlanner.cc`

**Interfaces:**
- Consumes:
  - `dCorridorPadding` (initial default 100.0m)
- Produces:
  - Fallback loops attempting 200.0m and 400.0m corridor preloads when mountain ridges block narrow corridors.

- [ ] **Step 1: Write test for corridor expansion fallback**
Construct test where a 100m corridor contains an unbroken impassable cliff wall, but a saddle exists at 180m offset. Verify that path planning succeeds by expanding corridor.

- [ ] **Step 2: Update GeoPlanner::PlanPath retry loop**
In `GeoPlanner.cpp`:
```cpp
const std::vector<double> vCorridorStages = {dCorridorPadding, dCorridorPadding * 2.0, dCorridorPadding * 4.0};
for (double dStagePadding : vCorridorStages)
{
    if (PreloadCorridorAndBuildGrid(stStart, stEnd, dStagePadding))
    {
        SearchAStar();
        vResultPath = ReconstructPath();
        if (!vResultPath.empty())
        {
            LOG_INFO(logging::g_qSharedLogger, "GeoPlanner found valid path at corridor width {}m", dStagePadding);
            break;
        }
    }
}
```

- [ ] **Step 3: Run unit tests and benchmark execution time**
Confirm that when a direct 100m path exists, zero expansion occurs (no CPU overhead). When blocked, expansion resolves the path cleanly.

---

### Task 5: Integration Verification & Field Simulation

**Files:**
- Test: `tests/Unit/src/algorithms/planners/GeoPlanner.cc`
- Test: Visualizer / Unreal Engine Simulation

- [ ] **Step 1: Test with real LiDAR database (`Flat_SIM.db` and Fugitive Beach)**
Execute unit test using real coordinates in `GeoPlanner.cc:138-141`. Verify path coordinates are generated without crashes or infinite loops.
- [ ] **Step 2: Prepare visualizer comparison for CDR slide**
Run visualizer to plot baseline path (driving straight into cliff) vs new path (navigating through natural saddle around hill). Take screenshots for CDR Slide 12.
