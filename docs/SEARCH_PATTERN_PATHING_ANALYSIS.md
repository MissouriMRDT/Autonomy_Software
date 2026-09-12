# Search Pattern Path Tracking & Circling Analysis

## Executive Summary

During autonomous operations, when entering `SearchPatternState`, the rover generates an Archimedean spiral search pattern, removes red-zone obstacles, and routes the path through the LiDAR grid using `GeoPlanner`. 

Under certain conditions—especially during longer runs or when video streaming / vision pipelines are active—the rover may deviate from the spiral path, veer off-course, and enter a permanent sequence of tight circles/loops rather than completing the spiral.

This document details the exact mathematics, control flow, and code mechanics causing this behavior for future reference. **Per instruction, no changes have been made to the pathing code.**

---

## 1. System Architecture & Control Pipeline

```
SearchPatternState::Start()
    │
    ├── 1. CalculateSpiralPatternWaypoints()   --> Generates skeleton spiral (1m radial step, 57° angular step)
    ├── 2. RemoveRedZonePoints()               --> Drops waypoints in unnavigable LiDAR tiles
    ├── 3. GeoPlanSearchPattern()              --> Runs A* between every pair of skeleton points
    │                                              Dense output: 300–800 waypoints (~0.2m–0.4m spacing)
    └── 4. PurePursuitController Configuration
            ├── SetReferencePath(m_vSearchPath)
            └── SetLookaheadIndex(5)           <-- Configured for 5 waypoints
```

In `SearchPatternState::Run()`, every cycle:
1. `PurePursuitController::Calculate(stCurrentRoverPose, dVelocity)` computes target heading.
2. `DriveBoard::CalculateMove(dVelocity, dGoalHeading, dCompassHeading, eArcadeDrive)` computes wheel speeds.
3. Wheel speeds are transmitted over RoveComm to the drivetrain.

---

## 2. Root Cause Analysis of the Circling Behavior

### Root Cause A: Topological Blinder Premature Stall on Dense Paths

In `src/algorithms/controllers/PurePursuitController.cpp`:
```cpp
int PurePursuitController::FindClosestWaypointIndex(const geoops::UTMCoordinate& stCurrentPosition)
{
    // ...
    for (size_t siIter = static_cast<size_t>(m_nCurrentReferencePathTargetIndex); siIter < m_vReferencePath.size(); ++siIter)
    {
        // ...
        if (dDistSq < dClosestDistanceSq)
        {
            dClosestDistanceSq    = dDistSq;
            nBestSegmentIndex     = static_cast<int>(siIter);
            nConsecutiveIncreases = 0;
        }
        else
        {
            nConsecutiveIncreases++;
            // TOPOLOGICAL BLINDER:
            // If the distance increases for N consecutive points, we have found the local minimum.
            if (nConsecutiveIncreases >= m_nLookaheadIndex)  // m_nLookaheadIndex == 5!
            {
                break;
            }
        }
    }
    return nBestSegmentIndex;
}
```

#### Why it fails:
1. **Original Assumption:** When `PurePursuitController` was originally written, reference paths were coarse skeleton waypoints spaced 2m–5m apart. 5 consecutive increases represented 10m–25m of path, which effectively prevented jumping to adjacent spiral loops.
2. **Dense Path Reality:** `GeoPlanSearchPattern` inserts dense A* grid nodes spaced ~0.2m to 0.4m apart. 5 waypoints corresponds to only **1.0 to 1.5 meters** along the path.
3. **Curvature Trigger:** Along a spiral curve or during a turn, if the rover has a slight cross-track offset, the Euclidean distance from the rover to the next 5 waypoints can easily increase by tiny increments (e.g., 2.01m, 2.02m, 2.03m, 2.04m, 2.05m).
4. **Permanent Stall:** The loop breaks after checking only 5 waypoints (~1 meter). It never inspects waypoint 6, 7, 8... where the path curves back closer to the rover. On the next tick, the loop starts from the same stalled index, immediately sees 5 increases again, and **permanently freezes `m_nCurrentReferencePathTargetIndex` in place behind the rover**.

---

### Root Cause B: Backward-Facing Off-Path Recovery

In `PurePursuitController::FindLookaheadWaypoint`:
```cpp
const geoops::UTMCoordinate& stClosest = m_vReferencePath[m_nCurrentReferencePathTargetIndex].GetUTMCoordinate();
double dClosestDist = std::hypot(stClosest.dEasting - stCurrentPosition.dEasting, 
                                 stClosest.dNorthing - stCurrentPosition.dNorthing);

// OFF-PATH RECOVERY:
if (dClosestDist > m_dLookaheadDistance) // m_dLookaheadDistance == 2.0m
{
    return m_vReferencePath[m_nCurrentReferencePathTargetIndex];
}
```

#### Why it creates infinite circles:
1. As the rover continues forward, its distance from the stalled index eventually exceeds `m_dLookaheadDistance` (2.0m).
2. The controller triggers `OFF-PATH RECOVERY`, returning `m_vReferencePath[m_nCurrentReferencePathTargetIndex]`.
3. Because the stalled waypoint is **behind** the rover, the heading calculation:
   $$\vec{v} = P_{\text{target}} - P_{\text{rover}}$$
   produces a vector pointing backwards.
4. The rover executes a 180° turn to drive back towards the stalled waypoint.
5. As it approaches within 2.0m, the regular forward search resumes, targeting a point ahead. The rover swings forward, overshoots past 2.0m, triggers off-path recovery behind it again, and reverses direction.
6. **Result:** The rover is trapped in an infinite limit cycle, spinning in tight circles around a phantom stalled waypoint.

---

### Root Cause C: Interaction with Camera Streams and System Latency

Why is this problem significantly more frequent or apparent when camera streams (WebRTC / ZED / RealSense) are active?

1. **CPU / Thread Contention:** Video decoding (H.264 packet decompression, YUV-to-RGB conversion, SWS scaling) and computer vision inference (ArUco, YOLO) consume significant CPU cycles.
2. **Cycle Timing Jitter:** When thread scheduling introduces minor latency spikes in the state machine loop, the vehicle travels further between control updates.
3. **Turn Overshoot:** In a continuous turn like a spiral, a delay of 50–100ms causes the rover to drift slightly outward by an additional 10–30 cm.
4. **Trigger Threshold:** When cross-track drift crosses the 2.0m lookahead radius threshold, off-path recovery activates immediately. Without camera load, the rover might stay just below 2.0m (e.g. 1.8m) and avoid triggering the 180° turnaround trap.

---

## 3. Recommended Future Remediation

When the team decides to update the path tracking logic, the following changes will resolve the issue permanently:

### 1. Physical Arc-Length Horizon in `FindClosestWaypointIndex`
Replace discrete index count (`nConsecutiveIncreases >= 5`) with a continuous physical path horizon:
- Search forward along the path up to a distance horizon (e.g., 15.0 meters). In a spiral, adjacent loops are separated by $>18$ meters of path length, so a 15-meter path horizon is topologically guaranteed never to jump rings.
- Only break early if Euclidean distance exceeds the minimum by a substantial margin (e.g. $>3.0$ meters) over at least 15 waypoints.

### 2. Arc-Length Lookahead in `FindLookaheadWaypoint`
Replace the backward-facing target with path-relative forward projection:
- Accumulate distance forward along the path curve from the closest waypoint until reaching the desired lookahead distance ($L_{\text{eff}} = \max(L, d_{\perp} \times 0.8)$).
- Linearly interpolate along the target segment for smooth tracking.
- Because the target is defined along the curve ahead of the closest point, the vector from the rover to the target always has a positive forward component along the path, guaranteeing the rover never commands a 180° turnaround.

### 3. End-of-Path Distance Metric
In `PurePursuitController::Calculate`, update the arrival check to calculate Euclidean distance to `stLastPoint` rather than `stSecondToLastPoint`.
