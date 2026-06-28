# Test Plan — LK vs Alternative Optical-Flow / Feature-Matching Trackers

**Conda env:** `drone`  
**Scripts to copy into this folder:** `flow_odometry.py`, `compare_trackers.py`

---

## Context

Exp01–03 established:
- LK (Shi-Tomasi + pyramidal `calcOpticalFlowPyrLK` + FB check) runs at **15 ms/frame** with **7.7 m RMSE** (agl, GT attitude, short dataset)
- Best deployable config: `--depth agl --stride 5 --attitude ahrs_compass` → 21.4 m flow-odom RMSE
- LK tracking (feat + flow + FB) costs only **1.7 ms combined** of the 15 ms total

The tracker has never been swapped out. This experiment asks: **does the choice of flow / matching method matter for odometry accuracy and latency?**

Six methods are compared. All use identical odometry math (de-rotate + LS solve); only the detection and flow/matching step differ.

---

## Dataset

Short only (`_in/isaac-sim-20260624_2337`, 7.5 min, 1 773 m, ~43 m mean alt).

---

## Methods

| ID | Detector | Flow / matching method | Filter | Notes |
|----|----------|------------------------|--------|-------|
| **lk** | Shi-Tomasi (`goodFeaturesToTrack`) | Pyramidal `calcOpticalFlowPyrLK` | FB check (1 px) | Current baseline |
| **fast_lk** | FAST corners (OpenVINS style) | Same pyramidal LK | FB check (1 px) | Corner detector swap only |
| **farneback** | Shi-Tomasi | `calcOpticalFlowFarneback` (dense), sampled at detected pts | FB check on dense inverse | Classic dense |
| **dis** | Shi-Tomasi | `DISOpticalFlow` medium preset (dense), sampled at detected pts | FB check on dense inverse | Fast dense |
| **orb** | ORB (`ORB_create`, 600 kpts) | BFMatcher (Hamming, crossCheck) | RANSAC F-matrix (1 px, 99.9%) | Descriptor matching |
| **sparse_raft** | Shi-Tomasi | RAFT-Small (torchvision) dense, sampled at detected pts | FB check on dense inverse | Deep learning |

ORB paradigm differs: detects independently in both frames then matches; prev-frame features are cached and reused next iteration. All others detect on prev and flow to cur.

---

## Experiment 1: Accuracy per Tracker (individual runs)

Settings: `--depth agl --stride 5 --attitude ahrs_compass`

| Run | Tracker | Command |
|-----|---------|---------|
| T-LK | lk | `conda run -n drone python experiment/04/flow_odometry.py --dir _in/isaac-sim-20260624_2337 --depth agl --stride 5 --attitude ahrs_compass --tracker lk` |
| T-FAST | fast_lk | `… --tracker fast_lk` |
| T-FARN | farneback | `… --tracker farneback` |
| T-DIS | dis | `… --tracker dis` |
| T-ORB | orb | `… --tracker orb` |
| T-RAFT | sparse_raft | `… --tracker sparse_raft` |

**Metrics:** flow-odom RMSE (m), final error (m), mean inlier count per frame.

---

## Experiment 2: Latency per Tracker (individual --benchmark runs)

Same settings + `--benchmark`.

**Metrics:** mean/P95 total latency (ms), sub-step breakdown (detect / track / solve), over-budget frame count.

---

## Experiment 3: Combined Comparison (single command)

Runs all six trackers sequentially and produces a unified accuracy + latency report:

```bash
conda run -n drone python experiment/04/compare_trackers.py \
    --dir _in/isaac-sim-20260624_2337
```

Outputs:
- Terminal: summary table (RMSE | final error | inliers | detect ms | track ms | solve ms | total ms | margin ms)
- `compare_trajectories.png` — all six trajectories + GT overlaid, plus error-over-time curves
- `compare_latency.png` — stacked bar chart: detect / track / solve per tracker, budget line

---

## Summary Table (targets)

| Tracker | Expected RMSE | Expected latency | Real-time? |
|---------|---------------|------------------|------------|
| lk | ~21.4 m (Exp02/03 ref) | ~15 ms | ✓ |
| fast_lk | ~21–23 m | ~14–16 ms | ✓ |
| farneback | ~20–25 m | ~20–40 ms | ✓ |
| dis | ~20–24 m | ~15–25 ms | ✓ |
| orb | ~22–28 m | ~5–15 ms | ✓ |
| sparse_raft | ~19–22 m | ~30–80 ms | marginal |

Primary question: **does any method beat LK RMSE while remaining within the 80 ms budget?**

---

## Open Items Carried from Exp03

- **Compass noise sensitivity:** `mag_noise_deg > 0` — quantify degradation from real magnetometer noise; defer to Exp05
- **Short-flight tilt gap:** AHRS roll/pitch scale error (1.31) — structural fix requires rangefinder or multi-cam
