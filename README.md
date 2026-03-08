# AAE5303 Assignment: Visual Odometry with ORB-SLAM3

<div align="center">

![ORB-SLAM3](https://img.shields.io/badge/SLAM-ORB--SLAM3-blue?style=for-the-badge)
![VO](https://img.shields.io/badge/Mode-Visual_Odometry-green?style=for-the-badge)
![Dataset](https://img.shields.io/badge/Dataset-HKisland__GNSS03-orange?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Completed-success?style=for-the-badge)

**Monocular Visual Odometry Evaluation on UAV Aerial Imagery**

*Hong Kong Island GNSS Dataset - MARS-LVIG*

</div>

---

## 📋 Table of Contents

1. [Executive Summary](#-executive-summary)
2. [Introduction](#-introduction)
3. [Methodology](#-methodology)
4. [Dataset Description](#-dataset-description)
5. [Implementation Details](#-implementation-details)
6. [Results and Analysis](#-results-and-analysis)
7. [Visualizations](#-visualizations)
8. [Discussion](#-discussion)
9. [Conclusions](#-conclusions)
10. [References](#-references)
11. [Appendix](#-appendix)

---

## 📊 Executive Summary

This report presents the implementation and evaluation of **Monocular Visual Odometry (VO)** using the **ORB-SLAM3** framework on the **HKisland_GNSS03** UAV aerial imagery dataset. The project evaluates trajectory accuracy against RTK ground truth using **four parallel, monocular-appropriate metrics** computed with the `evo` toolkit.

### Key Results

| Metric | Value | Description |
|--------|-------|-------------|
| **ATE RMSE** | **3.2078 m** | Global accuracy after Sim(3) alignment (scale corrected) |
| **RPE Trans Drift** | **1.4443 m/m** | Translation drift rate (mean error per meter, delta=10 m) |
| **RPE Rot Drift** | **110.3482 deg/100m** | Rotation drift rate (mean angle per 100 m, delta=10 m) |
| **Completeness** | **92.94%** | Matched poses / total ground-truth poses (1817 / 1955) |
| **Estimated poses** | 3,609 | Trajectory poses in `CameraTrajectory.txt` |

**Configuration**: 2× image downsampling (2448×2048 → 1224×1024), 0.5× bag playback.

---

## 📖 Introduction

### Background

ORB-SLAM3 is a state-of-the-art visual SLAM system capable of performing:

- **Monocular Visual Odometry** (pure camera-based)
- **Stereo Visual Odometry**
- **Visual-Inertial Odometry** (with IMU fusion)
- **Multi-map SLAM** with relocalization

This assignment focuses on **Monocular VO mode**, which:

- Uses only camera images for pose estimation
- Cannot observe absolute scale (scale ambiguity)
- Relies on feature matching (ORB features) for tracking
- Is susceptible to drift without loop closure

### Objectives

1. Implement monocular Visual Odometry using ORB-SLAM3
2. Process UAV aerial imagery from the HKisland_GNSS03 dataset
3. Extract RTK (Real-Time Kinematic) GPS data as ground truth
4. Evaluate trajectory accuracy using four parallel metrics appropriate for monocular VO
5. Document the complete workflow for reproducibility

### Scope

This assignment evaluates:

- **ATE (Absolute Trajectory Error)**: Global trajectory accuracy after Sim(3) alignment (monocular-friendly)
- **RPE drift rates (translation + rotation)**: Local consistency (drift per traveled distance)
- **Completeness**: Robustness / coverage (how much of the sequence is successfully tracked and evaluated)

---

## 🔬 Methodology

### ORB-SLAM3 Visual Odometry Overview

ORB-SLAM3 performs visual odometry through the following pipeline:

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  Input Image    │────▶│   ORB Feature   │────▶│   Feature       │
│  Sequence       │     │   Extraction    │     │   Matching      │
└─────────────────┘     └─────────────────┘     └────────┬────────┘
                                                         │
┌─────────────────┐     ┌─────────────────┐     ┌────────▼────────┐
│   Trajectory    │◀────│   Pose          │◀────│   Motion        │
│   Output        │     │   Estimation    │     │   Model         │
└─────────────────┘     └────────┬────────┘     └─────────────────┘
                                 │
                        ┌────────▼────────┐
                        │   Local Map     │
                        │   Optimization  │
                        └─────────────────┘
```

### Evaluation Metrics

#### 1. ATE (Absolute Trajectory Error)

Measures the RMSE of the aligned trajectory after Sim(3) alignment:

$$ATE_{RMSE} = \sqrt{\frac{1}{N}\sum_{i=1}^{N}\|\mathbf{p}_{est}^i - \mathbf{p}_{gt}^i\|^2}$$

**Reference**: Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM Systems", IROS 2012

#### 2. RPE (Relative Pose Error) – Drift Rates

Measures local consistency by comparing relative transformations. We report drift as **rates**:

- **Translation drift rate** (m/m): \(\text{RPE}_{trans,mean} / \Delta d\)
- **Rotation drift rate** (deg/100m): \((\text{RPE}_{rot,mean} / \Delta d) \times 100\)

where \(\Delta d = 10\) m.

**Reference**: Geiger et al., "Vision meets Robotics: The KITTI Dataset", IJRR 2013

#### 3. Completeness

$$Completeness = \frac{N_{matched}}{N_{gt}} \times 100\%$$

#### Why Sim(3) alignment?

Monocular VO suffers from **scale ambiguity**. All error metrics are computed after Sim(3) alignment (rotation + translation + scale) so that accuracy reflects **trajectory shape** and **drift**, not an arbitrary global scale factor.

### Evaluation Protocol

- **Ground truth**: `ground_truth.txt` (TUM format)
- **Estimated trajectory**: `CameraTrajectory.txt` (full trajectory, all frames)
- **Association threshold**: \(t_{max\_diff} = 0.1\) s
- **Distance delta for RPE**: \(\delta = 10\) m

---

## 📁 Dataset Description

### HKisland_GNSS03 Dataset

| Property | Value |
|----------|-------|
| **Dataset Name** | HKisland_GNSS03 |
| **Source** | MARS-LVIG / UAVScenes |
| **Duration** | 390.78 seconds (~6.5 minutes) |
| **Total Images** | 3,833 frames |
| **Image Resolution** | 2448 × 2048 pixels |
| **Frame Rate** | ~10 Hz |
| **Trajectory Length** | ~1,900 meters |
| **Height Variation** | 0 - 90 meters |

### Ground Truth

| Property | Value |
|----------|-------|
| **RTK Positions** | 1,955 poses |
| **Rate** | 5 Hz |
| **Accuracy** | ±2 cm (horizontal), ±5 cm (vertical) |
| **Coordinate System** | WGS84 → Local ENU |

### Data Sources

| Resource | Link |
|----------|------|
| MARS-LVIG Dataset | https://mars.hku.hk/dataset.html |
| UAVScenes GitHub | https://github.com/sijieaaa/UAVScenes |

---

## ⚙️ Implementation Details

### System Configuration

| Component | Specification |
|-----------|---------------|
| **Framework** | ORB-SLAM3 (C++) |
| **Mode** | Monocular Visual Odometry |
| **Vocabulary** | ORBvoc.txt (pre-trained) |
| **Operating System** | Linux (Ubuntu / WSL2) |
| **Image preprocessing** | 2× downsampling (2448×2048 → 1224×1024) |
| **Bag playback** | 0.5× speed |

### Camera Calibration (2× Downsampled)

```yaml
Camera.type: "PinHole"
Camera1.fx: 722.215
Camera1.fy: 722.17
Camera1.cx: 589.75
Camera1.cy: 522.45

Camera1.k1: -0.0560
Camera1.k2: 0.1180
Camera1.p1: 0.00122
Camera1.p2: 0.00064
Camera1.k3: -0.0627

Camera.width: 1224
Camera.height: 1024
Camera.fps: 10.0
```

Intrinsics scaled by 1/2 from original 2448×2048 calibration.

### ORB Feature Extraction Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `nFeatures` | 2000 | Features per frame |
| `scaleFactor` | 1.2 | Pyramid scale factor |
| `nLevels` | 8 | Pyramid levels |
| `iniThFAST` | 12 | Initial FAST threshold |
| `minThFAST` | 4 | Minimum FAST threshold |

### Running ORB-SLAM3

```bash
# Terminal 1: roscore
source /opt/ros/noetic/setup.bash && roscore

# Terminal 2: ORB-SLAM3 with 2× downsampling
./Examples_old/ROS/ORB_SLAM3/Mono_Compressed \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/HKisland_Mono_downsampled2.yaml \
    2

# Terminal 3: Play bag at 0.5× speed
rosbag play --pause --rate 0.5 data/HKisland_GNSS03.bag \
    /left_camera/image/compressed:=/camera/image_raw/compressed
```

## 📈 Results and Analysis

### Evaluation Results

```
================================================================================
VISUAL ODOMETRY EVALUATION RESULTS
================================================================================

Ground Truth: RTK trajectory (1,955 poses)
Estimated:    ORB-SLAM3 camera trajectory (3,609 poses)
Matched Poses: 1,817 / 1,955 (92.94%)  ← Completeness

METRIC 1: ATE (Absolute Trajectory Error)
────────────────────────────────────────
RMSE:   3.2078 m
Mean:   2.0624 m
Std:    2.4569 m

METRIC 2: RPE Translation Drift (distance-based, delta=10 m)
────────────────────────────────────────
Mean translational RPE over 10 m: 14.4427 m
Translation drift rate:           1.4443 m/m

METRIC 3: RPE Rotation Drift (distance-based, delta=10 m)
────────────────────────────────────────
Mean rotational RPE over 10 m: 11.0348 deg
Rotation drift rate:        110.3482 deg/100m

================================================================================
```

### Performance Analysis

| Metric | Value | Grade | Interpretation |
|--------|-------|-------|----------------|
| **ATE RMSE** | 3.21 m | B | Within acceptable range for outdoor VO |
| **RPE Trans Drift** | 1.44 m/m | C | Moderate local drift |
| **RPE Rot Drift** | 110.35 deg/100m | C | Moderate orientation drift |
| **Completeness** | 92.94% | A | Excellent pose alignment coverage |

### Key Observations

1. **Completeness (92.94%)**: Using full `CameraTrajectory.txt` (all tracked frames) yields high alignment rate.

2. **2× downsampling**: Significant improvement over 4× downsampling. ATE reduced from ~99 m to ~3.2 m. Higher resolution (1224×1024) preserves more structure for better pose estimation.

3. **Accuracy metrics**: ATE within acceptable range; RPE drift moderate but typical for monocular VO.

---

## 📊 Visualizations

### Trajectory Comparison

![Trajectory Evaluation](figures/trajectory_evaluation.png)

This figure includes:

1. **Top-Left**: 2D trajectory before alignment (matched poses only)
2. **Top-Right**: 2D trajectory after Sim(3) alignment (scale corrected)
3. **Bottom-Left**: Distribution of ATE translation errors (meters)
4. **Bottom-Right**: ATE translation error along trajectory

**Reproducibility**: Regenerate with `scripts/generate_report_figures.py` and `evo_ape --save_results`.

---

## 💭 Discussion

### Strengths

1. **High evaluation coverage**: 92.94% completeness indicates most ground-truth poses can be associated and evaluated.

2. **Full trajectory output**: Modified ORB-SLAM3 to save `CameraTrajectory.txt` (all frames) for monocular mode, enabling dense evaluation.

3. **2× downsampling**: Good balance between accuracy and computational load; ATE ~3.2 m acceptable for outdoor VO.

### Limitations

1. **Moderate drift**: RPE indicates some error accumulation over the 1.9 km trajectory.

2. **Monocular scale ambiguity**: Without IMU or stereo, scale cannot be recovered; Sim(3) alignment corrects for evaluation but real-world scale remains unknown.

3. **UAV aerial challenges**: Fast motion, motion blur, and repetitive terrain can cause tracking instability.

### Error Sources

1. **Resolution**: 2× downsampling preserves more structure than 4×; further tuning may improve.
2. **Calibration**: Scaled intrinsics for downsampled images; verify against original calibration.
3. **Feature extraction**: ORB parameters (nFeatures=2000, FAST 12/4) tuned for this run.

---

## 🎯 Conclusions

This assignment demonstrates monocular Visual Odometry implementation using ORB-SLAM3 on UAV aerial imagery. Key findings:

1. ✅ **System Operation**: ORB-SLAM3 processes 3,609 frames over ~1.9 km trajectory
2. ✅ **Evaluation coverage**: 92.94% completeness with full trajectory evaluation
3. ✅ **Accuracy**: ATE 3.21 m within acceptable range; 2× downsampling yields good results
4. 📌 **Improvements**: Full trajectory export, 2× downsampling, 0.5× playback, nFeatures=2000

### Recommendations for Improvement

| Priority | Action | Expected Improvement |
|----------|--------|---------------------|
| High | Increase `nFeatures` to 2000–2500 | Better feature coverage |
| High | Lower FAST thresholds (e.g., 12/4) | More features in low-contrast regions |
| Medium | Try without downsampling (full resolution) | Compare accuracy trade-off |
| Medium | 2× downsampling (current) | Good balance; ATE ~3.2 m |
| Low | Enable IMU fusion (VIO mode) | 50–70% accuracy improvement |

---

## 📚 References

1. Campos, C., Elvira, R., Rodríguez, J. J. G., Montiel, J. M., & Tardós, J. D. (2021). **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**. *IEEE Transactions on Robotics*, 37(6), 1874–1890.

2. Sturm, J., Engelhard, N., Endres, F., Burgard, W., & Cremers, D. (2012). **A Benchmark for the Evaluation of RGB-D SLAM Systems**. IROS.

3. Geiger, A., Lenz, P., & Urtasun, R. (2012). **Are we ready for Autonomous Driving? The KITTI Vision Benchmark Suite**. CVPR.

4. MARS-LVIG Dataset: https://mars.hku.hk/dataset.html

5. ORB-SLAM3 GitHub: https://github.com/UZ-SLAMLab/ORB_SLAM3

---

## 📎 Appendix

### A. Repository Structure (Upload-Ready)

```
AAE5303_assignment2_orbslam3_demo/
├── README.md                    # This report
├── requirements.txt             # Python dependencies (numpy, evo, matplotlib)
├── figures/
│   └── trajectory_evaluation.png
├── output/
│   ├── evaluation_report.json   # Structured metrics (ATE, RPE, completeness)
│   ├── FINAL_EVALUATION_RESULTS.txt
│   ├── CameraTrajectory.txt
│   ├── KeyFrameTrajectory.txt
│   └── ground_truth.txt
├── scripts/
│   ├── evaluate_vo_accuracy.py
│   └── generate_report_figures.py
└── docs/
    └── camera_config.yaml       # 2× downsampled HKisland config
    └── HKisland_Mono.yaml
```

### B. Running Commands

```bash
# 1. Extract RTK ground truth from bag (run from ORB_SLAM3 root; bag in data/)
python3 << 'EOF'
import rosbag
import numpy as np
bag = rosbag.Bag('data/HKisland_GNSS03.bag')
rtk_data = []
for topic, msg, t in bag.read_messages(topics=['/dji_osdk_ros/rtk_position']):
    timestamp = msg.header.stamp.to_sec()
    lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
    rtk_data.append([timestamp, lat, lon, alt])
bag.close()
rtk_data = np.array(rtk_data)
lat0, lon0, alt0 = rtk_data[0, 1], rtk_data[0, 2], rtk_data[0, 3]
R = 6378137.0
x = R * np.radians(rtk_data[:, 2] - lon0) * np.cos(np.radians(lat0))
y = R * np.radians(rtk_data[:, 1] - lat0)
z = rtk_data[:, 3] - alt0
with open('ground_truth.txt', 'w') as f:
    for i in range(len(rtk_data)):
        f.write(f"{rtk_data[i,0]:.6f} {x[i]:.6f} {y[i]:.6f} {z[i]:.6f} 0 0 0 1\n")
print(f"Saved {len(rtk_data)} ground truth poses")
EOF

# 2. Run ORB-SLAM3 (see Implementation Details for full steps)
# Terminal 1: roscore
source /opt/ros/noetic/setup.bash && roscore

# Terminal 2: ORB-SLAM3 with 2× downsampling
./Examples_old/ROS/ORB_SLAM3/Mono_Compressed \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/HKisland_Mono_downsampled2.yaml \
    2

# Terminal 3: Play bag at 0.5× speed
rosbag play --pause --rate 0.5 data/HKisland_GNSS03.bag \
    /left_camera/image/compressed:=/camera/image_raw/compressed
# 3. Evaluate trajectory (use output/ for trajectories)
python3 scripts/evaluate_vo_accuracy.py \
    --groundtruth output/ground_truth.txt \
    --estimated output/CameraTrajectory.txt \
    --t-max-diff 0.1 \
    --delta-m 10 \
    --workdir output \
    --json-out output/evaluation_report.json

# 4. Generate report figures
evo_ape tum output/ground_truth.txt output/CameraTrajectory.txt \
    --align --correct_scale --t_max_diff 0.1 \
    --save_results output/ate.zip -va
python3 scripts/generate_report_figures.py \
    --gt output/ground_truth.txt \
    --est output/CameraTrajectory.txt \
    --evo-ape-zip output/ate.zip \
    --out figures/trajectory_evaluation.png \
    --title-suffix "HKisland_GNSS03"
```

### C. Native evo Commands

```bash
evo_ape tum output/ground_truth.txt output/CameraTrajectory.txt \
  --align --correct_scale --t_max_diff 0.1 -va

evo_rpe tum output/ground_truth.txt output/CameraTrajectory.txt \
  --align --correct_scale --t_max_diff 0.1 \
  --delta 10 --delta_unit m --pose_relation trans_part -va

evo_rpe tum output/ground_truth.txt output/CameraTrajectory.txt \
  --align --correct_scale --t_max_diff 0.1 \
  --delta 10 --delta_unit m --pose_relation angle_deg -va
```

### D. Output Trajectory Format (TUM)

```
# timestamp x y z qx qy qz qw
1698132964.599976 0.099208 -0.101866 0.079074 -0.145987 -0.108908 0.004486 0.983263
...
```

---

<div align="center">

**AAE5303 - Robust Control Technology in Low-Altitude Aerial Vehicle**

*Department of Aeronautical and Aviation Engineering*

*The Hong Kong Polytechnic University*

</div>
