# ORB_SLAM3 完整运行与评估指南

参考 [README.md](README.md)、[HOW_TO_EVALUATE.md](HOW_TO_EVALUATE.md) 以及 [AAE5303_assignment2_orbslam3_demo-](https://github.com/XitingChen-Chloe/AAE5303_assignment2_orbslam3_demo-)，本指南提供从运行到评估的完整流程。

---

## 一、整体流程概览

```
┌─────────────────────┐    ┌─────────────────────┐    ┌─────────────────────┐
│ 1. 运行 ORB_SLAM3   │───▶│ 2. 提取 RTK 真值     │───▶│ 3. 轨迹评估 (evo)   │
│ (ROS + rosbag)      │    │ (ground_truth.txt)   │    │ + 生成分析图表      │
└─────────────────────┘    └─────────────────────┘    └─────────────────────┘
         │                            │                            │
         ▼                            ▼                            ▼
  KeyFrameTrajectory.txt      ground_truth.txt          metrics.json + 图表
```

---

## 二、Step 1：运行 ORB_SLAM3 获取轨迹

### 2.1 使用 HKisland 数据集（ROS bag）

**终端 1 - 启动 roscore**
```bash
source /opt/ros/noetic/setup.bash
roscore
```

**终端 2 - 运行 ORB_SLAM3 Mono_Compressed**
```bash
source /opt/ros/noetic/setup.bash
cd /root/ORB_SLAM3
./Examples_old/ROS/ORB_SLAM3/Mono_Compressed \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/HKisland_Mono.yaml
```

**终端 3 - 播放 bag**
```bash
source /opt/ros/noetic/setup.bash
cd /root/ORB_SLAM3
rosbag play --pause data/HKisland_GNSS03.bag \
    /left_camera/image/compressed:=/camera/image_raw/compressed
```

- 等待 ORB_SLAM3 显示 "ORB-SLAM3 is ready" 后，在终端 3 按**空格键**开始播放
- 程序结束后，在**运行目录**（`/root/ORB_SLAM3`）生成 `KeyFrameTrajectory.txt`

### 2.2 可选：同时保存完整相机轨迹（提高评估完整性）

当前 Mono_Compressed 节点只保存 `KeyFrameTrajectory.txt`（关键帧）。若需 `CameraTrajectory.txt`（所有跟踪帧，评估完整性更高），可在 `Examples_old/ROS/ORB_SLAM3/src/ros_mono_compressed.cc` 第 59 行前添加：

```cpp
SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
```

然后重新编译 ROS 节点。

---

## 三、Step 2：从 ROS Bag 提取 RTK 真值

在项目根目录执行：

```bash
cd /root/ORB_SLAM3
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

# Convert to local ENU coordinates
lat0, lon0, alt0 = rtk_data[0, 1], rtk_data[0, 2], rtk_data[0, 3]
R = 6378137.0

x = R * np.radians(rtk_data[:, 2] - lon0) * np.cos(np.radians(lat0))
y = R * np.radians(rtk_data[:, 1] - lat0)
z = rtk_data[:, 3] - alt0

# Save in TUM format
with open('ground_truth.txt', 'w') as f:
    for i in range(len(rtk_data)):
        f.write(f"{rtk_data[i,0]:.6f} {x[i]:.6f} {y[i]:.6f} {z[i]:.6f} 0 0 0 1\n")

print(f"Saved {len(rtk_data)} ground truth poses")
EOF
```

若 bag 中 RTK 话题名称不同，可先查看：
```bash
rosbag info data/HKisland_GNSS03.bag
```

---

## 四、Step 3：轨迹评估与分析

### 4.1 安装 evo

```bash
pip3 install evo --upgrade
```

### 4.2 方法 A：直接使用 evo 命令

**ATE（绝对轨迹误差，Sim3 对齐 + 尺度校正）**
```bash
cd /root/ORB_SLAM3
evo_ape tum ground_truth.txt KeyFrameTrajectory.txt -va --align --correct_scale --t_max_diff 0.1
```

**RPE（相对位姿误差）**
```bash
evo_rpe tum ground_truth.txt KeyFrameTrajectory.txt -va --align --correct_scale --t_max_diff 0.1
```

**生成 3D 轨迹对比图**
```bash
evo_ape tum ground_truth.txt KeyFrameTrajectory.txt -va --align --correct_scale --t_max_diff 0.1 --plot --plot_mode xyz
```

### 4.3 方法 B：使用 AAE5303 评估脚本（推荐，输出更完整）

从 [AAE5303_assignment2_orbslam3_demo-](https://github.com/XitingChen-Chloe/AAE5303_assignment2_orbslam3_demo-/tree/main/scripts) 下载或创建 `scripts/evaluate_vo_accuracy.py`，然后执行：

```bash
cd /root/ORB_SLAM3
python3 scripts/evaluate_vo_accuracy.py \
    --groundtruth ground_truth.txt \
    --estimated KeyFrameTrajectory.txt \
    --t-max-diff 0.1 \
    --delta-m 10 \
    --workdir evaluation_results \
    --json-out evaluation_results/metrics.json
```

**输出指标说明：**
| 指标 | 含义 |
|------|------|
| **ATE RMSE** | 全局轨迹误差（米），Sim3 对齐后 |
| **RPE trans drift (m/m)** | 平移漂移率（每米误差） |
| **RPE rot drift (deg/100m)** | 旋转漂移率（每 100 米角度误差） |
| **Completeness (%)** | 成功匹配的位姿占比 |

### 4.4 生成报告图表

使用 [generate_report_figures.py](https://github.com/XitingChen-Chloe/AAE5303_assignment2_orbslam3_demo-/blob/main/scripts/generate_report_figures.py)：

```bash
# 先运行 evo_ape 并保存结果
evo_ape tum ground_truth.txt KeyFrameTrajectory.txt \
    --align --correct_scale --t_max_diff 0.1 \
    --save_results evaluation_results/ate.zip -va

# 生成 2x2 报告图
python3 scripts/generate_report_figures.py \
    --gt ground_truth.txt \
    --est KeyFrameTrajectory.txt \
    --evo-ape-zip evaluation_results/ate.zip \
    --out figures/trajectory_evaluation.png \
    --title-suffix "HKisland_GNSS03"
```

图表包含：对齐前后 2D 轨迹、ATE 误差分布直方图、ATE 沿轨迹变化。

---

## 五、文件结构

```
/root/ORB_SLAM3/
├── KeyFrameTrajectory.txt      # ORB_SLAM3 输出（关键帧轨迹）
├── CameraTrajectory.txt        # 可选：完整相机轨迹（若修改了源码）
├── ground_truth.txt            # RTK 真值（TUM 格式）
├── evaluation_results/        # evo 评估结果
│   ├── ate.zip
│   ├── rpe_trans.zip
│   ├── rpe_rot.zip
│   └── metrics.json
├── figures/
│   └── trajectory_evaluation.png
└── scripts/
    ├── evaluate_vo_accuracy.py
    └── generate_report_figures.py
```

---

## 六、TUM 格式说明

每行：`timestamp tx ty tz qx qy qz qw`
- `timestamp`：秒（浮点）
- `tx ty tz`：位置（米）
- `qx qy qz qw`：四元数姿态

---

## 七、常见问题

| 问题 | 解决 |
|------|------|
| 跟踪丢失（LOST） | 慢速播放 `--rate 0.5`，或调低 `ORBextractor.iniThFAST` |
| 匹配率低 | 增大 `--t-max-diff`（如 0.2），或使用 `CameraTrajectory.txt` |
| evo 报错 | 确认时间戳为秒，且两文件格式正确 |
| 无 RTK 话题 | 用 `rosbag info` 检查话题名，修改提取脚本中的 topic |

---

## 八、参考链接

- [ORB_SLAM3 官方仓库](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [AAE5303 作业 Demo](https://github.com/XitingChen-Chloe/AAE5303_assignment2_orbslam3_demo-)
- [evo 工具文档](https://github.com/MichaelGrupp/evo)
