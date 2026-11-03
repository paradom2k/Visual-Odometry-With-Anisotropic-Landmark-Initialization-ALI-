# ALI-VINS

**ALI-VINS** 采用 OpenVINS 的 KLT 跟踪和 MSCKF + SLAM 估计设计，但检测器使用新的 GFTT（block-wise Shi--Tomasi），并在 landmark 初始化阶段使用 ALI（Anisotropic Landmark Initialization）。项目保留轻量的 Win32 UI，形成一个独立的视觉惯性里程计实现。

ALI-VINS 是一个独立实现，不是 OpenVINS 的分支或移植；它复用 OpenVINS 的 KLT 跟踪与 MSCKF/SLAM 设计，并替换了检测器（GFTT）和 landmark 初始化策略（ALI）。

## 目录

```
ALI-VINS/
  CMakeLists.txt          CMake 工程
  build_msvc.bat          使用 E:\jo_gvil\visual studio 的 MSVC 和 E:\jo_gvil\cmake 构建
  cmake/openvins.cmake    OpenVINS 源码子集、依赖路径和链接项
  src/
    app_main.cpp          原有 Win32 UI / --list / --run / --plot / --compare
    ali_vins_run.cpp      driver：manifest + IMU -> VioManager -> 轨迹
  thirdparty/openvins/    ov_core + ov_init + ov_msckf（无 ROS，无 ArUco）
  config/
    4seasons_neighborhood_3.conf
    4seasons_office_loop_5.conf
    openvins_4seasons.yaml / _calib.yaml / _imu.yaml
    openvins_4seasons_office.yaml / _office_calib.yaml
    data/                 两个序列的 manifest 与 reference
  tools/
    gauge_free_score.py   UI 使用的 ATE 打分器
    bag_extract.py, bag_inspect.py, make_manifests.py
  bin/                    构建产物和运行期 DLL
  sol/                    轨迹、日志、score
```

## 构建

```bat
build_msvc.bat
```

默认使用：

| 依赖 | 路径 |
|---|---|
| CMake | `E:\jo_gvil\cmake\bin\cmake.exe` |
| Ninja | `E:\jo_gvil\visual studio\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe` |
| MSVC | `E:\jo_gvil\visual studio\VC\Tools\MSVC\14.44.35207` |
| Windows SDK | `D:\Windows Kits\10\10.0.26100.0` |
| OpenCV | `E:/jo_gvil/jo_gvil 1.0/build/opencv-install` (4.6.0) |
| Eigen | `E:/jo_gvil/jo_gvil 1.0/github_prj/GraphRTK-INS-main/src/Third-party/Eigen` (3.3.5) |
| Ceres | `E:/jo_gvil/jo_gvil 1.0/github_prj/GraphRTK-INS-main/src/Third-party/ceres` (1.14.0) |
| Boost | `E:/jo_gvil/jo_gvil 1.0/github_prj/20261011/boost_1_92_0/boost_1_92_0` (1.92.0) |

四个依赖根都可以通过参数覆盖：`build_msvc.bat [OpenCV] [Eigen] [Ceres] [Boost] [jobs]`。

## 运行

GUI：

```bat
bin\ali_vins_app.exe
```

Headless：

```bat
bin\ali_vins_app.exe --list
bin\ali_vins_app.exe --run 4seasons_neighborhood_3
bin\ali_vins_app.exe --run 4seasons_office_loop_5
```

`--frames N` 可以覆盖配置里的帧数；`--no-score` 跳过打分。

## ALI 与 MSCKF/SLAM 设计

- 实验配置都有 `max_slam: 50`、`max_slam_in_update: 25`、`dt_slam_delay: 1`、`feat_rep_slam: ANCHORED_MSCKF_INVERSE_DEPTH`。这个 OpenVINS 版本没有 `use_slam` 开关，`max_slam > 0` 就是 SLAM 分支的开关；MSCKF 仍然同时运行。
- ALI 由 driver 默认设置 `OV_WEIGHT=l2cov` 开启。要跑无加权 baseline，在启动前设置空值：

```bat
set OV_WEIGHT=
bin\ali_vins_run.exe ...
```

- ALI 的唯一实现改动是在 `ov_msckf/src/update/UpdaterSLAM.cpp` 的 `delayed_init`：把整块 `R` 的稠密 LLT 改成按 2×2 测量块逐块白化。`R` 本来就是块对角的，数学等价；1000 帧上与原实现的最大位置差为 `1.2e-6 m`，但 2000 帧从 ~240 s 降到 ~86 s。

## 特征提取器

默认检测器已从 OpenVINS 的 per-cell `cv::FAST` 换成 **IC-GVINS 的 block-wise Shi-Tomasi (GFTT)**：

- `thirdparty/openvins/ov_core/src/track/Grider_GRID.h` 中每个网格单元调用 `cv::goodFeaturesToTrack`，`qualityLevel = 0.01`，`minDistance = min_px_dist`（当前配置 15），然后继续使用原有的亚像素细化。
- OpenVINS 的 mask 语义是 255=禁止检测，而 GFTT 是 255=有效区域，所以代码对 ROI mask 做了 `bitwise_not`。
- KLT 跟踪和网格布局沿用 OpenVINS；检测器改为 GFTT；ALI 用于 landmark 初始化。每帧最大特征数由 `.conf` 的 `max_features` 控制（见下一节）。

默认 GFTT；要回到原 FAST 做 A/B，设置：

```bat
set OV_DETECTOR=fast
```

## 每帧最大特征数

`.conf` 新增 `max_features`。UI 解析后通过 `--max-features` 传给 driver；driver 只覆盖 OpenVINS 的 `num_pts`（初始化完成后的每帧上限），`init_max_features` 仍保持 YAML 的 200，避免把动态初始化行为一起改掉。当前两个 conf 已设为 `max_features = 600`。

GFTT 下 neighborhood_3 前 2000 帧的上限筛选：

| max_features | 300 | 500 | 600 | 700 | 800 |
|---|---:|---:|---:|---:|---:|
| RMS 3D | 17.25 m | 7.22 m | **6.89 m** | 9.10 m | 15.26 m |

600 最好；500–600 是平台区，700/800 反而变差。实际跟踪数也随上限增长：300→223、500→348、600→434、800→486 条（第 2000 帧）。

## 验证

ALI 默认开启，`gauge_free_score.py` 做 yaw + translation（scale 固定 1）对齐后的 RMS 3D。下表同时列出原 FAST（`OV_DETECTOR=fast`）和新默认 GFTT，便于直接看这次替换的收益/代价：

| 序列 / 窗口 | 检测器 / 上限 | 帧数 | 初始化 | RMS 3D | path ratio |
|---|---|---:|---|---:|---:|
| neighborhood_3 全量 | FAST / 300 | 9487 | frame 278 | **13.3020 m** | 0.992 |
| neighborhood_3 全量 | GFTT / 300 | 9487 | frame 265 | 29.5353 m | 1.007 |
| neighborhood_3 全量 | GFTT / 600 | 9487 | frame 265 | **13.7226 m** | 1.006 |
| neighborhood_3 前 1000 帧 | FAST / 300 | 1000 | frame 278 | 4.8030 m | 0.914 |
| neighborhood_3 前 1000 帧 | GFTT / 300 | 1000 | frame 265 | **2.0883 m** | 0.964 |
| office_loop_5 6000 帧（论文窗口） | FAST / 300 | 6000 | frame 165 | 21.8369 m | 1.035 |
| office_loop_5 6000 帧（论文窗口） | GFTT / 300 | 6000 | frame 164 | 28.4095 m | 0.969 |
| office_loop_5 6000 帧（论文窗口） | GFTT / 600 | 6000 | frame 164 | **16.1811 m** | 1.058 |
| office_loop_5 全量 | FAST / 300 | 13746 | frame 165 | 69.5910 m | 1.029 |
| office_loop_5 全量 baseline（无 ALI） | FAST / 300 | 13746 | frame 165 | 66.1827 m | 1.040 |

结论：

- **把 GFTT 的上限从 300 提到 600 是明显有效的**：neighborhood_3 全量 29.5353 → **13.7226 m**，office_loop_5 6000 帧 28.4095 → **16.1811 m**。上限 600 时 GFTT 已经接近甚至超过 FAST 在这个窗口的结果（office 16.18 vs FAST 21.84；neighborhood 13.72 vs FAST 13.30）。
- 上限不是越高越好：2000 帧筛选里 600 最好，700 回到 9.10 m，800 回到 15.26 m。实际跟踪数在 800 时约 486 条，已经接近图像上 GFTT 的可用容量，继续加只会引入更多弱 Track。
- 这里只覆盖 `num_pts`，初始化仍用 YAML 的 `init_max_features: 200`。试过把 `init_max_features` 一起覆盖时（300/300），2000 帧 neighborhood_3 退化到 ratio 1.267（RMS 36.95），所以两个概念不要混在一起。
- `max_features` 已在两个 conf 中设为 600；想回到原上限改这一行即可。GFTT 600 的结果文件在本地 `sol/`（不随仓库提交）：`sol/4seasons_neighborhood_3_20261103_160647.*`、`sol/4seasons_office_loop_5_20261103_162315.*`；GFTT 300 对照：`sol/4seasons_neighborhood_3_20261103_151452.*`、`sol/4seasons_office_loop_5_20261103_152219.*`。

## 数据集路径

- neighborhood_3: 影像复用 `E:/jo_gvil/jo_gvil 1.0/build/inputs/fs_full/frames`（9487 帧），IMU 为 `.../fs_full/imu.csv`。
- office_loop_5: 影像在 `E:/driving dataset/COMPUTER VISION GROUP/recording_2021-01-07_12-04-03/undistorted_images/cam0`，IMU 复用 `E:/jo_gvil/recover_jo/e1/out/b/4seasons_office/imu.csv`。
- manifest 和 reference 已放在 `config/data/`；manifest 里的影像路径是绝对路径，IMU 路径在 conf 里也是绝对路径，因此工程本身不复制大文件。

## 说明

- 这是 ALI-VINS 的最小实现，复用了 ROS-free 的 `ov_core + ov_init + ov_msckf`；未引入 ROS、PCL、Pangolin、g2o、Sophus。
- `CpiV2.cpp` 未编译：动态初始化只用 `CpiV1`，保留它只会多一个非常慢的编译单元。
- OpenVINS 上游是 GPLv3；对外分发时需保留 `thirdparty/openvins/LICENSE`。
