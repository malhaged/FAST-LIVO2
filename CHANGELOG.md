# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased] - 2026-03-19

### Added
- **Global-LVBA Export Mode:** Body-frame PCD and image export for post-processing with [Global-LVBA](https://github.com/xuankuzcr/Global-LVBA).
  - `pcd_save.type`: 0 = world frame (default), 1 = body frame (required for Global-LVBA).
  - `image_save.img_save_en` / `image_save.interval`: Save undistorted camera images per VIO frame.
  - Per-frame pose files in TUM format: `Log/pcd/lidar_poses.txt`, `Log/image/image_poses.txt`.
- **Ouster OS1-128 Support:** Updated point struct registration in `preprocess.h` to match the current `ouster-ros` driver format (`uint16_t ring`, `signal`, `near_ir`; removed `intensity` and `ambient`).
- **ARM Build Flags:** `CMakeLists.txt` now detects CPU architecture and applies optimized compiler flags:
  - aarch64 (Jetson Orin NX, RK3588): `-O3 -mcpu=native -mtune=native -ffast-math`
  - ARM 32-bit: adds `-mfpu=neon`
  - x86-64: `-O3 -march=native -mtune=native -funroll-loops`
- **2026 AGT Aussen Dataset Config:** `config/2026_agt_aussen.yaml` and `config/camera_2026_agt_aussen.yaml` for Ouster OS1-128 + camera rig with iKalibr calibration.
- **IMU Init Retry Logic:** Extended motion-detection during IMU initialization with configurable retry count (`max_init_retries_`, default 5). After max retries, initialization proceeds regardless to avoid infinite init loops in dynamic environments.
- **Unit Test Infrastructure:** GTest targets `test_imu_processing` and `test_voxel_map` added to `CMakeLists.txt` (test sources excluded from the repo via `.gitignore`).

### Changed
- **IMU Init Motion Detection:** Welford's online algorithm now tracks per-axis variance throughout the full initialization window (previously only described, now fully implemented with retry loop).
- **README:** Added complete "About This Fork" section documenting bug fixes, new features, supported datasets, and build instructions.
- **`.gitignore`:** Added `docs/` and `test/` to prevent internal planning documents and development tests from being committed.

## [Unreleased] - 2026-03-06

### Added
- **Botanical Garden Support:** New configuration files for the Botanic Garden dataset.
  - `config/botanical.yaml`: Dataset-specific parameters (Xsens IMU, Livox Avia).
  - `config/camera_botanical.yaml`: Dalsa RGB0 (left color camera) calibration.
- **Robust Gravity Alignment:**
  - Added variance-based static detection during IMU initialization to ensure stable gravity estimation.
  - Extended initialization window via `imu_int_frame` parameter (set to 100 for Xsens).
  - Re-triggerable gravity alignment after IMU resets via `reset_imu` flag.
- **Voxel Map Memory Optimization:**
  - Implemented `points_clear_threshold` in `VoxelOctoTree` to clear point buffers once a plane is stabilized, reducing RAM for large maps.
  - Configurable `map_sliding_en` / `half_map_size` / `sliding_thresh` in `botanical.yaml`.
- **PCD Export Improvements:**
  - Added `save_raw_points` parameter to optionally save the unfiltered raw point cloud in addition to the downsampled one (default: `false`).
  - Switched `savePCD()` from `pcl::VoxelGrid` to `pcl::ApproximateVoxelGrid` (hash-based) to handle high-resolution (0.15 m) filtering of large maps without 32-bit PCL integer-index overflow.
  - Added `incremental_pcd_save_en` flag (currently disabled; accumulation and shutdown-filter is the active path).
- **Vibration-Adaptive IMU Noise (disabled):**
  - Added `vibration_adaptive_en`, `vibration_scale_acc`, `vibration_scale_gyr` parameters to `IMU_Processing` for dynamic covariance scaling.
  - Disabled (`vibration_adaptive_en: false`) in `botanical.yaml` — untested on this dataset, kept as opt-in.
- **Shutdown Grace Period:** `mapping_avia.launch.py` now sets `sigterm_timeout='120'` and `sigkill_timeout='60'` so the process has sufficient time to write PCD and COLMAP output on Ctrl+C.

### Changed
- **Launch Configuration:** Default launch file `mapping_avia.launch.py` now points to `botanical.yaml` and `camera_botanical.yaml`.
- **Initialization Sequence:** `LIVMapper` now initializes before `image_transport::ImageTransport` in `main.cpp` to ensure a valid node handle exists, preventing crashes during shutdown.
- **Data Handling:**
  - Switched from `cv_bridge::toCvShare` to `cv_bridge::toCvCopy` in `getImageFromMsg` to fix use-after-free with multiple simultaneous camera topics.
  - Replaced `std::endl` with `'\n'` in the COLMAP `points3D.txt` write loop — avoids per-line stream flush, preventing SIGKILL on large maps.

### Fixed
- **Mapping Robustness (reverted three Gemini regressions):**
  - **Removed `getInterpolatedPose`** from `LIVMapper` and the associated modified `processFrame`/`updateFrameState` signatures in `vio.h`/`vio.cpp`. The IMU-pose lookup was non-deterministic (only populated on LIO frames), giving VIO a randomly wrong initial guess → photometric alignment divergence → EKF drift.
  - **Removed EKF abort-guard** (`rot_add.norm() > 0.5`) from `voxel_map.cpp::StateEstimation`. The guard prevented LIO from correcting drift once it started, turning small errors into unrecoverable failures.
  - **Disabled `vibration_adaptive_en`** in `botanical.yaml`. The dynamic covariance scaling (up to 3×) was untested on this dataset and amplified the instability from the two items above.
- **Stability & Robustness:**
  - **IMU Timestamp Jumps:** `imu_cbk` now resets buffers and recovers when timestamp jumps > 0.2 s occur.
  - **VIO Projection Safety:** Null-pointer guard for `VisualPoint` retrieval and bounds check for map-point projection in `vio.cpp` prevent SIGSEGV during EKF state degeneration.
- **Node Lifecycle:** `LIVMapper` constructor now writes the internal `rclcpp::Node` back to the caller's `shared_ptr`, fixing subscriber initialization after the node is created.
