# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
