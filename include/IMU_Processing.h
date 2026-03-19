/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H

#include <Eigen/Eigen>
#include <fstream>
#include "common_lib.h"
#include <condition_variable>
#include <nav_msgs/msg/odometry.hpp>
#include <utils/so3_math.h>

const bool time_list(PointType &x,
                     PointType &y); //{return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4, 4) & T);
  void set_gyr_cov_scale(const V3D &scaler);
  void set_acc_cov_scale(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void set_inv_expo_cov(const double &inv_expo);
  void set_imu_init_frame_num(const int &num);
  void set_vibration_adaptive(bool en, double scale_acc, double scale_gyr);
  void set_init_motion_thresholds(double acc_thr, double gyr_thr, int max_retries);
  // Accessors for unit-testing
  int    get_init_retry_count() const { return init_retry_count_; }
  void   set_last_prop_end_time(double t) { last_prop_end_time = t; }
  void disable_imu();
  void disable_gravity_est();
  void disable_bias_est();
  void disable_exposure_est();
  void Process2(LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_);
  void UndistortPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);
  void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);

  ofstream fout_imu;
  double IMU_mean_acc_norm;
  V3D unbiased_gyr;
  vector<Pose6D> IMUpose;

  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double cov_inv_expo;
  double first_lidar_time;
  bool imu_time_init = false;
  bool imu_need_init = true;
  M3D Eye3d;
  V3D Zero3d;
  int lidar_type;

private:
  void Forward_without_imu(LidarMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);
  PointCloudXYZI pcl_wait_proc;
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu;
  PointCloudXYZI::Ptr cur_pcl_un_;
  M3D Lid_rot_to_IMU;
  V3D Lid_offset_to_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D var_acc_init;
  V3D var_gyr_init;
  V3D angvel_last;
  V3D acc_s_last;
  double last_prop_end_time;
  double time_last_scan;
  int init_iter_num = 1, MAX_INI_COUNT = 20;
  bool b_first_frame = true;
  bool imu_en = true;
  bool gravity_est_en = true;
  bool ba_bg_est_en = true;
  bool exposure_estimate_en = true;
  bool vibration_adaptive_en = false;
  double vibration_scale_acc = 1.0;
  double vibration_scale_gyr = 1.0;
  double ema_s_acc = 1.0;
  double ema_s_gyr = 1.0;
  V3D base_cov_acc = V3D(0.1, 0.1, 0.1);
  V3D base_cov_gyr = V3D(0.1, 0.1, 0.1);
  // IMU init motion-detection thresholds; configurable via set_init_motion_thresholds()
  double init_acc_var_threshold_ = 0.5;  // m^2/s^4 L2-norm of per-axis variance
  double init_gyr_var_threshold_ = 0.2;  // rad^2/s^2 L2-norm
  int    max_init_retries_        = 5;
  int    init_retry_count_        = 0;
};
typedef std::shared_ptr<ImuProcess> ImuProcessPtr;
#endif