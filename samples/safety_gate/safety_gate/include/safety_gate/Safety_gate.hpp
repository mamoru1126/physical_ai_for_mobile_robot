#ifndef SAFETY_GATE_HPP_
#define SAFETY_GATE_HPP_

#include <cstdint>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct RoiRange
{
  float minX;
  float maxX;
  float minY;
  float maxY;
  float minZ;
  float maxZ;
};

struct Velocity
{
  double linearX{0.0};
  double linearY{0.0};
  double linearZ{0.0};
  double angularX{0.0};
  double angularY{0.0};
  double angularZ{0.0};
};

class SafetyGate
{
public:
  /* 概要：安全機能ロジックを初期化する。
   * 入力：なし
   * 出力：なし
   */
  SafetyGate();

  /* 概要：設定ファイルから安全機能ロジックの設定値を読み込む。
   * 入力：config_file_path 設定ファイルのパス
   * 出力：なし
   */
  void loadConfig(const std::string & config_file_path);

  /* 概要：点群内の障害物有無に基づいて安全な速度指令値を生成する。
   * 入力：point_cloud LIDAR座標系のXYZ点群、input_velocity 入力速度指令値
   * 出力：障害物ありの場合は全成分0、障害物なしの場合は入力値と同じ速度指令値
   */
  Velocity generateSafeCmdVel(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & point_cloud,
    const Velocity & input_velocity) const;

private:
  /* 概要：入力点群からROI範囲内の点群を抽出する。
   * 入力：point_cloud LIDAR座標系のXYZ点群
   * 出力：ROI範囲内の点群
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr extractRoiPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & point_cloud) const;

  /* 概要：ROI内点群をVoxelGridでボクセル化する。
   * 入力：point_cloud ROI範囲内のXYZ点群
   * 出力：各占有ボクセルを代表点で表した点群
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelizePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & point_cloud) const;

  RoiRange roiRange;
  double voxelSize;
  std::uint32_t obstacleVoxelThreshold;
};

#endif
