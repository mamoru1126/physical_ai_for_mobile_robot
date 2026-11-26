#include "safety_gate/Safety_gate.hpp"

#include <stdexcept>
#include <Eigen/Core>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>

/* 概要：安全機能ロジックを初期化する。
 * 入力：なし
 * 出力：なし
 */
SafetyGate::SafetyGate()
: roiRange{-1.0, 0.4, -0.7, 0.7, -0.2, 0.8}, voxelSize(0.05), obstacleVoxelThreshold(3)
{
}

/* 概要：設定ファイルから安全機能ロジックの設定値を読み込む。
 * 入力：config_file_path 設定ファイルのパス
 * 出力：なし
 */
void SafetyGate::loadConfig(const std::string & config_file_path)
{
  const YAML::Node config = YAML::LoadFile(config_file_path);
  const YAML::Node roi = config["roi_range"];
  if (!roi || !config["voxel_size"] || !config["obstacle_voxel_threshold"]) {
    throw std::runtime_error("Required safety gate setting is missing");
  }

  roiRange.minX = roi["min_x"].as<float>();
  roiRange.maxX = roi["max_x"].as<float>();
  roiRange.minY = roi["min_y"].as<float>();
  roiRange.maxY = roi["max_y"].as<float>();
  roiRange.minZ = roi["min_z"].as<float>();
  roiRange.maxZ = roi["max_z"].as<float>();
  voxelSize = config["voxel_size"].as<float>();
  obstacleVoxelThreshold = config["obstacle_voxel_threshold"].as<std::uint32_t>();

  if (roiRange.minX > roiRange.maxX || roiRange.minY > roiRange.maxY ||
    roiRange.minZ > roiRange.maxZ) {
    throw std::runtime_error("ROI minimum value must not exceed maximum value");
  }
  if (voxelSize <= 0.0) {
    throw std::runtime_error("voxel_size must be greater than zero");
  }
  if (obstacleVoxelThreshold == 0U) {
    throw std::runtime_error("obstacle_voxel_threshold must be greater than zero");
  }
}

/* 概要：点群内の障害物有無に基づいて安全な速度指令値を生成する。
 * 入力：point_cloud LIDAR座標系のXYZ点群、input_velocity 入力速度指令値
 * 出力：障害物ありの場合は全成分0、障害物なしの場合は入力値と同じ速度指令値
 */
Velocity SafetyGate::generateSafeCmdVel(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & point_cloud,
  const Velocity & input_velocity) const
{
  if (!point_cloud) {
    return Velocity{};
  }
  const auto roi_point_cloud = extractRoiPointCloud(point_cloud);
  const auto voxelized_point_cloud = voxelizePointCloud(roi_point_cloud);
  const auto occupied_voxel_count = static_cast<std::uint32_t>(voxelized_point_cloud->size());
  return occupied_voxel_count >= obstacleVoxelThreshold ? Velocity{} : input_velocity;
}

/* 概要：入力点群からROI範囲内の点群を抽出する。
 * 入力：point_cloud LIDAR座標系のXYZ点群
 * 出力：ROI範囲内の点群
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr SafetyGate::extractRoiPointCloud(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & point_cloud) const
{
  auto roi_point_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::CropBox<pcl::PointXYZ> crop_box;
  crop_box.setInputCloud(point_cloud);
  crop_box.setMin(Eigen::Vector4f(
    roiRange.minX,
    roiRange.minY,
    roiRange.minZ,
    1.0F));
  crop_box.setMax(Eigen::Vector4f(
    roiRange.maxX,
    roiRange.maxY,
    roiRange.maxZ,
    1.0F));
  crop_box.filter(*roi_point_cloud);
  return roi_point_cloud;
}

/* 概要：ROI内点群をVoxelGridでボクセル化する。
 * 入力：point_cloud ROI範囲内のXYZ点群
 * 出力：各占有ボクセルを代表点で表した点群
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr SafetyGate::voxelizePointCloud(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & point_cloud) const
{
  auto voxelized_point_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (point_cloud->empty()) {
    return voxelized_point_cloud;
  }
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(point_cloud);
  voxel_grid.setLeafSize(
    voxelSize,
    voxelSize,
    voxelSize);
  voxel_grid.filter(*voxelized_point_cloud);
  return voxelized_point_cloud;
}
