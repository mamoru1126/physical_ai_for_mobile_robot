#include "safety_gate/Safety_gate.hpp"

#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

namespace
{
template<typename T>
void readIfPresent(const YAML::Node & node, const char * key, T & value)
{
  if (node && node[key]) {
    value = node[key].as<T>();
  }
}
}  // namespace

/* 概要：設定ファイル（YAML）から安全機能ロジックの設定値を読み込む。
 *       書いていない項目は既定の値のままにする。
 * 入力：config_file_path 設定ファイルのパス
 * 出力：なし（設定値が正しくなければ std::runtime_error を投げる）
 */
void SafetyGate::loadConfig(const std::string & config_file_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(config_file_path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("設定ファイルを読めません: " + config_file_path + ": " + e.what());
  }

  SafetyGateConfig c;
  try {
    if (root["footprint"]) {
      c.footprint.clear();
      for (const auto & vertex : root["footprint"]) {
        if (!vertex.IsSequence() || vertex.size() != 2U) {
          throw std::runtime_error("footprint の頂点は [x, y] で書いてください");
        }
        c.footprint.push_back(Point2{vertex[0].as<double>(), vertex[1].as<double>()});
      }
    }
    const YAML::Node height = root["height_range"];
    readIfPresent(height, "min_z", c.heightRange.minZ);
    readIfPresent(height, "max_z", c.heightRange.maxZ);

    const YAML::Node safety = root["safety"];
    readIfPresent(safety, "voxel_size", c.voxelSize);
    readIfPresent(safety, "obstacle_voxel_threshold", c.obstacleVoxelThreshold);
    readIfPresent(safety, "margin", c.margin);
    readIfPresent(safety, "time_horizon", c.timeHorizon);
    readIfPresent(safety, "stop_time", c.stopTime);
    readIfPresent(safety, "time_step", c.timeStep);

    const YAML::Node limits = root["limits"];
    readIfPresent(limits, "max_linear_x", c.maxLinearX);
    readIfPresent(limits, "max_linear_y", c.maxLinearY);
    readIfPresent(limits, "max_angular_z", c.maxAngularZ);

    double view_size = 2.0 * c.viewHalfSize;
    readIfPresent(root["web"], "view_size", view_size);
    c.viewHalfSize = 0.5 * view_size;
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("設定ファイルの値が読めません: " + std::string(e.what()));
  }
  setConfig(c);
}
