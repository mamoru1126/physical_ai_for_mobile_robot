#include "safety_gate/Safety_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace
{
constexpr double kStillEpsilon = 1e-6;

/* ボクセルの番号（x、y、z を 21 ビットずつ詰める） */
std::int64_t packKey(std::int64_t ix, std::int64_t iy, std::int64_t iz)
{
  constexpr std::int64_t kOffset = 1 << 20;
  constexpr std::int64_t kMask = (1 << 21) - 1;
  return (((ix + kOffset) & kMask) << 42) | (((iy + kOffset) & kMask) << 21) |
         ((iz + kOffset) & kMask);
}

std::int64_t unpack(std::int64_t key, int shift)
{
  constexpr std::int64_t kOffset = 1 << 20;
  constexpr std::int64_t kMask = (1 << 21) - 1;
  return ((key >> shift) & kMask) - kOffset;
}

bool isFinite(const Velocity & v)
{
  return std::isfinite(v.linearX) && std::isfinite(v.linearY) && std::isfinite(v.linearZ) &&
         std::isfinite(v.angularX) && std::isfinite(v.angularY) && std::isfinite(v.angularZ);
}

/* 点に最も近い足形の部分. 辺の番号（辺 i は頂点 i と i+1 を結ぶ）. 頂点に最も近いときは、その頂点の両側の辺 */
struct Feature
{
  double distance{0.0};
  std::size_t edgeA{0};
  std::size_t edgeB{0};
};

Feature closestFeature(const Point2 & p, const std::vector<Point2> & polygon)
{
  Feature feature;
  bool inside = false;
  double min_d2 = std::numeric_limits<double>::infinity();
  const std::size_t n = polygon.size();
  for (std::size_t i = 0; i < n; ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % n];
    if ((a.y > p.y) != (b.y > p.y)) {
      const double x_cross = (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x;
      if (p.x < x_cross) {
        inside = !inside;
      }
    }
    const double ex = b.x - a.x;
    const double ey = b.y - a.y;
    const double len2 = ex * ex + ey * ey;
    double t = len2 > 0.0 ? ((p.x - a.x) * ex + (p.y - a.y) * ey) / len2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    const double dx = a.x + t * ex - p.x;
    const double dy = a.y + t * ey - p.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < min_d2) {
      min_d2 = d2;
      feature.edgeA = i;
      feature.edgeB = i;
      if (t <= 0.0) {
        feature.edgeB = (i + n - 1) % n;
      } else if (t >= 1.0) {
        feature.edgeB = (i + 1) % n;
      }
    }
  }
  const double d = std::sqrt(min_d2);
  feature.distance = inside ? -d : d;
  return feature;
}

Velocity scaled(const Velocity & v, double s)
{
  Velocity out;
  out.linearX = v.linearX * s;
  out.linearY = v.linearY * s;
  out.angularZ = v.angularZ * s;
  return out;
}
}  // namespace

const char * toString(GateState state)
{
  switch (state) {
    case GateState::Pass: return "PASS";
    case GateState::Slow: return "SLOW";
    case GateState::Stop: return "STOP";
    case GateState::Idle: return "IDLE";
  }
  return "STOP";
}

/* 概要：安全機能ロジックを既定の設定値で初期化する。
 * 入力：なし
 * 出力：なし
 */
SafetyGate::SafetyGate()
{
  setConfig(SafetyGateConfig{});
}

/* 概要：設定値を確かめてから使う。
 * 入力：config 設定値
 * 出力：なし（設定値が正しくなければ std::runtime_error を投げる）
 */
void SafetyGate::setConfig(const SafetyGateConfig & new_config)
{
  const auto & c = new_config;
  if (c.footprint.size() < 3U) {
    throw std::runtime_error("footprint には 3 つ以上の頂点が要ります");
  }
  double area2 = 0.0;
  for (std::size_t i = 0; i < c.footprint.size(); ++i) {
    const auto & a = c.footprint[i];
    const auto & b = c.footprint[(i + 1) % c.footprint.size()];
    if (!std::isfinite(a.x) || !std::isfinite(a.y)) {
      throw std::runtime_error("footprint の頂点が数値ではありません");
    }
    area2 += a.x * b.y - b.x * a.y;
  }
  if (std::fabs(area2) < 1e-6) {
    throw std::runtime_error("footprint の面積が 0 です");
  }
  if (signedDistance(Point2{}, c.footprint) >= 0.0) {
    throw std::runtime_error("footprint はロボットの中心（base_link の原点）を囲んでください");
  }
  if (!(c.heightRange.minZ < c.heightRange.maxZ)) {
    throw std::runtime_error("height_range の min_z は max_z より小さくしてください");
  }
  if (!(c.viewHalfSize > 0.0)) {
    throw std::runtime_error("view_size は 0 より大きくしてください");
  }
  if (!(c.voxelSize > 0.0)) {
    throw std::runtime_error("voxel_size は 0 より大きくしてください");
  }
  if (c.obstacleVoxelThreshold == 0U) {
    throw std::runtime_error("obstacle_voxel_threshold は 1 以上にしてください");
  }
  if (!(c.margin >= 0.0)) {
    throw std::runtime_error("margin は 0 以上にしてください");
  }
  if (!(c.timeStep > 0.0) || !(c.timeHorizon >= c.timeStep)) {
    throw std::runtime_error("time_step は 0 より大きく、time_horizon 以下にしてください");
  }
  if (!(c.stopTime >= 0.0) || !(c.stopTime < c.timeHorizon)) {
    throw std::runtime_error("stop_time は 0 以上、time_horizon 未満にしてください");
  }
  if (!(c.maxLinearX >= 0.0) || !(c.maxLinearY >= 0.0) || !(c.maxAngularZ >= 0.0)) {
    throw std::runtime_error("速度の上限は 0 以上にしてください");
  }
  config = c;
  footprintRadius = 0.0;
  for (const auto & p : config.footprint) {
    footprintRadius = std::max(footprintRadius, std::hypot(p.x, p.y));
  }
}

const SafetyGateConfig & SafetyGate::getConfig() const
{
  return config;
}

/* 概要：base_link 座標系の点群から、障害物のボクセル（中心の点）を作る。 */
std::vector<Point3> SafetyGate::extractObstacleVoxels(const std::vector<Point3> & point_cloud) const
{
  return voxelizePointCloud(extractRoiPointCloud(point_cloud));
}

/* 概要：点群から、高さの範囲と表示の範囲に入る点だけを抽出する（数値でない点も除く）。 */
std::vector<Point3> SafetyGate::extractRoiPointCloud(const std::vector<Point3> & point_cloud) const
{
  std::vector<Point3> roi;
  roi.reserve(point_cloud.size());
  for (const auto & p : point_cloud) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    if (p.z < config.heightRange.minZ || p.z > config.heightRange.maxZ) {
      continue;
    }
    if (std::fabs(p.x) > config.viewHalfSize || std::fabs(p.y) > config.viewHalfSize) {
      continue;
    }
    roi.push_back(p);
  }
  return roi;
}

/* 概要：点群をボクセルにまとめ、各ボクセルを中心の点で表す。 */
std::vector<Point3> SafetyGate::voxelizePointCloud(const std::vector<Point3> & point_cloud) const
{
  std::unordered_set<std::int64_t> keys;
  keys.reserve(point_cloud.size());
  const double vs = config.voxelSize;
  for (const auto & p : point_cloud) {
    keys.insert(packKey(
      static_cast<std::int64_t>(std::floor(p.x / vs)),
      static_cast<std::int64_t>(std::floor(p.y / vs)),
      static_cast<std::int64_t>(std::floor(p.z / vs))));
  }
  std::vector<std::int64_t> sorted(keys.begin(), keys.end());
  std::sort(sorted.begin(), sorted.end());
  std::vector<Point3> voxels;
  voxels.reserve(sorted.size());
  for (const auto key : sorted) {
    voxels.push_back(Point3{
      static_cast<float>((static_cast<double>(unpack(key, 42)) + 0.5) * vs),
      static_cast<float>((static_cast<double>(unpack(key, 21)) + 0.5) * vs),
      static_cast<float>((static_cast<double>(unpack(key, 0)) + 0.5) * vs)});
  }
  return voxels;
}

/* 概要：速度指令値を上限で抑える。数値でない成分があれば 0 にする。 */
Velocity SafetyGate::clampVelocity(const Velocity & v) const
{
  Velocity out;
  if (!isFinite(v)) {
    return out;
  }
  out.linearX = std::clamp(v.linearX, -config.maxLinearX, config.maxLinearX);
  out.linearY = std::clamp(v.linearY, -config.maxLinearY, config.maxLinearY);
  out.angularZ = std::clamp(v.angularZ, -config.maxAngularZ, config.maxAngularZ);
  return out;
}

/* 概要：点から多角形までの符号付き距離を求める（外側なら正、内側なら負）。 */
double SafetyGate::signedDistance(const Point2 & p, const std::vector<Point2> & polygon)
{
  return closestFeature(p, polygon).distance;
}

/* 概要：速度指令値のまま t 秒動いたときのロボットの姿勢を求める（はじめの姿勢から見た値）。 */
Pose2 SafetyGate::predictPose(const Velocity & v, double t)
{
  const double yaw = v.angularZ * t;
  Pose2 pose;
  pose.yaw = yaw;
  if (std::fabs(v.angularZ) < kStillEpsilon) {
    pose.x = v.linearX * t;
    pose.y = v.linearY * t;
    return pose;
  }
  // ロボットから見た速度が一定のまま回る動き（円弧）を、そのまま積分した式
  const double s = std::sin(yaw);
  const double c = std::cos(yaw);
  pose.x = (v.linearX * s + v.linearY * (c - 1.0)) / v.angularZ;
  pose.y = (v.linearX * (1.0 - c) + v.linearY * s) / v.angularZ;
  return pose;
}

/* 概要：障害物のボクセルと速度指令値から、安全な速度指令値を生成する。 */
GateResult SafetyGate::generateSafeCmdVel(
  const std::vector<Point3> & obstacle_voxels,
  const Velocity & input_velocity) const
{
  GateResult result;
  const Velocity cmd = clampVelocity(input_velocity);
  const auto steps = static_cast<std::size_t>(std::ceil(config.timeHorizon / config.timeStep - 1e-9));
  result.predictedPoses.reserve(steps + 1);
  for (std::size_t k = 0; k <= steps; ++k) {
    result.predictedPoses.push_back(
      predictPose(cmd, std::min(static_cast<double>(k) * config.timeStep, config.timeHorizon)));
  }

  const bool still = std::fabs(cmd.linearX) < kStillEpsilon &&
    std::fabs(cmd.linearY) < kStillEpsilon && std::fabs(cmd.angularZ) < kStillEpsilon;
  if (still) {
    result.state = GateState::Idle;
    return result;
  }

  // ここより遠い点は、見る時間のうちに足形（＋余裕）へ届かない
  const double reach = footprintRadius + config.margin + config.voxelSize +
    std::hypot(cmd.linearX, cmd.linearY) * config.timeHorizon;

  // 1. 今の距離. 足形の辺ごとに、いまどこまで近くに障害物があるか（ノイズに負けないよう、
  //    近い順に obstacle_voxel_threshold 番目の距離）を求める
  struct Candidate
  {
    std::size_t index;
    Point2 point;
    double distance;
  };
  std::vector<Candidate> candidates;
  const std::size_t edges = config.footprint.size();
  std::vector<std::vector<double>> edge_distances(edges);
  for (std::size_t i = 0; i < obstacle_voxels.size(); ++i) {
    const Point2 p{obstacle_voxels[i].x, obstacle_voxels[i].y};
    if (std::hypot(p.x, p.y) > reach) {
      continue;
    }
    const Feature now = closestFeature(p, config.footprint);
    if (now.distance < 0.0) {
      continue;  // 足形の中の点は、ロボット自身などとみなして使わない
    }
    candidates.push_back(Candidate{i, p, now.distance});
    edge_distances[now.edgeA].push_back(now.distance);
    if (now.edgeB != now.edgeA) {
      edge_distances[now.edgeB].push_back(now.distance);
    }
  }
  std::vector<double> edge_clearance(edges, std::numeric_limits<double>::infinity());
  const std::size_t k = config.obstacleVoxelThreshold;
  for (std::size_t e = 0; e < edges; ++e) {
    auto & list = edge_distances[e];
    if (list.size() >= k) {
      std::nth_element(list.begin(), list.begin() + (k - 1), list.end());
      edge_clearance[e] = list[k - 1];
    }
  }

  // 2. 指令のまま動いたときに、余裕の中へ近づいてくるボクセルを探す.
  //    許す距離は、余裕（margin）と、その辺の今の距離の小さいほう. そのため、すでに余裕より近くにある
  //    壁と平行に進むのは止めず、その壁にさらに近づくのは止める
  std::vector<double> hit_times;
  for (const auto & candidate : candidates) {
    for (std::size_t step = 1; step <= steps; ++step) {
      const Pose2 & pose = result.predictedPoses[step];
      const double dx = candidate.point.x - pose.x;
      const double dy = candidate.point.y - pose.y;
      const double c = std::cos(pose.yaw);
      const double s = std::sin(pose.yaw);
      const Point2 local{c * dx + s * dy, -s * dx + c * dy};
      const Feature then = closestFeature(local, config.footprint);
      const double allowed = std::min(
        config.margin, std::min(edge_clearance[then.edgeA], edge_clearance[then.edgeB]));
      if (then.distance < allowed - config.approachEpsilon &&
        then.distance < candidate.distance - config.approachEpsilon)
      {
        hit_times.push_back(
          std::min(static_cast<double>(step) * config.timeStep, config.timeHorizon));
        result.threatIndices.push_back(candidate.index);
        break;
      }
    }
  }

  if (hit_times.size() >= config.obstacleVoxelThreshold) {
    std::vector<double> sorted = hit_times;
    std::nth_element(
      sorted.begin(), sorted.begin() + (config.obstacleVoxelThreshold - 1), sorted.end());
    result.collisionTime = sorted[config.obstacleVoxelThreshold - 1];
  }

  if (result.collisionTime < 0.0) {
    result.state = GateState::Pass;
    result.scale = 1.0;
  } else if (result.collisionTime <= config.stopTime) {
    result.state = GateState::Stop;
    result.scale = 0.0;
  } else {
    result.state = GateState::Slow;
    result.scale = (result.collisionTime - config.stopTime) / (config.timeHorizon - config.stopTime);
  }
  result.velocity = scaled(cmd, result.scale);
  return result;
}
