#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "safety_gate/Safety_gate.hpp"

namespace
{

/* 足形 0.7 m × 0.6 m（前後 ±0.35、左右 ±0.30）、余裕 0.1 m、2 秒先まで、0.5 秒より早ければ止める */
SafetyGateConfig createConfig()
{
  SafetyGateConfig config;
  config.footprint = {{0.35, 0.30}, {0.35, -0.30}, {-0.35, -0.30}, {-0.35, 0.30}};
  config.heightRange = {0.05, 1.5};
  config.viewHalfSize = 5.0;
  config.voxelSize = 0.05;
  config.obstacleVoxelThreshold = 3;
  config.margin = 0.10;
  config.timeHorizon = 2.0;
  config.stopTime = 0.5;
  config.timeStep = 0.1;
  config.maxLinearX = 0.5;
  config.maxLinearY = 0.5;
  config.maxAngularZ = 1.0;
  return config;
}

Velocity createVelocity(double linear_x, double linear_y, double angular_z)
{
  Velocity velocity;
  velocity.linearX = linear_x;
  velocity.linearY = linear_y;
  velocity.angularZ = angular_z;
  return velocity;
}

/* x = 一定、y を 5 cm ずつずらした「壁」（高さ 0.5 m） */
std::vector<Point3> createWallAtX(float x, int count)
{
  std::vector<Point3> points;
  for (int i = 0; i < count; ++i) {
    points.push_back({x, -0.10F + 0.05F * static_cast<float>(i), 0.5F});
  }
  return points;
}

}  // namespace

class SafetyGateTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    safetyGate.setConfig(createConfig());
  }

  SafetyGate safetyGate;
};

/* UT-01 障害物なし: 入力した指令をそのまま返す */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_NoObstacle)
{
  const auto result = safetyGate.generateSafeCmdVel({}, createVelocity(0.3, 0.0, 0.0));
  EXPECT_EQ(result.state, GateState::Pass);
  EXPECT_DOUBLE_EQ(result.velocity.linearX, 0.3);
  EXPECT_DOUBLE_EQ(result.scale, 1.0);
  EXPECT_LT(result.collisionTime, 0.0);
}

/* UT-02 進む先の障害物が閾値未満: そのまま返す（ノイズとみなす） */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_BelowThreshold)
{
  const auto voxels = safetyGate.extractObstacleVoxels(createWallAtX(0.50F, 2));
  ASSERT_EQ(voxels.size(), 2U);
  const auto result = safetyGate.generateSafeCmdVel(voxels, createVelocity(0.3, 0.0, 0.0));
  EXPECT_EQ(result.state, GateState::Pass);
  EXPECT_DOUBLE_EQ(result.velocity.linearX, 0.3);
}

/* UT-03 進む先のすぐ近くに、閾値と同じ数の障害物: 止める */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_ThresholdEqual)
{
  const auto voxels = safetyGate.extractObstacleVoxels(createWallAtX(0.50F, 3));
  ASSERT_EQ(voxels.size(), 3U);
  const auto result = safetyGate.generateSafeCmdVel(voxels, createVelocity(0.3, 0.0, 0.0));
  EXPECT_EQ(result.state, GateState::Stop);
  EXPECT_DOUBLE_EQ(result.velocity.linearX, 0.0);
  EXPECT_DOUBLE_EQ(result.velocity.angularZ, 0.0);
  EXPECT_EQ(result.threatIndices.size(), 3U);
}

/* UT-04 進む先のすぐ近くに、閾値を超える障害物: 止める */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_ThresholdOver)
{
  const auto voxels = safetyGate.extractObstacleVoxels(createWallAtX(0.50F, 10));
  const auto result = safetyGate.generateSafeCmdVel(voxels, createVelocity(0.3, 0.0, 0.0));
  EXPECT_EQ(result.state, GateState::Stop);
  EXPECT_DOUBLE_EQ(result.velocity.linearX, 0.0);
}

/* UT-05 空の点群: そのまま返す */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_EmptyPointCloud)
{
  const auto voxels = safetyGate.extractObstacleVoxels({});
  EXPECT_TRUE(voxels.empty());
  const auto result = safetyGate.generateSafeCmdVel(voxels, createVelocity(0.3, 0.0, 0.0));
  EXPECT_EQ(result.state, GateState::Pass);
  EXPECT_DOUBLE_EQ(result.velocity.linearX, 0.3);
}

/* UT-06 停止指令: 0 を返す（IDLE） */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_StopCommand)
{
  const auto voxels = safetyGate.extractObstacleVoxels(createWallAtX(0.50F, 10));
  const auto result = safetyGate.generateSafeCmdVel(voxels, createVelocity(0.0, 0.0, 0.0));
  EXPECT_EQ(result.state, GateState::Idle);
  EXPECT_DOUBLE_EQ(result.velocity.linearX, 0.0);
  EXPECT_DOUBLE_EQ(result.velocity.angularZ, 0.0);
}

/* UT-07 範囲内の点だけを残す（すべて範囲内） */
TEST_F(SafetyGateTest, ExtractObstacleVoxels_AllInside)
{
  const auto voxels = safetyGate.extractObstacleVoxels(
    {{1.0F, 0.0F, 0.5F}, {0.0F, 2.0F, 1.0F}, {-3.0F, -1.0F, 0.1F}});
  EXPECT_EQ(voxels.size(), 3U);
}

/* UT-08 範囲外の点を除く（床、天井、表示の範囲の外、数値でない点） */
TEST_F(SafetyGateTest, ExtractObstacleVoxels_AllOutside)
{
  const auto voxels = safetyGate.extractObstacleVoxels(
    {{1.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 2.0F}, {6.0F, 0.0F, 0.5F}, {0.0F, -6.0F, 0.5F},
      {std::nanf(""), 0.0F, 0.5F}});
  EXPECT_TRUE(voxels.empty());
}

/* UT-09 範囲の内と外が混ざる */
TEST_F(SafetyGateTest, ExtractObstacleVoxels_Mixed)
{
  const auto voxels = safetyGate.extractObstacleVoxels(
    {{1.0F, 0.0F, 0.5F}, {2.0F, 0.0F, 0.5F}, {1.0F, 0.0F, 0.0F}, {9.0F, 0.0F, 0.5F},
      {1.0F, 0.0F, 3.0F}});
  EXPECT_EQ(voxels.size(), 2U);
}

/* UT-10 同じボクセルの点は 1 つにまとめ、中心の点で表す */
TEST_F(SafetyGateTest, ExtractObstacleVoxels_Voxelize)
{
  const auto voxels = safetyGate.extractObstacleVoxels(
    {{1.01F, 0.01F, 0.51F}, {1.04F, 0.04F, 0.54F}, {1.06F, 0.01F, 0.51F}});
  ASSERT_EQ(voxels.size(), 2U);
  EXPECT_NEAR(voxels[0].x, 1.025F, 1e-4);
  EXPECT_NEAR(voxels[0].y, 0.025F, 1e-4);
  EXPECT_NEAR(voxels[0].z, 0.525F, 1e-4);
}

/* UT-11 前に障害物があっても、下がる指令は通す（離れる動きは止めない） */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_BackAwayFromObstacle)
{
  const auto voxels = safetyGate.extractObstacleVoxels(createWallAtX(0.40F, 5));
  const auto result = safetyGate.generateSafeCmdVel(voxels, createVelocity(-0.2, 0.0, 0.0));
  EXPECT_EQ(result.state, GateState::Pass);
  EXPECT_DOUBLE_EQ(result.velocity.linearX, -0.2);
}

/* UT-12 後ろに障害物があっても、前に進む指令は通す */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_ObstacleBehind)
{
  const auto voxels = safetyGate.extractObstacleVoxels(createWallAtX(-0.40F, 5));
  const auto result = safetyGate.generateSafeCmdVel(voxels, createVelocity(0.3, 0.0, 0.0));
  EXPECT_EQ(result.state, GateState::Pass);
}

/* UT-13 後ろに障害物があり、下がる指令: 止める（後ろも見る） */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_BackIntoObstacle)
{
  const auto voxels = safetyGate.extractObstacleVoxels(createWallAtX(-0.50F, 5));
  const auto result = safetyGate.generateSafeCmdVel(voxels, createVelocity(-0.3, 0.0, 0.0));
  EXPECT_EQ(result.state, GateState::Stop);
}

/* UT-14 少し先の障害物: 速度を落とす（止めはしない） */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_SlowDown)
{
  // 前の縁 0.35 + 余裕 0.10 = 0.45. 壁 1.0 m まで 0.55 m、0.4 m/s なら 1.4 秒ほど
  const auto voxels = safetyGate.extractObstacleVoxels(createWallAtX(1.0F, 5));
  const auto result = safetyGate.generateSafeCmdVel(voxels, createVelocity(0.4, 0.0, 0.0));
  EXPECT_EQ(result.state, GateState::Slow);
  EXPECT_GT(result.collisionTime, 0.5);
  EXPECT_LE(result.collisionTime, 2.0);
  EXPECT_GT(result.velocity.linearX, 0.0);
  EXPECT_LT(result.velocity.linearX, 0.4);
  EXPECT_NEAR(result.velocity.linearX, 0.4 * result.scale, 1e-9);
}

/* UT-15 遠くの障害物（見る時間のうちに届かない）: そのまま通す */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_FarObstacle)
{
  const auto voxels = safetyGate.extractObstacleVoxels(createWallAtX(3.0F, 5));
  const auto result = safetyGate.generateSafeCmdVel(voxels, createVelocity(0.3, 0.0, 0.0));
  EXPECT_EQ(result.state, GateState::Pass);
}

/* UT-16 その場で回る: すぐに角が近づく向きは止める. 逆の向きは、ぶつかるのが先なので止めない.
 *       遠くの障害物なら、どちらに回っても通す */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_RotateInPlace)
{
  // 前の縁の右寄りのすぐ前. 左に回る（反時計回り）と、右前の角が前に出てきて近づく.
  // 右に回る（時計回り）と、右前の角は下がっていく（ただし大きく回ると、左前の角がやってくる）
  std::vector<Point3> points;
  for (int i = 0; i < 4; ++i) {
    points.push_back({0.42F, -0.08F - 0.05F * static_cast<float>(i), 0.5F});
  }
  const auto voxels = safetyGate.extractObstacleVoxels(points);
  const auto blocked = safetyGate.generateSafeCmdVel(voxels, createVelocity(0.0, 0.0, 0.8));
  EXPECT_EQ(blocked.state, GateState::Stop);

  const auto other_way = safetyGate.generateSafeCmdVel(voxels, createVelocity(0.0, 0.0, -0.8));
  EXPECT_NE(other_way.state, GateState::Stop);
  EXPECT_GT(other_way.collisionTime, 0.5);

  const auto far = safetyGate.extractObstacleVoxels(createWallAtX(1.2F, 5));
  const auto free_turn = safetyGate.generateSafeCmdVel(far, createVelocity(0.0, 0.0, 0.8));
  EXPECT_EQ(free_turn.state, GateState::Pass);
  EXPECT_DOUBLE_EQ(free_turn.velocity.angularZ, 0.8);
}

/* UT-17 足形の中の点（ロボット自身など）は使わない */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_IgnoreInsideFootprint)
{
  const auto voxels = safetyGate.extractObstacleVoxels(createWallAtX(0.20F, 10));
  const auto result = safetyGate.generateSafeCmdVel(voxels, createVelocity(0.3, 0.0, 0.0));
  EXPECT_EQ(result.state, GateState::Pass);
}

/* UT-18 余裕より近い横の壁と平行に進む: 通す. 壁に近づく・前の人に近づくのは止める */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_ParallelToWall)
{
  std::vector<Point3> wall;
  for (int i = 0; i < 60; ++i) {
    wall.push_back({-1.0F + 0.05F * static_cast<float>(i), 0.37F, 0.5F});  // 左の縁から 7.5 cm
  }
  const auto voxels = safetyGate.extractObstacleVoxels(wall);
  const auto result = safetyGate.generateSafeCmdVel(voxels, createVelocity(0.3, 0.0, 0.0));
  EXPECT_EQ(result.state, GateState::Pass);

  const auto toward = safetyGate.generateSafeCmdVel(voxels, createVelocity(0.0, 0.2, 0.0));
  EXPECT_EQ(toward.state, GateState::Stop);

  // 横に壁があっても、前の障害物には余裕（margin）で止まる
  auto with_front = wall;
  const auto front = createWallAtX(0.50F, 5);
  with_front.insert(with_front.end(), front.begin(), front.end());
  const auto front_voxels = safetyGate.extractObstacleVoxels(with_front);
  const auto ahead = safetyGate.generateSafeCmdVel(front_voxels, createVelocity(0.3, 0.0, 0.0));
  EXPECT_EQ(ahead.state, GateState::Stop);
}

/* UT-19 指令を上限で抑え、数値でない指令は 0 にする */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_ClampAndNaN)
{
  const auto clamped = safetyGate.generateSafeCmdVel({}, createVelocity(2.0, -2.0, 5.0));
  EXPECT_DOUBLE_EQ(clamped.velocity.linearX, 0.5);
  EXPECT_DOUBLE_EQ(clamped.velocity.linearY, -0.5);
  EXPECT_DOUBLE_EQ(clamped.velocity.angularZ, 1.0);

  const auto nan = safetyGate.generateSafeCmdVel({}, createVelocity(std::nan(""), 0.0, 0.0));
  EXPECT_EQ(nan.state, GateState::Idle);
  EXPECT_DOUBLE_EQ(nan.velocity.linearX, 0.0);
}

/* UT-20 多角形までの符号付き距離（外は正、中は負） */
TEST(SafetyGateGeometryTest, SignedDistance)
{
  const std::vector<Point2> square{{1.0, 1.0}, {1.0, -1.0}, {-1.0, -1.0}, {-1.0, 1.0}};
  EXPECT_NEAR(SafetyGate::signedDistance({2.0, 0.0}, square), 1.0, 1e-9);
  EXPECT_NEAR(SafetyGate::signedDistance({0.0, 0.0}, square), -1.0, 1e-9);
  EXPECT_NEAR(SafetyGate::signedDistance({0.5, 0.0}, square), -0.5, 1e-9);
  EXPECT_NEAR(SafetyGate::signedDistance({2.0, 2.0}, square), std::sqrt(2.0), 1e-9);
}

/* UT-21 姿勢の予測（まっすぐ、円弧、横） */
TEST(SafetyGateGeometryTest, PredictPose)
{
  const auto straight = SafetyGate::predictPose(createVelocity(0.5, 0.0, 0.0), 2.0);
  EXPECT_NEAR(straight.x, 1.0, 1e-9);
  EXPECT_NEAR(straight.y, 0.0, 1e-9);

  // 半径 1 m の円を 1/4 周: (1, 1)、向き 90 度
  const auto arc = SafetyGate::predictPose(createVelocity(1.0, 0.0, 1.0), M_PI / 2.0);
  EXPECT_NEAR(arc.x, 1.0, 1e-9);
  EXPECT_NEAR(arc.y, 1.0, 1e-9);
  EXPECT_NEAR(arc.yaw, M_PI / 2.0, 1e-9);

  const auto side = SafetyGate::predictPose(createVelocity(0.0, 0.5, 0.0), 1.0);
  EXPECT_NEAR(side.y, 0.5, 1e-9);
}

/* UT-22 正しくない設定値は受け付けない */
TEST(SafetyGateConfigTest, RejectInvalidConfig)
{
  SafetyGate gate;
  auto config = createConfig();
  config.footprint = {{0.3, 0.3}, {0.3, -0.3}};
  EXPECT_THROW(gate.setConfig(config), std::runtime_error);

  config = createConfig();
  config.footprint = {{1.0, 1.0}, {1.0, 0.5}, {0.5, 0.5}};  // 原点を囲まない
  EXPECT_THROW(gate.setConfig(config), std::runtime_error);

  config = createConfig();
  config.stopTime = 2.0;
  EXPECT_THROW(gate.setConfig(config), std::runtime_error);

  config = createConfig();
  config.voxelSize = 0.0;
  EXPECT_THROW(gate.setConfig(config), std::runtime_error);

  config = createConfig();
  config.obstacleVoxelThreshold = 0;
  EXPECT_THROW(gate.setConfig(config), std::runtime_error);
}

#ifdef TEST_CONFIG_PATH
/* UT-23 設定ファイルを読む */
TEST(SafetyGateConfigTest, LoadConfigFile)
{
  SafetyGate gate;
  gate.loadConfig(TEST_CONFIG_PATH);
  const auto & config = gate.getConfig();
  EXPECT_EQ(config.footprint.size(), 4U);
  EXPECT_DOUBLE_EQ(config.viewHalfSize, 5.0);
  EXPECT_EQ(config.obstacleVoxelThreshold, 3U);
  EXPECT_THROW(gate.loadConfig("/nonexistent/config.yaml"), std::runtime_error);
}
#endif
