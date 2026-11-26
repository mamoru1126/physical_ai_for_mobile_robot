#include <gtest/gtest.h>

#define private public
#include "safety_gate/Safety_gate.hpp"
#undef private

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace
{

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

PointCloud::Ptr createPointCloud(
    const std::vector<pcl::PointXYZ> & points)
{
    auto point_cloud = pcl::make_shared<PointCloud>();

    for (const auto & point : points)
    {
        point_cloud->push_back(point);
    }

    return point_cloud;
}

Velocity createVelocity(
    const float linear_x,
    const float linear_y,
    const float angular_z)
{
    Velocity velocity;
    velocity.linearX = linear_x;
    velocity.linearY = linear_y;
    velocity.angularZ = angular_z;

    return velocity;
}

}  // namespace

class SafetyGateTest : public ::testing::Test
{
protected:

    SafetyGate safetyGate;

    void SetUp() override
    {
        safetyGate.obstacleVoxelThreshold = 3;
        safetyGate.voxelSize = 0.1F;

        safetyGate.roiRange.minX = -1.0F;
        safetyGate.roiRange.maxX = 1.0F;
        safetyGate.roiRange.minY = -1.0F;
        safetyGate.roiRange.maxY = 1.0F;
        safetyGate.roiRange.minZ = -1.0F;
        safetyGate.roiRange.maxZ = 1.0F;
    }
};

/**
 * UT-01
 * 障害物なし
 */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_NoObstacle)
{
    auto point_cloud = createPointCloud({});

    const auto input_velocity =
        createVelocity(0.5F, 0.0F, 0.0F);

    const auto result =
        safetyGate.generateSafeCmdVel(
            point_cloud,
            input_velocity);

    EXPECT_FLOAT_EQ(result.linearX, 0.5F);
    EXPECT_FLOAT_EQ(result.linearY, 0.0F);
    EXPECT_FLOAT_EQ(result.angularZ, 0.0F);
}

/**
 * UT-02
 * 障害物数が閾値未満
 */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_BelowThreshold)
{
    auto point_cloud = createPointCloud(
    {
        {0.0F, 0.0F, 0.0F},
        {0.3F, 0.0F, 0.0F}
    });

    const auto input_velocity =
        createVelocity(0.5F, 0.0F, 0.0F);

    const auto result =
        safetyGate.generateSafeCmdVel(
            point_cloud,
            input_velocity);

    EXPECT_FLOAT_EQ(result.linearX, 0.5F);
    EXPECT_FLOAT_EQ(result.linearY, 0.0F);
    EXPECT_FLOAT_EQ(result.angularZ, 0.0F);
}

/**
 * UT-03
 * 障害物数が閾値と一致
 */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_ThresholdEqual)
{
    auto point_cloud = createPointCloud(
    {
        {0.0F, 0.0F, 0.0F},
        {0.3F, 0.0F, 0.0F},
        {0.6F, 0.0F, 0.0F}
    });

    const auto input_velocity =
        createVelocity(0.5F, 0.0F, 0.0F);

    const auto result =
        safetyGate.generateSafeCmdVel(
            point_cloud,
            input_velocity);

    EXPECT_FLOAT_EQ(result.linearX, 0.0F);
    EXPECT_FLOAT_EQ(result.linearY, 0.0F);
    EXPECT_FLOAT_EQ(result.angularZ, 0.0F);
}

/**
 * UT-04
 * 障害物数が閾値超過
 */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_ThresholdOver)
{
    auto point_cloud = createPointCloud(
    {
        {0.0F, 0.0F, 0.0F},
        {0.3F, 0.0F, 0.0F},
        {0.6F, 0.0F, 0.0F},
        {0.9F, 0.0F, 0.0F},
        {-0.9F, 0.0F, 0.0F},
        {0.0F, 0.3F, 0.0F},
        {0.0F, 0.6F, 0.0F},
        {0.0F, 0.9F, 0.0F},
        {0.5F, 0.5F, 0.0F},
        {-0.5F, -0.5F, 0.0F}
    });

    const auto input_velocity =
        createVelocity(0.5F, 0.0F, 0.0F);

    const auto result =
        safetyGate.generateSafeCmdVel(
            point_cloud,
            input_velocity);

    EXPECT_FLOAT_EQ(result.linearX, 0.0F);
    EXPECT_FLOAT_EQ(result.linearY, 0.0F);
    EXPECT_FLOAT_EQ(result.angularZ, 0.0F);
}

/**
 * UT-05
 * 空点群入力
 */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_EmptyPointCloud)
{
    auto point_cloud =
        pcl::make_shared<PointCloud>();

    const auto input_velocity =
        createVelocity(0.5F, 0.0F, 0.0F);

    const auto result =
        safetyGate.generateSafeCmdVel(
            point_cloud,
            input_velocity);

    EXPECT_FLOAT_EQ(result.linearX, 0.5F);
    EXPECT_FLOAT_EQ(result.linearY, 0.0F);
    EXPECT_FLOAT_EQ(result.angularZ, 0.0F);
}

/**
 * UT-06
 * 停止指令入力
 */
TEST_F(SafetyGateTest, GenerateSafeCmdVel_StopCommand)
{
    auto point_cloud = createPointCloud({});

    const auto input_velocity =
        createVelocity(0.0F, 0.0F, 0.0F);

    const auto result =
        safetyGate.generateSafeCmdVel(
            point_cloud,
            input_velocity);

    EXPECT_FLOAT_EQ(result.linearX, 0.0F);
    EXPECT_FLOAT_EQ(result.linearY, 0.0F);
    EXPECT_FLOAT_EQ(result.angularZ, 0.0F);
}

/**
 * UT-07
 * ROI内点のみ抽出
 */
TEST_F(SafetyGateTest, ExtractRoiPointCloud_AllInside)
{
    auto point_cloud = createPointCloud(
    {
        {0.0F, 0.0F, 0.0F},
        {0.5F, 0.5F, 0.0F},
        {-0.5F, 0.0F, 0.0F}
    });

    auto result =
        safetyGate.extractRoiPointCloud(
            point_cloud);

    EXPECT_EQ(result->size(), 3U);
}

/**
 * UT-08
 * ROI外点のみ除外
 */
TEST_F(SafetyGateTest, ExtractRoiPointCloud_AllOutside)
{
    auto point_cloud = createPointCloud(
    {
        {2.0F, 0.0F, 0.0F},
        {3.0F, 0.0F, 0.0F},
        {4.0F, 0.0F, 0.0F}
    });

    auto result =
        safetyGate.extractRoiPointCloud(
            point_cloud);

    EXPECT_EQ(result->size(), 0U);
}

/**
 * UT-09
 * ROI内外混在
 */
TEST_F(SafetyGateTest, ExtractRoiPointCloud_Mixed)
{
    auto point_cloud = createPointCloud(
    {
        {0.0F, 0.0F, 0.0F},
        {0.5F, 0.0F, 0.0F},
        {2.0F, 0.0F, 0.0F},
        {3.0F, 0.0F, 0.0F},
        {4.0F, 0.0F, 0.0F}
    });

    auto result =
        safetyGate.extractRoiPointCloud(
            point_cloud);

    EXPECT_EQ(result->size(), 2U);
}

/**
 * UT-10
 * 空点群入力
 */
TEST_F(SafetyGateTest, ExtractRoiPointCloud_Empty)
{
    auto point_cloud =
        pcl::make_shared<PointCloud>();

    auto result =
        safetyGate.extractRoiPointCloud(
            point_cloud);

    EXPECT_TRUE(result->empty());
}