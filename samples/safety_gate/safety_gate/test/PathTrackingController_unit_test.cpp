#include <cmath>

#include "PathTrackingController.hpp"
#include "MobiKdTree.hpp"

#include <gtest/gtest.h>

namespace
{

/* 経路上の点の座標生成用 */
geometry_msgs::msg::Point createPoint(
    double x,
    double y)
{
    geometry_msgs::msg::Point p;

    p.x = x;
    p.y = y;
    p.z = 0.0;

    return p;
}

/* @mobiパス生成用 */
constexpr double MOBI_POINT_INTERVAL = 0.01;

constexpr double MOBI_LEFT_X = 10.0;
constexpr double MOBI_RIGHT_X = 30.0;

constexpr double MOBI_BOTTOM_Y = 10.0;
constexpr double MOBI_START_Y = 20.0;
constexpr double MOBI_GOAL_Y = 15.0;
constexpr double MOBI_TOP_Y = 40.0;

std::vector<geometry_msgs::msg::Point> createMobiPath()
{
    std::vector<geometry_msgs::msg::Point> path;

    auto addPoint =
        [&path](double x, double y)
        {
            geometry_msgs::msg::Point point;

            point.x = x;
            point.y = y;
            point.z = 0.0;

            path.push_back(point);
        };

    /* 左辺上側 */
    for (
        double y = MOBI_START_Y;
        y <= MOBI_TOP_Y;
        y += MOBI_POINT_INTERVAL)
    {
        addPoint(
            MOBI_LEFT_X,
            y);
    }

    /* 上辺 */
    for (
        double x = MOBI_LEFT_X + MOBI_POINT_INTERVAL;
        x <= MOBI_RIGHT_X;
        x += MOBI_POINT_INTERVAL)
    {
        addPoint(
            x,
            MOBI_TOP_Y);
    }

    /* 右辺 */
    for (
        double y = MOBI_TOP_Y - MOBI_POINT_INTERVAL;
        y >= MOBI_BOTTOM_Y;
        y -= MOBI_POINT_INTERVAL)
    {
        addPoint(
            MOBI_RIGHT_X,
            y);
    }

    /* 下辺 */
    for (
        double x = MOBI_RIGHT_X - MOBI_POINT_INTERVAL;
        x >= MOBI_LEFT_X;
        x -= MOBI_POINT_INTERVAL)
    {
        addPoint(
            x,
            MOBI_BOTTOM_Y);
    }

    /* 左辺下側 */
    for (
        double y = MOBI_BOTTOM_Y + MOBI_POINT_INTERVAL;
        y <= MOBI_GOAL_Y;
        y += MOBI_POINT_INTERVAL)
    {
        addPoint(
            MOBI_LEFT_X,
            y);
    }

    return path;
}

/* VLAパス生成用 */
constexpr double PATH_LENGTH = 5.0;
constexpr double POINT_INTERVAL = 0.01;
constexpr double VLA_END_OFFSET_X = 0.5;

std::vector<geometry_msgs::msg::Point> createVlaPath(
    double robotX,
    double robotY)
{
    std::vector<geometry_msgs::msg::Point> path;

    for (
        double y = 0.0;
        y <= PATH_LENGTH;
        y += POINT_INTERVAL)
    {
        geometry_msgs::msg::Point point;

        /* 放物線で右へ緩やかに曲がる */
        const double offsetX =
            VLA_END_OFFSET_X *
            (y * y) /
            (PATH_LENGTH * PATH_LENGTH);

        point.x = robotX + offsetX;
        point.y = robotY + y;
        point.z = 0.0;

        path.push_back(point);
    }

    return path;
}

} // namespace

/* テスト用定数 */
constexpr double TEST_OFFSET = 1.0;
constexpr double TEST_VELOCITY = 1.0;
constexpr double ANGULAR_TOLERANCE = 1e-2;

constexpr double MOBI_MIDDLE_Y =
    (MOBI_START_Y + MOBI_TOP_Y) / 2.0;

/* UT-01 */
TEST(MobiKdTreeTest, FindNearest)
{
    const auto mobiPath =
        createMobiPath();

    MobiKdTree kdTree(
        mobiPath);

    const std::vector<geometry_msgs::msg::Point> queries =
    {
        createPoint(10.05, 25.00),
        createPoint(29.95, 40.10),
        createPoint(30.20, 20.00),
        createPoint(20.00, 10.20),
        createPoint(9.90, 14.50),
        createPoint(15.00, 39.80),
        createPoint(30.05, 35.00),
        createPoint(25.00, 10.05),
        createPoint(10.15, 30.00),
        createPoint(10.05, 10.50)
    };

    for (const auto& query : queries)
    {
        double expectedDistance =
            std::numeric_limits<double>::max();

        size_t expectedIndex = 0;

        for (size_t i = 0; i < mobiPath.size(); ++i)
        {
            const double dx =
                query.x - mobiPath[i].x;

            const double dy =
                query.y - mobiPath[i].y;

            const double distance =
                std::hypot(
                    dx,
                    dy);

            if (distance < expectedDistance)
            {
                expectedDistance = distance;
                expectedIndex = i;
            }
        }

        auto [actualDistance, actualIndex] =
            kdTree.findNearest(
                query);

        EXPECT_EQ(
            actualIndex,
            expectedIndex);
    }
}

/* UT-02 */
TEST(PathTrackingControllerTest, CheckModeSwitchLessThanThreshold)
{
    PathTrackingController controller(
        createMobiPath());

    constexpr double OFFSET = 0.1;

    auto vlaPath =
        createVlaPath(
            MOBI_LEFT_X -
                PathTrackingController::DISTANCE_THRESHOLD +
                OFFSET -
                VLA_END_OFFSET_X,
            MOBI_GOAL_Y);

    controller.checkModeSwitch(vlaPath);

    EXPECT_EQ(
        controller.currentMode,
        PathTrackingController::Mode::VLA);
}

/* UT-03 */
TEST(PathTrackingControllerTest, CheckModeSwitchOverThreshold)
{
    PathTrackingController controller(
        createMobiPath());

    constexpr double OFFSET = 0.1;

    auto vlaPath =
        createVlaPath(
            MOBI_LEFT_X -
                PathTrackingController::DISTANCE_THRESHOLD -
                OFFSET -
                VLA_END_OFFSET_X,
            MOBI_GOAL_Y);

    controller.checkModeSwitch(vlaPath);

    EXPECT_EQ(
        controller.currentMode,
        PathTrackingController::Mode::MOBI);
}

/* UT-04 */
TEST(PathTrackingControllerTest, VlaControlHeadingMatch)
{
    PathTrackingController controller(
        createMobiPath());

    PathTrackingController::RobotState robot {};

    robot.x = MOBI_LEFT_X;
    robot.y = MOBI_START_Y;
    robot.yaw = std::numbers::pi / 2.0;

    auto cmd =
        controller.calculateCmdVel(
            robot,
            createVlaPath(
                robot.x,
                robot.y));

    EXPECT_NEAR(
        cmd.angular,
        0.0,
        ANGULAR_TOLERANCE);
}

/* UT-05 */
TEST(PathTrackingControllerTest, VlaControlRightHeading)
{
    PathTrackingController controller(
        createMobiPath());

    PathTrackingController::RobotState robot {};

    robot.x = MOBI_LEFT_X;
    robot.y = MOBI_START_Y;
    robot.yaw = std::numbers::pi / 4.0;

    auto cmd =
        controller.calculateCmdVel(
            robot,
            createVlaPath(
                robot.x,
                robot.y));

    EXPECT_GT(
        cmd.angular,
        0.0);
}

/* UT-06 */
TEST(PathTrackingControllerTest, VlaControlLeftHeading)
{
    PathTrackingController controller(
        createMobiPath());

    PathTrackingController::RobotState robot {};

    robot.x = MOBI_LEFT_X;
    robot.y = MOBI_START_Y;
    robot.yaw = std::numbers::pi * 3.0 / 4.0;

    auto cmd =
        controller.calculateCmdVel(
            robot,
            createVlaPath(
                robot.x,
                robot.y));

    EXPECT_LT(
        cmd.angular,
        0.0);
}

/* UT-07 */
TEST(PathTrackingControllerTest, MobiControlOnPath)
{
    PathTrackingController controller(
        createMobiPath());

    controller.currentMode =
        PathTrackingController::Mode::MOBI;

    PathTrackingController::RobotState robot {};

    robot.x = MOBI_LEFT_X;
    robot.y = MOBI_MIDDLE_Y;
    robot.yaw = std::numbers::pi / 2.0;
    robot.velocity = TEST_VELOCITY;

    auto cmd =
        controller.calculateCmdVel(
            robot,
            {});

    EXPECT_NEAR(
        cmd.angular,
        0.0,
        ANGULAR_TOLERANCE);
}

/* UT-08 */
TEST(PathTrackingControllerTest, MobiControlRightSideOffset)
{
    PathTrackingController controller(
        createMobiPath());

    controller.currentMode =
        PathTrackingController::Mode::MOBI;

    PathTrackingController::RobotState robot {};

    robot.x = MOBI_LEFT_X + TEST_OFFSET;
    robot.y = MOBI_MIDDLE_Y;
    robot.yaw = std::numbers::pi / 2.0;
    robot.velocity = TEST_VELOCITY;

    auto cmd =
        controller.calculateCmdVel(
            robot,
            {});

    EXPECT_GT(
        cmd.angular,
        0.0);
}

/* UT-09 */
TEST(PathTrackingControllerTest, MobiControlLeftSideOffset)
{
    PathTrackingController controller(
        createMobiPath());

    controller.currentMode =
        PathTrackingController::Mode::MOBI;

    PathTrackingController::RobotState robot {};

    robot.x = MOBI_LEFT_X - TEST_OFFSET;
    robot.y = MOBI_MIDDLE_Y;
    robot.yaw = std::numbers::pi / 2.0;
    robot.velocity = TEST_VELOCITY;

    auto cmd =
        controller.calculateCmdVel(
            robot,
            {});

    EXPECT_LT(
        cmd.angular,
        0.0);
}

/* UT-10 */
TEST(PathTrackingControllerTest, MobiControlRightHeading)
{
    PathTrackingController controller(
        createMobiPath());

    controller.currentMode =
        PathTrackingController::Mode::MOBI;

    PathTrackingController::RobotState robot {};

    robot.x = MOBI_LEFT_X;
    robot.y = MOBI_MIDDLE_Y;
    robot.yaw = 0.0;
    robot.velocity = TEST_VELOCITY;

    auto cmd =
        controller.calculateCmdVel(
            robot,
            {});

    EXPECT_GT(
        cmd.angular,
        0.0);
}

/* UT-11 */
TEST(PathTrackingControllerTest, MobiControlLeftHeading)
{
    PathTrackingController controller(
        createMobiPath());

    controller.currentMode =
        PathTrackingController::Mode::MOBI;

    PathTrackingController::RobotState robot {};

    robot.x = MOBI_LEFT_X;
    robot.y = MOBI_MIDDLE_Y;
    robot.yaw = std::numbers::pi;
    robot.velocity = TEST_VELOCITY;

    auto cmd =
        controller.calculateCmdVel(
            robot,
            {});

    EXPECT_LT(
        cmd.angular,
        0.0);
}