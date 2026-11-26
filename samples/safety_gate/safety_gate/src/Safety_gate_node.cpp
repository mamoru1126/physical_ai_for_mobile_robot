#include "safety_gate/Safety_gate_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace
{
constexpr char kNodeName[] = "safety_gate_node";
constexpr char kVlmCmdVelTopic[] = "/vlm_cmd_vel";
constexpr char kPointCloudTopic[] = "/point_cloud";
constexpr char kCmdVelTopic[] = "/cmd_vel";
constexpr std::chrono::milliseconds kPollingPeriod{20};
}

/* 概要：Publisher、Subscriber、タイマおよび安全機能ロジックを初期化する。
 * 入力：なし
 * 出力：なし
 */
SafetyGateNode::SafetyGateNode()
: Node(kNodeName)
{
  const std::string config_path =
    ament_index_cpp::get_package_share_directory("safety_gate") + "/config/config.yaml";
  safetyGate.loadConfig(config_path);

  vlmCmdVelSubscriber = create_subscription<geometry_msgs::msg::Twist>(
    kVlmCmdVelTopic, rclcpp::QoS(1).reliable(),
    std::bind(&SafetyGateNode::vlmCmdVelSubscriptionCallback, this, std::placeholders::_1));
  pointCloudSubscriber = create_subscription<sensor_msgs::msg::PointCloud2>(
    kPointCloudTopic, rclcpp::SensorDataQoS(),
    std::bind(&SafetyGateNode::pointCloudSubscriptionCallback, this, std::placeholders::_1));
  cmdVelPublisher = create_publisher<geometry_msgs::msg::Twist>(
    kCmdVelTopic, rclcpp::QoS(1).reliable());
  pollingTimer = create_wall_timer(
    kPollingPeriod, std::bind(&SafetyGateNode::timerCallback, this));
}

/* 概要：最新の点群と速度指令値から安全な速度指令値を生成し配信する。
 * 入力：なし
 * 出力：なし
 */
void SafetyGateNode::timerCallback()
{
  if (!vlmCmdVelMessage || !pointCloudMessage) {
    cmdVelPublisher->publish(geometry_msgs::msg::Twist{});
    return;
  }

  auto point_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*pointCloudMessage, *point_cloud);
  const Velocity input_velocity = toVelocity(*vlmCmdVelMessage);
  const Velocity safe_velocity = safetyGate.generateSafeCmdVel(point_cloud, input_velocity);
  cmdVelPublisher->publish(toTwist(safe_velocity));
}

/* 概要：VLMから受信した速度指令値を保存する。
 * 入力：message VLMから受信した速度指令値
 * 出力：なし
 */
void SafetyGateNode::vlmCmdVelSubscriptionCallback(
  geometry_msgs::msg::Twist::ConstSharedPtr message)
{
  vlmCmdVelMessage = message;
}

/* 概要：LIDARから受信した点群データを保存する。
 * 入力：message LIDARから受信した点群データ
 * 出力：なし
 */
void SafetyGateNode::pointCloudSubscriptionCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  pointCloudMessage = message;
}

/* 概要：ROS 2の速度指令値をROS非依存の速度型へ変換する。
 * 入力：message ROS 2の速度指令値
 * 出力：ROS非依存の速度指令値
 */
Velocity SafetyGateNode::toVelocity(const geometry_msgs::msg::Twist & message)
{
  return Velocity{message.linear.x, message.linear.y, message.linear.z,
    message.angular.x, message.angular.y, message.angular.z};
}

/* 概要：ROS非依存の速度型をROS 2の速度指令値へ変換する。
 * 入力：velocity ROS非依存の速度指令値
 * 出力：ROS 2の速度指令値
 */
geometry_msgs::msg::Twist SafetyGateNode::toTwist(const Velocity & velocity)
{
  geometry_msgs::msg::Twist message;
  message.linear.x = velocity.linearX;
  message.linear.y = velocity.linearY;
  message.linear.z = velocity.linearZ;
  message.angular.x = velocity.angularX;
  message.angular.y = velocity.angularY;
  message.angular.z = velocity.angularZ;
  return message;
}

/* 概要：SafetyGateNodeを起動し、ROS 2のイベント処理を実行する。
 * 入力：argc コマンドライン引数の数、argv コマンドライン引数
 * 出力：正常終了時は0
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyGateNode>());
  rclcpp::shutdown();
  return 0;
}
