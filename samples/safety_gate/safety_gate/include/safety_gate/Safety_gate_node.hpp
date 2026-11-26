#ifndef SAFETY_GATE_NODE_HPP_
#define SAFETY_GATE_NODE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "safety_gate/Safety_gate.hpp"

class SafetyGateNode : public rclcpp::Node
{
public:
  /* 概要：Publisher、Subscriber、タイマおよび安全機能ロジックを初期化する。
   * 入力：なし
   * 出力：なし
   */
  SafetyGateNode();

private:
  /* 概要：最新の点群と速度指令値から安全な速度指令値を生成し配信する。
   * 入力：なし
   * 出力：なし
   */
  void timerCallback();

  /* 概要：VLMから受信した速度指令値を保存する。
   * 入力：message VLMから受信した速度指令値
   * 出力：なし
   */
  void vlmCmdVelSubscriptionCallback(geometry_msgs::msg::Twist::ConstSharedPtr message);

  /* 概要：LIDARから受信した点群データを保存する。
   * 入力：message LIDARから受信した点群データ
   * 出力：なし
   */
  void pointCloudSubscriptionCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);

  /* 概要：ROS 2の速度指令値をROS非依存の速度型へ変換する。
   * 入力：message ROS 2の速度指令値
   * 出力：ROS非依存の速度指令値
   */
  static Velocity toVelocity(const geometry_msgs::msg::Twist & message);

  /* 概要：ROS非依存の速度型をROS 2の速度指令値へ変換する。
   * 入力：velocity ROS非依存の速度指令値
   * 出力：ROS 2の速度指令値
   */
  static geometry_msgs::msg::Twist toTwist(const Velocity & velocity);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vlmCmdVelSubscriber;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSubscriber;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher;
  rclcpp::TimerBase::SharedPtr pollingTimer;
  geometry_msgs::msg::Twist::ConstSharedPtr vlmCmdVelMessage;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pointCloudMessage;
  SafetyGate safetyGate;
};

#endif
