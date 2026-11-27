#ifndef SAFETY_GATE_NODE_HPP_
#define SAFETY_GATE_NODE_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "safety_gate/Safety_gate.hpp"

/* 通信まわりの設定値（設定ファイルの topics、frames、tf、safety、web） */
struct NodeSettings
{
  std::string cmdInTopic{"/vlm_cmd_vel"};
  std::string cloudTopic{"/cloud"};
  std::string cmdOutTopic{"/cmd_vel"};
  std::string stateTopic{"/safety_gate/state"};
  std::string debugTopic{"/safety_gate/debug"};
  std::string baseFrame{"base_link"};
  std::string cloudFrame{};  // 空なら点群のヘッダの座標系
  bool publishStaticTf{false};
  std::string tfParent{"base_link"};
  std::string tfChild{"cloud_link"};
  std::vector<double> tfXyz{0.0, 0.0, 0.0};
  std::vector<double> tfRpy{0.0, 0.0, 0.0};
  double cmdTimeout{0.5};
  double cloudTimeout{0.5};
  double rate{20.0};
  bool publishWhenIdle{true};   // 指令が来ていない間も 0 を出し続けるか（false: 0 を idleTail 秒出してから黙る）
  double idleTail{0.5};
  double webRate{5.0};
  double cellSize{0.05};
};

class SafetyGateNode : public rclcpp::Node
{
public:
  /* 概要：設定ファイルを読み、Publisher、Subscriber、TF、タイマおよび安全機能ロジックを初期化する。
   * 入力：なし（ROS 2 のパラメータ config_file に設定ファイルのパス）
   * 出力：なし
   */
  SafetyGateNode();

private:
  /* 概要：最新の点群と速度指令値から安全な速度指令値を生成し配信する。
   *       指令や点群が古い・TF がないときは、止める指令を配信する。
   *       publish_when_idle が false なら、指令が途絶えて idle_tail 秒たった後は何も配信しない
   *       （元のシステムと同じ /cmd_vel を使い分けるため）。
   * 入力：なし
   * 出力：なし
   */
  void timerCallback();

  /* 概要：VLMから受信した速度指令値を、受け取った時刻とともに保存する。
   * 入力：message VLMから受信した速度指令値
   * 出力：なし
   */
  void vlmCmdVelSubscriptionCallback(geometry_msgs::msg::Twist::ConstSharedPtr message);

  /* 概要：LIDARから受信した点群を base_link に直し、障害物のボクセルにして保存する。
   * 入力：message LIDARから受信した点群データ
   * 出力：なし
   */
  void pointCloudSubscriptionCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);

  /* 概要：点群を TF で base_link 座標系に直す。
   * 入力：message 点群、points 直した点群の入れ先、error うまくいかなかったときの理由の入れ先
   * 出力：うまくいけば true
   */
  bool transformPointCloud(
    const sensor_msgs::msg::PointCloud2 & message, std::vector<Point3> & points,
    std::string & error);

  /* 概要：WebUI 用の情報（小数 2 桁、点は表示のマスの番号）を JSON にして配信する。
   * 入力：result 判定の結果、input 入力速度指令値、state 状態、reason 理由
   * 出力：なし
   */
  void publishDebug(
    const GateResult & result, const Velocity & input, const std::string & state,
    const std::string & reason);

  /* 概要：設定ファイルから通信まわりの設定値を読み込む。
   * 入力：config_file_path 設定ファイルのパス
   * 出力：設定値
   */
  static NodeSettings loadSettings(const std::string & config_file_path);

  /* 概要：ROS 2のTwistをROS 2非依存のVelocityへ変換する。 */
  static Velocity toVelocity(const geometry_msgs::msg::Twist & message);

  /* 概要：ROS 2非依存のVelocityをROS 2のTwistへ変換する。 */
  static geometry_msgs::msg::Twist toTwist(const Velocity & velocity);

  using SteadyTime = std::chrono::steady_clock::time_point;

  NodeSettings settings;
  SafetyGate safetyGate;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vlmCmdVelSubscriber;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSubscriber;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statePublisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debugPublisher;
  rclcpp::TimerBase::SharedPtr pollingTimer;

  std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster;

  std::optional<Velocity> vlmCmdVel;
  SteadyTime vlmCmdVelTime{};
  std::vector<Point3> obstacleVoxels;
  bool hasCloud{false};
  SteadyTime cloudTime{};
  std::string cloudError{"点群がまだ来ていません"};
  std::string cloudFrameUsed{};
  SteadyTime lastDebugTime{};
  std::string lastState{};      // ログに状態の変わり目だけを出すため
  bool wasIdle{false};
  SteadyTime idleSince{};       // 指令が途絶えた（IDLE になった）時刻
  bool lastPublishing{true};    // ログに、出す・黙るの変わり目だけを出すため
};

#endif
