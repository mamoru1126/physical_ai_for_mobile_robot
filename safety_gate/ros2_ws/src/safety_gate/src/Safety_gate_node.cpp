#include "safety_gate/Safety_gate_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <sstream>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <yaml-cpp/yaml.h>

namespace
{
constexpr char kNodeName[] = "safety_gate_node";

double secondsSince(std::chrono::steady_clock::time_point t)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
}

/* 小数 2 桁に丸めて文字列にする（末尾の 0 は落とす. 0.50 → 0.5、-0.00 → 0） */
std::string fmt2(double value)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.2f", value);
  std::string s(buffer);
  if (s.find('.') != std::string::npos) {
    while (!s.empty() && s.back() == '0') {
      s.pop_back();
    }
    if (!s.empty() && s.back() == '.') {
      s.pop_back();
    }
  }
  if (s == "-0") {
    s = "0";
  }
  return s;
}

std::string jsonString(const std::string & text)
{
  std::string out = "\"";
  for (const char ch : text) {
    switch (ch) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      default: out += ch;
    }
  }
  return out + "\"";
}

std::string base64(const std::vector<std::uint8_t> & bytes)
{
  static const char table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((bytes.size() + 2) / 3 * 4);
  std::size_t i = 0;
  for (; i + 2 < bytes.size(); i += 3) {
    const std::uint32_t n = (bytes[i] << 16) | (bytes[i + 1] << 8) | bytes[i + 2];
    out += table[(n >> 18) & 63];
    out += table[(n >> 12) & 63];
    out += table[(n >> 6) & 63];
    out += table[n & 63];
  }
  if (i + 1 == bytes.size()) {
    const std::uint32_t n = bytes[i] << 16;
    out += table[(n >> 18) & 63];
    out += table[(n >> 12) & 63];
    out += "==";
  } else if (i + 2 == bytes.size()) {
    const std::uint32_t n = (bytes[i] << 16) | (bytes[i + 1] << 8);
    out += table[(n >> 18) & 63];
    out += table[(n >> 12) & 63];
    out += table[(n >> 6) & 63];
    out += '=';
  }
  return out;
}

/* 点を表示のマスの番号（x 方向の番号 × 一辺のマスの数 + y 方向の番号）にし、
 * 重なりを除いて、小さい順に little endian の整数（2 か 4 バイト）で並べる */
std::vector<std::uint8_t> encodeCells(
  const std::vector<Point2> & points, double half, double cell, std::uint32_t width,
  std::size_t bytes_per_index)
{
  std::vector<std::uint32_t> indices;
  indices.reserve(points.size());
  for (const auto & p : points) {
    const double fx = (p.x + half) / cell;
    const double fy = (p.y + half) / cell;
    if (fx < 0.0 || fy < 0.0) {
      continue;
    }
    const auto ix = static_cast<std::uint32_t>(fx);
    const auto iy = static_cast<std::uint32_t>(fy);
    if (ix >= width || iy >= width) {
      continue;
    }
    indices.push_back(ix * width + iy);
  }
  std::sort(indices.begin(), indices.end());
  indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
  std::vector<std::uint8_t> bytes;
  bytes.reserve(indices.size() * bytes_per_index);
  for (const auto index : indices) {
    for (std::size_t b = 0; b < bytes_per_index; ++b) {
      bytes.push_back(static_cast<std::uint8_t>((index >> (8 * b)) & 0xFF));
    }
  }
  return bytes;
}

std::string stripSlash(const std::string & frame)
{
  std::size_t start = 0;
  while (start < frame.size() && frame[start] == '/') {
    ++start;
  }
  return frame.substr(start);
}

template<typename T>
void readIfPresent(const YAML::Node & node, const char * key, T & value)
{
  if (node && node[key]) {
    value = node[key].as<T>();
  }
}
}  // namespace

/* 概要：設定ファイルから通信まわりの設定値を読み込む。 */
NodeSettings SafetyGateNode::loadSettings(const std::string & config_file_path)
{
  NodeSettings s;
  const YAML::Node root = YAML::LoadFile(config_file_path);
  const YAML::Node topics = root["topics"];
  readIfPresent(topics, "cmd_in", s.cmdInTopic);
  readIfPresent(topics, "cloud", s.cloudTopic);
  readIfPresent(topics, "cmd_out", s.cmdOutTopic);
  readIfPresent(topics, "state", s.stateTopic);
  readIfPresent(topics, "debug", s.debugTopic);
  const YAML::Node frames = root["frames"];
  readIfPresent(frames, "base", s.baseFrame);
  readIfPresent(frames, "cloud", s.cloudFrame);
  const YAML::Node tf = root["tf"];
  readIfPresent(tf, "publish_static", s.publishStaticTf);
  readIfPresent(tf, "parent", s.tfParent);
  readIfPresent(tf, "child", s.tfChild);
  readIfPresent(tf, "xyz", s.tfXyz);
  readIfPresent(tf, "rpy", s.tfRpy);
  const YAML::Node safety = root["safety"];
  readIfPresent(safety, "cmd_timeout", s.cmdTimeout);
  readIfPresent(safety, "cloud_timeout", s.cloudTimeout);
  readIfPresent(safety, "rate", s.rate);
  readIfPresent(safety, "publish_when_idle", s.publishWhenIdle);
  readIfPresent(safety, "idle_tail", s.idleTail);
  const YAML::Node web = root["web"];
  readIfPresent(web, "rate", s.webRate);
  readIfPresent(web, "cell_size", s.cellSize);

  s.baseFrame = stripSlash(s.baseFrame);
  s.cloudFrame = stripSlash(s.cloudFrame);
  s.tfParent = stripSlash(s.tfParent);
  s.tfChild = stripSlash(s.tfChild);
  if (s.tfXyz.size() != 3U || s.tfRpy.size() != 3U) {
    throw std::runtime_error("tf の xyz と rpy は 3 つの数で書いてください");
  }
  if (!(s.rate > 0.0) || !(s.webRate > 0.0) || !(s.cellSize > 0.0)) {
    throw std::runtime_error("safety.rate、web.rate、web.cell_size は 0 より大きくしてください");
  }
  if (!(s.cmdTimeout > 0.0) || !(s.cloudTimeout > 0.0)) {
    throw std::runtime_error("cmd_timeout と cloud_timeout は 0 より大きくしてください");
  }
  if (s.idleTail < 0.0) {
    throw std::runtime_error("idle_tail は 0 以上にしてください");
  }
  return s;
}

/* 概要：設定ファイルを読み、Publisher、Subscriber、TF、タイマおよび安全機能ロジックを初期化する。 */
SafetyGateNode::SafetyGateNode()
: Node(kNodeName)
{
  const std::string default_path =
    ament_index_cpp::get_package_share_directory("safety_gate") + "/config/config.yaml";
  const std::string config_path = declare_parameter<std::string>("config_file", default_path);
  RCLCPP_INFO(get_logger(), "設定ファイル: %s", config_path.c_str());
  settings = loadSettings(config_path);
  safetyGate.loadConfig(config_path);

  tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
  tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
  if (settings.publishStaticTf) {
    staticTfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = settings.tfParent;
    t.child_frame_id = settings.tfChild;
    t.transform.translation.x = settings.tfXyz[0];
    t.transform.translation.y = settings.tfXyz[1];
    t.transform.translation.z = settings.tfXyz[2];
    tf2::Quaternion q;
    q.setRPY(settings.tfRpy[0], settings.tfRpy[1], settings.tfRpy[2]);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    staticTfBroadcaster->sendTransform(t);
    RCLCPP_INFO(
      get_logger(), "TF を出します: %s → %s", settings.tfParent.c_str(), settings.tfChild.c_str());
  }

  // 指令も点群も、送り手が best effort でも reliable でも受けられるように best effort で受ける
  vlmCmdVelSubscriber = create_subscription<geometry_msgs::msg::Twist>(
    settings.cmdInTopic, rclcpp::QoS(10).best_effort(),
    std::bind(&SafetyGateNode::vlmCmdVelSubscriptionCallback, this, std::placeholders::_1));
  pointCloudSubscriber = create_subscription<sensor_msgs::msg::PointCloud2>(
    settings.cloudTopic, rclcpp::SensorDataQoS(),
    std::bind(&SafetyGateNode::pointCloudSubscriptionCallback, this, std::placeholders::_1));
  cmdVelPublisher = create_publisher<geometry_msgs::msg::Twist>(settings.cmdOutTopic, 10);
  statePublisher = create_publisher<std_msgs::msg::String>(settings.stateTopic, 10);
  debugPublisher = create_publisher<std_msgs::msg::String>(settings.debugTopic, 1);

  const auto period = std::chrono::duration<double>(1.0 / settings.rate);
  pollingTimer = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SafetyGateNode::timerCallback, this));

  RCLCPP_INFO(
    get_logger(), "受ける: %s、%s  出す: %s（%s 座標系で判定）", settings.cmdInTopic.c_str(),
    settings.cloudTopic.c_str(), settings.cmdOutTopic.c_str(), settings.baseFrame.c_str());
  if (settings.publishWhenIdle) {
    RCLCPP_INFO(get_logger(), "指令が来ていない間も 0 を出し続けます");
  } else {
    RCLCPP_INFO(
      get_logger(), "指令が来ていない間は、0 を %.2f 秒出してから黙ります（元のシステムと /cmd_vel を使い分ける）",
      settings.idleTail);
    lastPublishing = false;
  }
}

/* 概要：最新の点群と速度指令値から安全な速度指令値を生成し配信する。 */
void SafetyGateNode::timerCallback()
{
  Velocity input{};
  GateResult result;
  std::string state;
  std::string reason;

  const bool cmd_fresh = vlmCmdVel && secondsSince(vlmCmdVelTime) <= settings.cmdTimeout;
  const bool cloud_fresh = hasCloud && secondsSince(cloudTime) <= settings.cloudTimeout;
  if (!cmd_fresh) {
    state = "IDLE";
    reason = vlmCmdVel ? "指令が途切れました" : "指令がまだ来ていません";
  } else {
    input = *vlmCmdVel;
    if (!cloud_fresh) {
      state = "STOP";
      reason = hasCloud ? "点群が途切れました" : cloudError;
    } else {
      result = safetyGate.generateSafeCmdVel(obstacleVoxels, input);
      state = toString(result.state);
      switch (result.state) {
        case GateState::Pass: reason = "進む先に障害物はありません"; break;
        case GateState::Slow: reason = "進む先に障害物があるので、速度を落としています"; break;
        case GateState::Stop: reason = "進む先のすぐ近くに障害物があります"; break;
        case GateState::Idle: reason = "指令が 0 です"; break;
      }
    }
  }
  if (state != "PASS" && state != "SLOW") {
    result.velocity = Velocity{};
  }
  // 指令が途絶えたら（IDLE）、設定によっては 0 を少し出してから黙る
  if (!cmd_fresh && !wasIdle) {
    idleSince = std::chrono::steady_clock::now();
  }
  wasIdle = !cmd_fresh;
  bool publishing = true;
  if (!settings.publishWhenIdle && !cmd_fresh) {
    publishing = vlmCmdVel.has_value() && secondsSince(idleSince) <= settings.idleTail;
  }
  if (publishing) {
    cmdVelPublisher->publish(toTwist(result.velocity));
  }
  if (publishing != lastPublishing) {
    RCLCPP_INFO(
      get_logger(), "%s", publishing ? "速度指令を出し始めました" :
      "指令が来ていないので、速度指令を出すのをやめました");
    lastPublishing = publishing;
  }
  if (state != lastState) {
    RCLCPP_INFO(
      get_logger(), "状態: %s → %s（%s）", lastState.empty() ? "(起動)" : lastState.c_str(),
      state.c_str(), reason.c_str());
    lastState = state;
  }

  std_msgs::msg::String state_message;
  state_message.data = state + " " + reason;
  statePublisher->publish(state_message);

  if (secondsSince(lastDebugTime) >= 1.0 / settings.webRate) {
    lastDebugTime = std::chrono::steady_clock::now();
    publishDebug(result, input, state, reason);
  }
}

/* 概要：VLMから受信した速度指令値を、受け取った時刻とともに保存する。 */
void SafetyGateNode::vlmCmdVelSubscriptionCallback(
  geometry_msgs::msg::Twist::ConstSharedPtr message)
{
  vlmCmdVel = toVelocity(*message);
  vlmCmdVelTime = std::chrono::steady_clock::now();
}

/* 概要：LIDARから受信した点群を base_link に直し、障害物のボクセルにして保存する。 */
void SafetyGateNode::pointCloudSubscriptionCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  std::vector<Point3> points;
  std::string error;
  if (!transformPointCloud(*message, points, error)) {
    hasCloud = false;  // 直せない点群では判定しない（止める）
    cloudError = error;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.c_str());
    return;
  }
  obstacleVoxels = safetyGate.extractObstacleVoxels(points);
  hasCloud = true;
  cloudTime = std::chrono::steady_clock::now();
}

/* 概要：点群を TF で base_link 座標系に直す。 */
bool SafetyGateNode::transformPointCloud(
  const sensor_msgs::msg::PointCloud2 & message, std::vector<Point3> & points, std::string & error)
{
  const std::string source =
    settings.cloudFrame.empty() ? stripSlash(message.header.frame_id) : settings.cloudFrame;
  if (source.empty()) {
    error = "点群の座標系の名前が空です（frames.cloud に書いてください）";
    return false;
  }
  cloudFrameUsed = source;

  // 取り付けは動かない前提で、いちばん新しい TF を使う
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tfBuffer->lookupTransform(settings.baseFrame, source, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    error = "TF がありません（" + settings.baseFrame + " ← " + source + "）: " + e.what();
    return false;
  }
  const auto & tr = transform.transform.translation;
  const auto & ro = transform.transform.rotation;
  const tf2::Matrix3x3 m(tf2::Quaternion(ro.x, ro.y, ro.z, ro.w));

  const sensor_msgs::msg::PointField * field_x = nullptr;
  for (const auto & field : message.fields) {
    if (field.name == "x") {
      field_x = &field;
    }
  }
  if (!field_x) {
    error = "点群に x、y、z がありません";
    return false;
  }

  const std::size_t count = static_cast<std::size_t>(message.width) * message.height;
  points.clear();
  points.reserve(count);
  auto push = [&](double x, double y, double z) {
      points.push_back(Point3{
        static_cast<float>(m[0][0] * x + m[0][1] * y + m[0][2] * z + tr.x),
        static_cast<float>(m[1][0] * x + m[1][1] * y + m[1][2] * z + tr.y),
        static_cast<float>(m[2][0] * x + m[2][1] * y + m[2][2] * z + tr.z)});
    };
  try {
    if (field_x->datatype == sensor_msgs::msg::PointField::FLOAT64) {
      sensor_msgs::PointCloud2ConstIterator<double> ix(message, "x"), iy(message, "y"),
      iz(message, "z");
      for (; ix != ix.end(); ++ix, ++iy, ++iz) {
        push(*ix, *iy, *iz);
      }
    } else {
      sensor_msgs::PointCloud2ConstIterator<float> ix(message, "x"), iy(message, "y"),
      iz(message, "z");
      for (; ix != ix.end(); ++ix, ++iy, ++iz) {
        push(*ix, *iy, *iz);
      }
    }
  } catch (const std::exception & e) {
    error = std::string("点群が読めません: ") + e.what();
    return false;
  }
  return true;
}

/* 概要：WebUI 用の情報（小数 2 桁、点は表示のマスの番号）を JSON にして配信する。 */
void SafetyGateNode::publishDebug(
  const GateResult & result, const Velocity & input, const std::string & state,
  const std::string & reason)
{
  const auto & config = safetyGate.getConfig();
  const double half = config.viewHalfSize;
  const double cell = settings.cellSize;
  const auto width = static_cast<std::uint32_t>(std::ceil(2.0 * half / cell - 1e-9));
  const std::size_t bytes_per_index =
    static_cast<std::uint64_t>(width) * width <= 65536U ? 2U : 4U;

  std::vector<Point2> all;
  all.reserve(obstacleVoxels.size());
  if (hasCloud) {
    for (const auto & v : obstacleVoxels) {
      all.push_back(Point2{v.x, v.y});
    }
  }
  std::vector<Point2> threats;
  for (const auto index : result.threatIndices) {
    if (index < obstacleVoxels.size()) {
      threats.push_back(Point2{obstacleVoxels[index].x, obstacleVoxels[index].y});
    }
  }

  std::ostringstream json;
  json << "{\"state\":" << jsonString(state) << ",\"reason\":" << jsonString(reason);
  json << ",\"in\":[" << fmt2(input.linearX) << "," << fmt2(input.linearY) << "," <<
    fmt2(input.angularZ) << "]";
  json << ",\"out\":[" << fmt2(result.velocity.linearX) << "," << fmt2(result.velocity.linearY) <<
    "," << fmt2(result.velocity.angularZ) << "]";
  json << ",\"scale\":" << fmt2(result.scale) << ",\"tc\":" << fmt2(result.collisionTime);
  json << ",\"fp\":[";
  for (std::size_t i = 0; i < config.footprint.size(); ++i) {
    json << (i ? "," : "") << "[" << fmt2(config.footprint[i].x) << "," <<
      fmt2(config.footprint[i].y) << "]";
  }
  json << "],\"margin\":" << fmt2(config.margin);
  json << ",\"pub\":" << (lastPublishing ? "true" : "false");
  json << ",\"poses\":[";
  bool first = true;
  for (std::size_t i = 0; i < result.predictedPoses.size(); i += 2) {
    const auto & p = result.predictedPoses[i];
    json << (first ? "" : ",") << "[" << fmt2(p.x) << "," << fmt2(p.y) << "," << fmt2(p.yaw) <<
      "]";
    first = false;
  }
  json << "],\"half\":" << fmt2(half) << ",\"cell\":" << fmt2(cell) << ",\"w\":" << width <<
    ",\"bpi\":" << bytes_per_index;
  json << ",\"obs\":\"" << base64(encodeCells(all, half, cell, width, bytes_per_index)) << "\"";
  json << ",\"thr\":\"" << base64(encodeCells(threats, half, cell, width, bytes_per_index)) <<
    "\"";
  json << ",\"nvox\":" << (hasCloud ? obstacleVoxels.size() : 0U);
  json << ",\"age_cloud\":" << (hasCloud ? fmt2(secondsSince(cloudTime)) : "null");
  json << ",\"age_cmd\":" << (vlmCmdVel ? fmt2(secondsSince(vlmCmdVelTime)) : "null");
  json << ",\"frames\":[" << jsonString(settings.baseFrame) << "," <<
    jsonString(cloudFrameUsed) << "]";
  json << ",\"topics\":[" << jsonString(settings.cmdInTopic) << "," <<
    jsonString(settings.cloudTopic) << "," << jsonString(settings.cmdOutTopic) << "]}";

  std_msgs::msg::String message;
  message.data = json.str();
  debugPublisher->publish(message);
}

/* 概要：ROS 2のTwistをROS 2非依存のVelocityへ変換する。 */
Velocity SafetyGateNode::toVelocity(const geometry_msgs::msg::Twist & message)
{
  return Velocity{message.linear.x, message.linear.y, message.linear.z,
    message.angular.x, message.angular.y, message.angular.z};
}

/* 概要：ROS 2非依存のVelocityをROS 2のTwistへ変換する。 */
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

/* 概要：SafetyGateNodeを起動し、ROS 2のイベント処理を実行する。 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SafetyGateNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger(kNodeName), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
