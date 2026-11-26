#ifndef SAFETY_GATE_HPP_
#define SAFETY_GATE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

/* 点（m）. 点群は base_link 座標系に直してから渡す */
struct Point3
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

/* 平面の点（m）. 足形の頂点などに使う */
struct Point2
{
  double x{0.0};
  double y{0.0};
};

/* 平面の姿勢（m、rad）. 予測したロボットの位置と向き */
struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Velocity
{
  double linearX{0.0};
  double linearY{0.0};
  double linearZ{0.0};
  double angularX{0.0};
  double angularY{0.0};
  double angularZ{0.0};
};

/* 障害物として見る高さの範囲（base_link の z、m）. 床と天井を除く */
struct HeightRange
{
  double minZ{0.05};
  double maxZ{1.5};
};

/* 安全機能ロジックの設定値 */
struct SafetyGateConfig
{
  std::vector<Point2> footprint{{0.35, 0.30}, {0.35, -0.30}, {-0.35, -0.30}, {-0.35, 0.30}};
  HeightRange heightRange{};
  double viewHalfSize{5.0};             // 点群を残す範囲（ロボットから前後左右、m）
  double voxelSize{0.05};               // ボクセルの大きさ（m）
  std::uint32_t obstacleVoxelThreshold{3};  // 障害物ありとみなすボクセルの数
  double margin{0.10};                  // 足形のまわりの余裕（m）
  double timeHorizon{2.0};              // 何秒先までの動きを見るか（s）
  double stopTime{0.5};                 // これより早くぶつかるなら止める（s）
  double timeStep{0.1};                 // 動きを予測する刻み（s）
  double approachEpsilon{0.005};        // 「近づく」とみなす距離の差（m）
  double maxLinearX{0.5};               // 速度の上限（m/s、rad/s）
  double maxLinearY{0.5};
  double maxAngularZ{1.0};
};

/* 判定の結果の種類 */
enum class GateState
{
  Pass,   // そのまま通す
  Slow,   // 速度を落として通す
  Stop,   // 止める
  Idle    // 指令が 0（止まっている）
};

/* 判定の結果 */
struct GateResult
{
  Velocity velocity{};                 // 出力する速度指令値
  GateState state{GateState::Idle};
  double scale{0.0};                   // 入力に掛けた倍率（0〜1）
  double collisionTime{-1.0};          // ぶつかるまでの時間（s）. 見る範囲でぶつからなければ負
  std::vector<std::size_t> threatIndices;  // 近づく先にある障害物ボクセルの番号
  std::vector<Pose2> predictedPoses;       // 予測したロボットの姿勢（0 秒から timeHorizon まで）
};

class SafetyGate
{
public:
  /* 概要：安全機能ロジックを既定の設定値で初期化する。
   * 入力：なし
   * 出力：なし
   */
  SafetyGate();

  /* 概要：設定ファイル（YAML）から安全機能ロジックの設定値を読み込む。
   * 入力：config_file_path 設定ファイルのパス
   * 出力：なし（設定値が正しくなければ std::runtime_error を投げる）
   */
  void loadConfig(const std::string & config_file_path);

  /* 概要：設定値を確かめてから使う。
   * 入力：config 設定値
   * 出力：なし（設定値が正しくなければ std::runtime_error を投げる）
   */
  void setConfig(const SafetyGateConfig & config);

  /* 概要：使っている設定値を返す。
   * 入力：なし
   * 出力：設定値
   */
  const SafetyGateConfig & getConfig() const;

  /* 概要：base_link 座標系の点群から、障害物のボクセル（中心の点）を作る。
   * 入力：point_cloud base_link 座標系の点群
   * 出力：範囲内の点をボクセルにまとめた、各ボクセルの中心の点
   */
  std::vector<Point3> extractObstacleVoxels(const std::vector<Point3> & point_cloud) const;

  /* 概要：障害物のボクセルと速度指令値から、安全な速度指令値を生成する。
   *       指令のまま動いたときの足形の動きを予測し、足形（＋余裕）に「近づいて入ってくる」
   *       ボクセルが閾値の数以上になるまでの時間で、止める・速度を落とす・そのまま通すを決める。
   *       遠ざかる動き（障害物から離れる・横をすれ違う）は止めない。
   * 入力：obstacle_voxels base_link 座標系の障害物のボクセル、input_velocity 入力速度指令値
   * 出力：判定の結果
   */
  GateResult generateSafeCmdVel(
    const std::vector<Point3> & obstacle_voxels,
    const Velocity & input_velocity) const;

  /* 概要：点から多角形までの符号付き距離を求める（外側なら正、内側なら負）。
   * 入力：point 点、polygon 多角形の頂点
   * 出力：符号付き距離（m）
   */
  static double signedDistance(const Point2 & point, const std::vector<Point2> & polygon);

  /* 概要：速度指令値のまま t 秒動いたときのロボットの姿勢を求める（はじめの姿勢から見た値）。
   * 入力：velocity 速度指令値（ロボットから見た値）、t 時間（s）
   * 出力：姿勢
   */
  static Pose2 predictPose(const Velocity & velocity, double t);

private:
  /* 概要：点群から、高さの範囲と表示の範囲に入る点だけを抽出する（数値でない点も除く）。
   * 入力：point_cloud base_link 座標系の点群
   * 出力：範囲内の点群
   */
  std::vector<Point3> extractRoiPointCloud(const std::vector<Point3> & point_cloud) const;

  /* 概要：点群をボクセルにまとめ、各ボクセルを中心の点で表す。
   * 入力：point_cloud 点群
   * 出力：各占有ボクセルの中心の点（並びは決まった順）
   */
  std::vector<Point3> voxelizePointCloud(const std::vector<Point3> & point_cloud) const;

  /* 概要：速度指令値を上限で抑える。数値でない成分があれば 0 にする。
   * 入力：velocity 速度指令値
   * 出力：抑えた速度指令値（平面の移動に使わない成分は 0）
   */
  Velocity clampVelocity(const Velocity & velocity) const;

  SafetyGateConfig config;
  double footprintRadius{0.0};  // 足形の中心から最も遠い頂点までの距離
};

/* 概要：判定の結果の種類を文字列にする。
 * 入力：state 判定の結果の種類
 * 出力："PASS"、"SLOW"、"STOP"、"IDLE"
 */
const char * toString(GateState state);

#endif
