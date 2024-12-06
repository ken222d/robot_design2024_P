// Copyright 2024 Keitaro Nakamura
// SPDX-FileCopyrightText: 2024 Haruto Yamamoto, Akira Matsumoto
// SPDX-License-Identifier: Apache 2.0
// このファイルは元々Keitaro NakamuraとRyotaro Karikomiによって作成され、その後Haruto YamamotoとAkira Matsumoyoによって変更されました。

// 設計方針
//        構成
//            void move_specific_joint
//                特定の関節を現在の角度からn°動かす
//            void stamping 
//                変数の定義
//                init_pose
//                xyz座標を移動するfor文
//                一時停止
//                void move_specific_jointで第2関節を-5°動かす
//                一時停止
//                void move_specific_jointで第2関節を5°動かす
//                init_pose
//                rclcpp::shutdown();
//
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <iostream> // std::cout を使うために必要

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class PickAndPlaceTf : public rclcpp::Node
{
public:
  PickAndPlaceTf(
    rclcpp::Node::SharedPtr move_group_arm_node,
    rclcpp::Node::SharedPtr move_group_gripper_node)
  : Node("pick_and_move_tf_node")
  {
    using namespace std::placeholders;
    move_group_arm_ = std::make_shared<MoveGroupInterface>(move_group_arm_node, "arm");
    move_group_arm_->setMaxVelocityScalingFactor(0.7);
    move_group_arm_->setMaxAccelerationScalingFactor(0.7);

    move_group_gripper_ = std::make_shared<MoveGroupInterface>(move_group_gripper_node, "gripper");
    move_group_gripper_->setMaxVelocityScalingFactor(1.0);
    move_group_gripper_->setMaxAccelerationScalingFactor(1.0);

    // SRDFに定義されている"home"の姿勢にする
    move_group_arm_->setNamedTarget("home");
    move_group_arm_->move();

    // 可動範囲を制限する
    moveit_msgs::msg::Constraints constraints;
    constraints.name = "arm_constraints";

    moveit_msgs::msg::JointConstraint joint_constraint;
    joint_constraint.joint_name = "crane_x7_lower_arm_fixed_part_joint";
    joint_constraint.position = 0.0;
    joint_constraint.tolerance_above = angles::from_degrees(90);  // ここ以下4つの角度を増やすと可動域が増えます。ただし大きい動きになることがあるので注意が必要です。
    joint_constraint.tolerance_below = angles::from_degrees(90);
    joint_constraint.weight = 1.0;
    constraints.joint_constraints.push_back(joint_constraint);

    joint_constraint.joint_name = "crane_x7_upper_arm_revolute_part_twist_joint";
    joint_constraint.position = 0.0;
    joint_constraint.tolerance_above = angles::from_degrees(90);
    joint_constraint.tolerance_below = angles::from_degrees(90);
    joint_constraint.weight = 0.8;
    constraints.joint_constraints.push_back(joint_constraint);

    // move_group_arm_->setPathConstraints(constraints);

    // 真上から見下ろす撮影姿勢
    // crane_x7_upper_arm_revolute_part_rotate_jointにかかる負荷が高いため長時間の使用に向いておりません
    // control_arm(0.15, 0.0, 0.3, -180, 0, 90);

    // 関節への負荷が低い撮影姿勢
    init_pose();

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&PickAndPlaceTf::on_timer, this));
  }

private:
  void on_timer()
  {
    // target_0のtf位置姿勢を取得
    geometry_msgs::msg::TransformStamped tf_msg;

    try {
      tf_msg = tf_buffer_->lookupTransform(
        "base_link", "target_0",
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform base_link to target: %s",
        ex.what());
      return;
    }

    rclcpp::Time now = this->get_clock()->now();
    const std::chrono::nanoseconds FILTERING_TIME = 2s;
    const std::chrono::nanoseconds STOP_TIME_THRESHOLD = 3s;
    const double DISTANCE_THRESHOLD = 0.01;
    tf2::Stamped<tf2::Transform> tf;
    tf2::convert(tf_msg, tf);
    const auto TF_ELAPSED_TIME = now.nanoseconds() - tf.stamp_.time_since_epoch().count();
    const auto TF_STOP_TIME = now.nanoseconds() - tf_past_.stamp_.time_since_epoch().count();
    const double TARGET_Z_MIN_LIMIT = 0.04;

    // 現在時刻から2秒以内に受け取ったtfを使用
    if (TF_ELAPSED_TIME < FILTERING_TIME.count()) {
      double tf_diff = (tf_past_.getOrigin() - tf.getOrigin()).length();
      // 把持対象の位置が停止していることを判定
      if (tf_diff < DISTANCE_THRESHOLD) {
        // 把持対象が3秒以上停止している場合ピッキング動作開始
        if (TF_STOP_TIME > STOP_TIME_THRESHOLD.count()) {
          // 把持対象が低すぎる場合は把持位置を調整
          if (tf.getOrigin().z() < TARGET_Z_MIN_LIMIT) {
            tf.getOrigin().setZ(TARGET_Z_MIN_LIMIT);
          }
          stamping(tf.getOrigin());
        }
      } else {
        tf_past_ = tf;
      }
    }
  }

  void init_pose()
  {
    std::vector<double> joint_values;
    joint_values.push_back(angles::from_degrees(0.0));
    joint_values.push_back(angles::from_degrees(85));
    joint_values.push_back(angles::from_degrees(0.0));
    joint_values.push_back(angles::from_degrees(-160));
    joint_values.push_back(angles::from_degrees(0.0));
    joint_values.push_back(angles::from_degrees(-50));
    joint_values.push_back(angles::from_degrees(90));
    move_group_arm_->setJointValueTarget(joint_values);
    move_group_arm_->move();
  }

  // 特定の関節の角度を動かす関数
  void move_specific_joint(int joint_index, double relative_angle_deg)
  {
    // 現在の関節角度を取得
    std::vector<double> current_joint_values = move_group_arm_->getCurrentJointValues();

    // 指定した関節の相対角度を計算
    if (joint_index >= 0 && joint_index < current_joint_values.size())
    {
        current_joint_values[joint_index] += angles::from_degrees(relative_angle_deg);

        // 新しい関節値を設定して動かす
        move_group_arm_->setJointValueTarget(current_joint_values);
        move_group_arm_->move();
    }
    else
    {
        std::cerr << "Invalid joint index: " << joint_index << std::endl;
    }
  }


  void stamping(tf2::Vector3 target_position)
  {
    const double GRIPPER_DEFAULT = 0.0;
    const double GRIPPER_OPEN = angles::from_degrees(60.0);
    const double GRIPPER_CLOSE = angles::from_degrees(15.0);
    const int move_steps = 10;

    // 現在位置を取得
    geometry_msgs::msg::Pose current_pose = move_group_arm_->getCurrentPose().pose;

    // ハンドを閉める
    control_gripper(GRIPPER_DEFAULT);

    // ホームポジションへ移動（重複している可能性があり, 後から削除する可能性が高い）
    init_pose();

    // 経路を10分割して(target_position.x(), target_position.y(), 0.1)まで移動
    for (int i = 1; i <= move_steps; ++i) {
        geometry_msgs::msg::Pose intermediate_pose;
        intermediate_pose.position.x = current_pose.position.x + (target_position.x() - current_pose.position.x) * i / move_steps;
        intermediate_pose.position.y = current_pose.position.y + (target_position.y() - current_pose.position.y) * i / move_steps;
        intermediate_pose.position.z = current_pose.position.z + (0.2 - current_pose.position.z) * i / move_steps;
        intermediate_pose.orientation = current_pose.orientation; // 同じ姿勢を維持

        control_arm(intermediate_pose.position.x, intermediate_pose.position.y, intermediate_pose.position.z, 90, 0, 90);
        // 現在のループカウントを表示
        std::cout << "Move steps loop iteration: " << i << "/" << move_steps << std::endl;
    }

    rclcpp::sleep_for(std::chrono::seconds(2)); 

    std::cout << "目標の座標に到達しました" << std::endl;

    // ハンコを押す動作を-1°ずつ5回のループで実行
    for (int i = 0; i < 5; ++i) {
        move_specific_joint(1, -1); // -1°ずつ動かす
        std::cout << "Step " << (i + 1) << ": Joint moved by -1°" << std::endl;
    }

    rclcpp::sleep_for(std::chrono::seconds(5));

    std::cout << "ハンコを押す動作が完了しました" << std::endl;

    // ハンコを離す動作を0.5°ずつ5回のループで実行
    for (int i = 0; i < 5; ++i) {
        move_specific_joint(1, 1); // 0.5°ずつ動かす
        std::cout << "Step " << (i + 1) << ": Joint moved by 1°" << std::endl;
    }

    // 初期姿勢に戻る
    init_pose();

    // ループを回避するため動作終了
    rclcpp::shutdown();
  }

  // グリッパ制御
  void control_gripper(const double angle)
  {
    auto joint_values = move_group_gripper_->getCurrentJointValues();
    joint_values[0] = angle;
    move_group_gripper_->setJointValueTarget(joint_values);
    move_group_gripper_->move();
  }

  // アーム制御
  void control_arm(
    const double x, const double y, const double z,
    const double roll, const double pitch, const double yaw)
  {
    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion q;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    q.setRPY(angles::from_degrees(roll), angles::from_degrees(pitch), angles::from_degrees(yaw));
    target_pose.orientation = tf2::toMsg(q);
    move_group_arm_->setPoseTarget(target_pose);
    move_group_arm_->move();
  }

  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  tf2::Stamped<tf2::Transform> tf_past_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto pick_and_move_tf_node = std::make_shared<PickAndPlaceTf>(
    move_group_arm_node,
    move_group_gripper_node);
  exec.add_node(pick_and_move_tf_node);
  exec.add_node(move_group_arm_node);
  exec.add_node(move_group_gripper_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

