#pragma once

#include "nimo_arm/arm_control.hpp"
#include "nimo_arm/types.hpp"
#include "nimo_arm_msgs/msg/arm_status.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <array>
#include <string>

namespace nimo_arm_ros {

inline constexpr std::array<const char*, 14> kJointNames = {
  "Joint1_L", "Joint2_L", "Joint3_L", "Joint4_L", "Joint5_L", "Joint6_L", "Joint7_L",
  "Joint1_R", "Joint2_R", "Joint3_R", "Joint4_R", "Joint5_R", "Joint6_R", "Joint7_R",
};

inline uint8_t ToMsg(nimo_arm::OpState s)
{
  return static_cast<uint8_t>(s);
}

inline uint8_t ToMsg(nimo_arm::ControlMode m)
{
  return static_cast<uint8_t>(m);
}

inline uint8_t ToMsg(nimo_arm::MotionKind k)
{
  return static_cast<uint8_t>(k);
}

inline uint8_t ToMsg(nimo_arm::ErrorCode e)
{
  return static_cast<uint8_t>(e);
}

inline uint8_t ToMsg(nimo_arm::RejectReason r)
{
  return static_cast<uint8_t>(r);
}

inline uint8_t ToMsg(nimo_arm::AlarmSource a)
{
  return static_cast<uint8_t>(a);
}

inline uint8_t ToMsg(nimo_arm::BrakeCommand b)
{
  return static_cast<uint8_t>(b);
}

inline nimo_arm::ControlMode ToControlMode(uint8_t m)
{
  return static_cast<nimo_arm::ControlMode>(m);
}

inline nimo_arm::BrakeCommand ToBrake(uint8_t b)
{
  return static_cast<nimo_arm::BrakeCommand>(b);
}

inline nimo_arm::JogCartAxis ToJogCartAxis(uint8_t a)
{
  return static_cast<nimo_arm::JogCartAxis>(a);
}

inline nimo_arm::V7d ToV7(const std::array<double, 7> & q)
{
  nimo_arm::V7d v;
  for (int i = 0; i < 7; ++i) {
    v(i) = q[static_cast<size_t>(i)];
  }
  return v;
}

inline nimo_arm::V7d ToV7(const double * q)
{
  nimo_arm::V7d v;
  for (int i = 0; i < 7; ++i) {
    v(i) = q[i];
  }
  return v;
}

template<typename Seq>
inline nimo_arm::V7d ToV7Seq(const Seq & q)
{
  nimo_arm::V7d v;
  for (int i = 0; i < 7; ++i) {
    v(i) = q[static_cast<size_t>(i)];
  }
  return v;
}

inline void FillArray(const nimo_arm::V7d & v, double * out)
{
  for (int i = 0; i < 7; ++i) {
    out[i] = v(i);
  }
}

inline nimo_arm::Pose ToPose(const geometry_msgs::msg::Pose & p)
{
  nimo_arm::Pose out;
  out.pos = nimo_arm::V3d(p.position.x, p.position.y, p.position.z);
  out.quat = nimo_arm::Quat(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  out.quat.normalize();
  return out;
}

inline geometry_msgs::msg::Pose ToMsgPose(const nimo_arm::Pose & p)
{
  geometry_msgs::msg::Pose out;
  out.position.x = p.pos.x();
  out.position.y = p.pos.y();
  out.position.z = p.pos.z();
  out.orientation.w = p.quat.w();
  out.orientation.x = p.quat.x();
  out.orientation.y = p.quat.y();
  out.orientation.z = p.quat.z();
  return out;
}

inline void FillJointState(
  sensor_msgs::msg::JointState & js,
  const nimo_arm::RobotState & left,
  const nimo_arm::RobotState & right,
  const rclcpp::Time & stamp)
{
  js.header.stamp = stamp;
  js.name.assign(kJointNames.begin(), kJointNames.end());
  js.position.resize(14);
  js.velocity.resize(14);
  js.effort.resize(14);
  for (int i = 0; i < 7; ++i) {
    js.position[static_cast<size_t>(i)] = left.joint.q(i);
    js.velocity[static_cast<size_t>(i)] = left.joint.dq(i);
    js.effort[static_cast<size_t>(i)] = left.joint.tau(i);
    js.position[static_cast<size_t>(i + 7)] = right.joint.q(i);
    js.velocity[static_cast<size_t>(i + 7)] = right.joint.dq(i);
    js.effort[static_cast<size_t>(i + 7)] = right.joint.tau(i);
  }
}

inline void FillArmJointState(
  sensor_msgs::msg::JointState & js,
  const char * suffix,
  const nimo_arm::JointState & joint,
  const rclcpp::Time & stamp)
{
  js.header.stamp = stamp;
  js.name.resize(7);
  js.position.resize(7);
  js.velocity.resize(7);
  js.effort.resize(7);
  for (int i = 0; i < 7; ++i) {
    js.name[static_cast<size_t>(i)] = std::string("Joint") + std::to_string(i + 1) + suffix;
    js.position[static_cast<size_t>(i)] = joint.q(i);
    js.velocity[static_cast<size_t>(i)] = joint.dq(i);
    js.effort[static_cast<size_t>(i)] = joint.tau(i);
  }
}

inline void FillArmStatus(nimo_arm_msgs::msg::ArmStatus & msg, const nimo_arm::Arm & arm, const rclcpp::Time & stamp)
{
  msg.header.stamp = stamp;
  msg.op_state = ToMsg(arm.GetOpState());
  msg.control_mode = ToMsg(arm.GetControlMode());
  msg.control_mode_settled = arm.ControlModeSettled();
  msg.motion_kind = ToMsg(arm.GetMotionKind());
  msg.error_code = ToMsg(arm.GetErrorCode());
  msg.error_message = nimo_arm::ErrorCodeMessage(arm.GetErrorCode());
  msg.reject_reason = ToMsg(arm.GetLastRejectReason());
  msg.reject_message = nimo_arm::RejectReasonMessage(arm.GetLastRejectReason());
  msg.alarm_source = ToMsg(arm.GetAlarmSource());
  msg.is_alarm = arm.IsAlarm();
  msg.is_joint_vel_alarm = arm.IsJointVelAlarm();
  msg.motion_done = arm.IsMotionDone();
  msg.brake = ToMsg(arm.GetBrakeCommand());

  // Task space SE3×φ + joints: resp/ref filled together from library RobotState.
  nimo_arm::RobotState resp{};
  nimo_arm::RobotState ref{};
  arm.GetResp(resp);
  arm.GetRef(ref);
  for (int i = 0; i < nimo_arm::kDof; ++i) {
    msg.q_resp[static_cast<size_t>(i)] = resp.joint.q(i);
    msg.q_ref[static_cast<size_t>(i)] = ref.joint.q(i);
  }
  msg.tcp_resp = ToMsgPose(resp.tcp);
  msg.tcp_ref = ToMsgPose(ref.tcp);
  msg.phi_resp = resp.phi;
  msg.phi_ref = ref.phi;
  msg.phi_resp_valid = resp.phi_valid;
  msg.phi_ref_valid = ref.phi_valid;
}

}  // namespace nimo_arm_ros
