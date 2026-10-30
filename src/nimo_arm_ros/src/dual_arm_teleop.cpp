#include "nimo_arm_ros/dual_arm_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

using namespace std::chrono_literals;

namespace nimo_arm_ros
{
namespace
{

bool TriggerHeld(double trigger, bool was_held, double press, double release)
{
  return was_held ? (trigger >= release) : (trigger >= press);
}

}  // namespace

void DualArmNode::BindTeleopTargetPoses()
  {
    if (!get_parameter("subscribe_teleop_poses").as_bool()) {return;}
    teleop_pose_timeout_s_ = get_parameter("teleop_pose_timeout_s").as_double();
    const std::string left_topic = get_parameter("teleop_left_pose_topic").as_string();
    const std::string right_topic = get_parameter("teleop_right_pose_topic").as_string();

    auto bind = [this](const std::string & topic, bool left) {
        auto sub = create_subscription<geometry_msgs::msg::PoseStamped>(
          topic, 10,
          [this, left](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            OnTeleopTargetPose(left, msg);
          });
        stream_subs_.push_back(sub);
      };
    bind(left_topic, true);
    bind(right_topic, false);

    teleop_watchdog_ = create_wall_timer(20ms, [this]() { TeleopPoseWatchdog(); });
    RCLCPP_INFO(
      get_logger(), "teleop poses: %s | %s (timeout=%.3fs)", left_topic.c_str(),
      right_topic.c_str(), teleop_pose_timeout_s_);
  }

void DualArmNode::BindGripperJoy()
  {
    subscribe_gripper_joy_ = get_parameter("subscribe_gripper_joy").as_bool();
    if (!subscribe_gripper_joy_) {return;}
    gripper_press_ = get_parameter("gripper_press").as_double();
    gripper_release_ = get_parameter("gripper_release").as_double();
    const std::string left_topic = get_parameter("gripper_left_joy_topic").as_string();
    const std::string right_topic = get_parameter("gripper_right_joy_topic").as_string();

    auto bind = [this](const std::string & topic, bool left) {
        auto sub = create_subscription<sensor_msgs::msg::Joy>(
          topic, 10,
          [this, left](const sensor_msgs::msg::Joy::SharedPtr msg) {
            OnGripperJoy(left, msg);
          });
        stream_subs_.push_back(sub);
      };
    bind(left_topic, true);
    bind(right_topic, false);

    // Closed pose needs periodic torque refresh (same as test_rt_teleop TickGripper).
    gripper_refresh_timer_ = create_wall_timer(50ms, [this]() { GripperClosedRefresh(); });
    RCLCPP_INFO(
      get_logger(),
      "gripper joy: %s | %s (axes[2] edge toggle, press=%.2f release=%.2f)",
      left_topic.c_str(), right_topic.c_str(), gripper_press_, gripper_release_);
  }

void DualArmNode::OnGripperJoy(bool left, const sensor_msgs::msg::Joy::SharedPtr & msg)
  {
    if (!initialized_ || !msg || msg->axes.size() < 3) {return;}
    const int arm_index = left ? 0 : 1;
    if (!dual_.GripperConnected(arm_index)) {return;}

    GripperLatch & latch = left ? grip_l_ : grip_r_;
    const double trigger = static_cast<double>(msg->axes[2]);
    const bool held = TriggerHeld(trigger, latch.trigger_held, gripper_press_, gripper_release_);
    if (held && !latch.trigger_held) {
      latch.closed = !latch.closed;
      const double ratio = latch.closed ? 1.0 : 0.0;
      if (!dual_.SetGripperGoal(arm_index, ratio)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "SetGripperGoal(%d, %.0f) failed", arm_index,
          ratio);
      } else {
        RCLCPP_INFO(
          get_logger(), "gripper %s → %s", left ? "left" : "right",
          latch.closed ? "close" : "open");
      }
    }
    latch.trigger_held = held;
  }

void DualArmNode::GripperClosedRefresh()
  {
    if (!initialized_) {return;}
    if (grip_l_.closed && dual_.GripperConnected(0)) {
      (void)dual_.SetGripperGoal(0, 1.0);
    }
    if (grip_r_.closed && dual_.GripperConnected(1)) {
      (void)dual_.SetGripperGoal(1, 1.0);
    }
  }

void DualArmNode::BindTeleopTargetJoints()
  {
    if (!get_parameter("subscribe_teleop_joints").as_bool()) {return;}
    const std::string left_topic = get_parameter("teleop_left_joint_topic").as_string();
    const std::string right_topic = get_parameter("teleop_right_joint_topic").as_string();

    auto bind = [this](const std::string & topic, bool left) {
        auto sub = create_subscription<sensor_msgs::msg::JointState>(
          topic, 10,
          [this, left](const sensor_msgs::msg::JointState::SharedPtr msg) {
            OnTeleopTargetJoints(left, msg);
          });
        stream_subs_.push_back(sub);
      };
    bind(left_topic, true);
    bind(right_topic, false);

    RCLCPP_INFO(
      get_logger(), "teleop joints: %s | %s (ServoJ H=%.3fs)", left_topic.c_str(),
      right_topic.c_str(), servoj_interp_time_s_);
  }

bool DualArmNode::ArmUnsafe(const nimo_arm::Arm & arm)
  {
    const auto op = arm.GetOpState();
    return op == nimo_arm::OpState::Fault || op == nimo_arm::OpState::EStop || arm.IsAlarm();
  }

void DualArmNode::StopServoJStreams()
  {
    if (!initialized_) {return;}
    // Soft stop → Ready; ignore reject if already Ready / Fault / EStop.
    (void)dual_.Left().Stop();
    (void)dual_.Right().Stop();
  }

void DualArmNode::SetLastEvent(const std::string & event)
  {
    std::lock_guard<std::mutex> lock(last_event_mu_);
    last_event_ = event;
    ++last_event_seq_;
  }

void DualArmNode::AbortServoJTeleop(const char * reason)
  {
    if (!servoj_teleop_enabled_.exchange(false)) {
      ResetServoJAlign();
      return;
    }
    ResetServoJAlign();
    StopServoJStreams();
    const char * why = reason ? reason : "unknown";
    SetLastEvent(std::string("servoj_abort: ") + why);
    RCLCPP_WARN(get_logger(), "ServoJ teleop aborted: %s", why);
  }

void DualArmNode::MaybeAbortServoJTeleop()
  {
    if (!servoj_teleop_enabled_.load()) {return;}
    if (!initialized_) {
      AbortServoJTeleop("not initialized");
      return;
    }
    if (ArmUnsafe(dual_.Left())) {
      AbortServoJTeleop("left Fault/EStop/Alarm");
      return;
    }
    if (ArmUnsafe(dual_.Right())) {
      AbortServoJTeleop("right Fault/EStop/Alarm");
      return;
    }
    const double to = servoj_joint_timeout_s_;
    if (to <= 0.0) {return;}
    const auto stamp = now();
    auto stale = [&](bool streaming, const rclcpp::Time & t) {
        return streaming && t.nanoseconds() > 0 && (stamp - t).seconds() > to;
      };
    if (stale(servoj_stream_left_.load(), t_servoj_left_)) {
      AbortServoJTeleop("left target_joints timeout");
      return;
    }
    if (stale(servoj_stream_right_.load(), t_servoj_right_)) {
      AbortServoJTeleop("right target_joints timeout");
      return;
    }
  }

bool DualArmNode::EnsureServoJAligned(bool left, const nimo_arm::V7d & q_cmd)
  {
    std::atomic<bool> & need = left ? servoj_need_align_left_ : servoj_need_align_right_;
    std::atomic<bool> & aligning = left ? servoj_aligning_left_ : servoj_aligning_right_;
    std::atomic<bool> & seen_busy =
      left ? servoj_align_seen_busy_left_ : servoj_align_seen_busy_right_;
    nimo_arm::Arm & arm = left ? dual_.Left() : dual_.Right();

    if (ArmUnsafe(arm)) {
      AbortServoJTeleop(left ? "left unsafe during align" : "right unsafe during align");
      return false;
    }

    if (aligning.load()) {
      if (!arm.IsMotionDone()) {
        seen_busy.store(true);
        return false;
      }
      // motion_done stays true until RT consumes pending MovJ — wait for busy edge.
      if (!seen_busy.load()) {
        return false;
      }
      aligning.store(false);
      need.store(false);
      seen_busy.store(false);
      RCLCPP_INFO(get_logger(), "ServoJ align done (%s)", left ? "left" : "right");
      return true;
    }

    if (!need.load()) {return true;}

    const auto op = arm.GetOpState();
    if (op != nimo_arm::OpState::Ready && op != nimo_arm::OpState::Moving) {
      char buf[192];
      std::snprintf(
        buf, sizeof(buf),
        "%s align blocked: op=%u (need Ready after Init+Enable)", left ? "left" : "right",
        static_cast<unsigned>(op));
      AbortServoJTeleop(buf);
      return false;
    }
    // Leftover Moving with done latch can reject a new MovJ — soft-stop first.
    if (op == nimo_arm::OpState::Moving) {
      (void)arm.Stop();
    }

    nimo_arm::RobotState ref{};
    arm.GetRef(ref);
    double max_abs = 0.0;
    for (int i = 0; i < nimo_arm::kDof; ++i) {
      max_abs = std::max(max_abs, std::abs(q_cmd(i) - ref.joint.q(i)));
    }
    if (max_abs <= servoj_align_tol_rad_) {
      need.store(false);
      return true;
    }

    if (!arm.MovJ(q_cmd)) {
      const std::string why = RejectMsg(arm);
      char buf[256];
      std::snprintf(
        buf, sizeof(buf), "%s align MovJ rejected: %s (max|dq|=%.3f)", left ? "left" : "right",
        why.c_str(), max_abs);
      AbortServoJTeleop(buf);
      return false;
    }
    aligning.store(true);
    seen_busy.store(false);
    RCLCPP_INFO(
      get_logger(), "ServoJ align MovJ (%s) max|dq|=%.3f > tol=%.3f", left ? "left" : "right",
      max_abs, servoj_align_tol_rad_);
    return false;
  }

void DualArmNode::ResetServoJAlign()
  {
    servoj_need_align_left_.store(false);
    servoj_need_align_right_.store(false);
    servoj_aligning_left_.store(false);
    servoj_aligning_right_.store(false);
    servoj_align_seen_busy_left_.store(false);
    servoj_align_seen_busy_right_.store(false);
    servoj_stream_left_.store(false);
    servoj_stream_right_.store(false);
    t_servoj_left_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    t_servoj_right_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

void DualArmNode::OnTeleopTargetJoints(
    bool left, const sensor_msgs::msg::JointState::SharedPtr & msg)
  {
    if (!initialized_ || !servoj_teleop_enabled_.load() || traj_replaying_.load()) {
      return;
    }
    if (!msg || msg->position.size() < 7) {return;}
    nimo_arm::Arm & arm = left ? dual_.Left() : dual_.Right();
    if (ArmUnsafe(arm)) {
      AbortServoJTeleop(left ? "left Fault/EStop/Alarm" : "right Fault/EStop/Alarm");
      return;
    }
    const nimo_arm::V7d q = ToV7Seq(msg->position);
    if (!EnsureServoJAligned(left, q)) {return;}
    if (!arm.ServoJ(q, servoj_interp_time_s_)) {
      AbortServoJTeleop(left ? "left ServoJ rejected" : "right ServoJ rejected");
      return;
    }
    const auto stamp = now();
    if (left) {
      t_servoj_left_ = stamp;
      servoj_stream_left_.store(true);
    } else {
      t_servoj_right_ = stamp;
      servoj_stream_right_.store(true);
    }
  }

void DualArmNode::OnTeleopTargetPose(
    bool left, const geometry_msgs::msg::PoseStamped::SharedPtr & msg)
  {
    if (!initialized_ || !teleop_motion_enabled_.load() || traj_replaying_.load()) {
      return;
    }
    const auto stamp = now();
    if (left) {
      last_teleop_left_ = msg->pose;
      has_teleop_left_ = true;
      t_teleop_left_ = stamp;
    } else {
      last_teleop_right_ = msg->pose;
      has_teleop_right_ = true;
      t_teleop_right_ = stamp;
    }
    nimo_arm::Arm & arm = left ? dual_.Left() : dual_.Right();
    arm.ServoPByPico(ToPose(msg->pose), true);
  }

void DualArmNode::TeleopPoseWatchdog()
  {
    if (!initialized_ || !teleop_motion_enabled_.load() || traj_replaying_.load()) {
      return;
    }
    const auto stamp = now();
    const double to = teleop_pose_timeout_s_;
    auto stale = [&](const rclcpp::Time & t) {
        return (stamp - t).seconds() > to;
      };
    if (has_teleop_left_ && stale(t_teleop_left_)) {
      dual_.Left().ServoPByPico(ToPose(last_teleop_left_), false);
      has_teleop_left_ = false;
    }
    if (has_teleop_right_ && stale(t_teleop_right_)) {
      dual_.Right().ServoPByPico(ToPose(last_teleop_right_), false);
      has_teleop_right_ = false;
    }
  }

void DualArmNode::EndPicoSessions()
  {
    if (!initialized_) {return;}
    if (has_teleop_left_) {
      dual_.Left().ServoPByPico(ToPose(last_teleop_left_), false);
      has_teleop_left_ = false;
    } else {
      dual_.Left().ServoPByPico(nimo_arm::Pose{}, false);
    }
    if (has_teleop_right_) {
      dual_.Right().ServoPByPico(ToPose(last_teleop_right_), false);
      has_teleop_right_ = false;
    } else {
      dual_.Right().ServoPByPico(nimo_arm::Pose{}, false);
    }
  }

bool DualArmNode::AnyTeleopMotion() const
  {
    return teleop_motion_enabled_.load() || servoj_teleop_enabled_.load();
  }

void DualArmNode::HandleSetTeleopMotion(
    const std::shared_ptr<nimo_arm_msgs::srv::SetTeleopMotion::Request> & req,
    const std::shared_ptr<nimo_arm_msgs::srv::SetTeleopMotion::Response> & res)
  {
    const bool is_servoj =
      (req->mode == nimo_arm_msgs::srv::SetTeleopMotion::Request::MODE_SERVOJ);
    if (req->enable && traj_replaying_.load()) {
      res->success = false;
      res->message = "traj replay active; stop replay first";
      return;
    }

    if (is_servoj) {
      if (req->enable) {
        if (!initialized_) {
          res->success = false;
          res->message = "not initialized";
          return;
        }
        if (ArmUnsafe(dual_.Left()) || ArmUnsafe(dual_.Right())) {
          res->success = false;
          res->message = "arm Fault/EStop/Alarm; clear before ServoJ teleop";
          return;
        }
        if (teleop_motion_enabled_.exchange(false)) {
          EndPicoSessions();
        }
        ResetServoJAlign();
        servoj_need_align_left_.store(true);
        servoj_need_align_right_.store(true);
        servoj_teleop_enabled_.store(true);
        SetLastEvent("servoj_on");
        res->success = true;
        res->message = "servoj teleop on (align if |dq|>tol; auto-abort on fault)";
      } else {
        servoj_teleop_enabled_.store(false);
        ResetServoJAlign();
        StopServoJStreams();
        SetLastEvent("servoj_off");
        res->success = true;
        res->message = "servoj teleop off (Stop→Ready)";
      }
      return;
    }

    // Pico (default / MODE_PICO)
    if (req->enable) {
      const bool was_servoj = servoj_teleop_enabled_.exchange(false);
      ResetServoJAlign();
      if (was_servoj) {StopServoJStreams();}
      teleop_motion_enabled_.store(true);
      SetLastEvent("pico_on");
      res->success = true;
      res->message = "pico teleop on";
    } else {
      teleop_motion_enabled_.store(false);
      EndPicoSessions();
      SetLastEvent("pico_off");
      res->success = true;
      res->message = "pico teleop off";
    }
  }

}  // namespace nimo_arm_ros
