#pragma once

#include "nimo_arm_ros/converters.hpp"
#include "nimo_arm_ros/rt_thread.hpp"
#include "nimo_arm_ros/run_diagnostics.hpp"
#include "nimo_arm_ros/traj_store.hpp"

#include "nimo_arm/arm_control.hpp"
#include "nimo_arm/types.hpp"

#include "nimo_arm_msgs/action/mov_j.hpp"
#include "nimo_arm_msgs/action/mov_l.hpp"
#include "nimo_arm_msgs/msg/dual_arm_status.hpp"
#include "nimo_arm_msgs/msg/jog_cart.hpp"
#include "nimo_arm_msgs/msg/jog_joint.hpp"
#include "nimo_arm_msgs/msg/servo_p_by_pico.hpp"
#include "nimo_arm_msgs/msg/waypoint.hpp"
#include "nimo_arm_msgs/srv/record_start.hpp"
#include "nimo_arm_msgs/srv/record_stop.hpp"
#include "nimo_arm_msgs/srv/set_brake.hpp"
#include "nimo_arm_msgs/srv/set_control_mode.hpp"
#include "nimo_arm_msgs/srv/set_cart_imp_kd.hpp"
#include "nimo_arm_msgs/srv/set_joint_imp_kd.hpp"
#include "nimo_arm_msgs/srv/set_teleop_motion.hpp"
#include "nimo_arm_msgs/srv/traj_list.hpp"
#include "nimo_arm_msgs/srv/traj_postprocess.hpp"
#include "nimo_arm_msgs/srv/traj_record_start.hpp"
#include "nimo_arm_msgs/srv/traj_record_stop.hpp"
#include "nimo_arm_msgs/srv/traj_replay_start.hpp"
#include "nimo_arm_msgs/srv/traj_replay_stop.hpp"
#include "nimo_arm_msgs/srv/waypoint_delete.hpp"
#include "nimo_arm_msgs/srv/waypoint_goto.hpp"
#include "nimo_arm_msgs/srv/waypoint_list.hpp"
#include "nimo_arm_msgs/srv/waypoint_save.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace fs = std::filesystem;

namespace nimo_arm_ros
{

class DualArmNode : public rclcpp::Node
{
public:
  using MovJ = nimo_arm_msgs::action::MovJ;
  using MovL = nimo_arm_msgs::action::MovL;
  using GoalHandleMovJ = rclcpp_action::ServerGoalHandle<MovJ>;
  using GoalHandleMovL = rclcpp_action::ServerGoalHandle<MovL>;

  DualArmNode();
  ~DualArmNode() override;

private:
  void ResolvePaths();
  std::string PrepareConfigForInit();
  double ScaleJogJoint(int joint, double direction) const;
  double ScaleJogCart(uint8_t axis, double direction) const;
  void BindArmTriggers(const std::string & side, bool left);
  void BindStreaming(const std::string & side, bool left);
  void BindTeleopTargetPoses();
  void BindTeleopTargetJoints();
  void BindGripperJoy();
  void OnGripperJoy(bool left, const sensor_msgs::msg::Joy::SharedPtr & msg);
  void GripperClosedRefresh();
  void OnTeleopTargetPose(
    bool left, const geometry_msgs::msg::PoseStamped::SharedPtr & msg);
  void OnTeleopTargetJoints(
    bool left, const sensor_msgs::msg::JointState::SharedPtr & msg);
  /** ServoJ teleop: optional MovJ align when |q_cmd-q_ref| > tol; returns true if streaming ok. */
  bool EnsureServoJAligned(bool left, const nimo_arm::V7d & q_cmd);
  void TeleopPoseWatchdog();
  void EndPicoSessions();
  /** Soft-stop both arms so OpState leaves Moving after ServoJ stream ends. */
  void StopServoJStreams();
  void ResetServoJAlign();
  void AbortServoJTeleop(const char * reason);
  void SetLastEvent(const std::string & event);
  /** Fault/EStop/Alarm or joint-stream timeout → clear servoj_teleop_enabled. */
  void MaybeAbortServoJTeleop();
  static bool ArmUnsafe(const nimo_arm::Arm & arm);
  bool AnyTeleopMotion() const;
  void HandleSetTeleopMotion(
    const std::shared_ptr<nimo_arm_msgs::srv::SetTeleopMotion::Request> & req,
    const std::shared_ptr<nimo_arm_msgs::srv::SetTeleopMotion::Response> & res);
  static std::string RejectMsg(const nimo_arm::Arm & arm);
  bool RequireInit(const std::shared_ptr<std_srvs::srv::Trigger::Response> & res);
  void HandleInit(const std::shared_ptr<std_srvs::srv::Trigger::Response> & res);
  void HandleShutdown(const std::shared_ptr<std_srvs::srv::Trigger::Response> & res);
  void HandleGoPreset(bool home, const std::shared_ptr<std_srvs::srv::Trigger::Response> & res);
  void StartRunThread();
  void StopRunThread();
  void PublishState();
  void ExecuteMovJ(const std::shared_ptr<GoalHandleMovJ> gh);
  void ExecuteMovL(const std::shared_ptr<GoalHandleMovL> gh);
  void LoadWaypointsFile();
  bool SaveWaypointsFile();
  void HandleWaypointSave(
    const std::shared_ptr<nimo_arm_msgs::srv::WaypointSave::Request> & req,
    const std::shared_ptr<nimo_arm_msgs::srv::WaypointSave::Response> & res);
  void HandleWaypointDelete(
    const std::shared_ptr<nimo_arm_msgs::srv::WaypointDelete::Request> & req,
    const std::shared_ptr<nimo_arm_msgs::srv::WaypointDelete::Response> & res);
  void HandleWaypointList(const std::shared_ptr<nimo_arm_msgs::srv::WaypointList::Response> & res);
  void HandleWaypointGoto(
    const std::shared_ptr<nimo_arm_msgs::srv::WaypointGoto::Request> & req,
    const std::shared_ptr<nimo_arm_msgs::srv::WaypointGoto::Response> & res);
  void HandleRecordStart(
    const std::shared_ptr<nimo_arm_msgs::srv::RecordStart::Request> & req,
    const std::shared_ptr<nimo_arm_msgs::srv::RecordStart::Response> & res);
  void HandleRecordStop(const std::shared_ptr<nimo_arm_msgs::srv::RecordStop::Response> & res);
  void StopRecordingUnlocked();
  void MaybeRecordSample();
  /** Start status Record if idle; returns false + message if already recording or fail. */
  bool StartRecordUnlocked(const std::string & path, double sample_period, std::string & err);
  /** Stop auto-started replay Record (no-op if not owned by replay). */
  void StopReplayAutoRecordUnlocked();
  static fs::path DefaultRecordDir();
  static fs::path DefaultTrajDir();
  void HandleTrajRecordStart(
    const std::shared_ptr<nimo_arm_msgs::srv::TrajRecordStart::Request> & req,
    const std::shared_ptr<nimo_arm_msgs::srv::TrajRecordStart::Response> & res);
  void HandleTrajRecordStop(
    const std::shared_ptr<nimo_arm_msgs::srv::TrajRecordStop::Response> & res);
  void StopTrajRecordingUnlocked();
  void MaybeTrajRecordSample();
  void HandleTrajList(
    const std::shared_ptr<nimo_arm_msgs::srv::TrajList::Response> & res);
  void HandleTrajPostprocess(
    const std::shared_ptr<nimo_arm_msgs::srv::TrajPostprocess::Request> & req,
    const std::shared_ptr<nimo_arm_msgs::srv::TrajPostprocess::Response> & res);
  void HandleTrajReplayStart(
    const std::shared_ptr<nimo_arm_msgs::srv::TrajReplayStart::Request> & req,
    const std::shared_ptr<nimo_arm_msgs::srv::TrajReplayStart::Response> & res);
  void HandleTrajReplayStop(
    const std::shared_ptr<nimo_arm_msgs::srv::TrajReplayStop::Response> & res);
  void StopTrajReplayUnlocked();
  void TrajReplayTick();

  nimo_arm::DualArm dual_;
  std::atomic<bool> initialized_{false};
  std::atomic<bool> teleop_motion_enabled_{false};   // Pico 遥操作
  std::atomic<bool> servoj_teleop_enabled_{false};   // ServoJ 遥操作
  std::mutex last_event_mu_;
  std::string last_event_;
  uint32_t last_event_seq_{0};
  std::atomic<bool> traj_recording_{false};
  std::atomic<bool> traj_replaying_{false};
  std::atomic<bool> run_stop_{false};
  std::atomic<bool> run_alive_{false};
  std::thread run_thread_;
  RunDiagnostics diag_;
  std::mutex api_mutex_;

  std::string ns_;
  std::string config_path_;
  std::string waypoint_file_;

  bool has_presets_{false};
  nimo_arm::Config cfg_{};
  nimo_arm::V7d home_left_{};
  nimo_arm::V7d home_right_{};
  nimo_arm::V7d work_left_{};
  nimo_arm::V7d work_right_{};

  std::mutex wp_mutex_;
  std::unordered_map<std::string, nimo_arm_msgs::msg::Waypoint> waypoints_;

  std::mutex record_mutex_;
  bool recording_{false};
  bool replay_owns_record_{false};
  std::ofstream record_file_;
  std::string record_path_;
  uint64_t record_rows_{0};
  double record_period_{0.02};
  std::chrono::steady_clock::time_point record_next_{};

  std::mutex traj_mutex_;
  std::ofstream traj_file_;
  std::string traj_path_;
  uint64_t traj_rows_{0};
  double traj_period_{0.02};
  double traj_t0_{0.0};
  std::chrono::steady_clock::time_point traj_next_{};
  std::vector<TrajFrame> replay_frames_;
  size_t replay_index_{0};
  double replay_rate_{1.0};
  bool replay_auto_record_{false};
  std::string replay_record_path_;
  rclcpp::TimerBase::SharedPtr replay_timer_;

  double teleop_pose_timeout_s_{0.25};
  double servoj_interp_time_s_{0.02};
  double servoj_align_tol_rad_{0.25};
  double servoj_joint_timeout_s_{0.25};
  std::atomic<bool> servoj_need_align_left_{false};
  std::atomic<bool> servoj_need_align_right_{false};
  std::atomic<bool> servoj_aligning_left_{false};
  std::atomic<bool> servoj_aligning_right_{false};
  /** MovJ pending is async; ignore IsMotionDone until RT has cleared it once. */
  std::atomic<bool> servoj_align_seen_busy_left_{false};
  std::atomic<bool> servoj_align_seen_busy_right_{false};
  std::atomic<bool> servoj_stream_left_{false};
  std::atomic<bool> servoj_stream_right_{false};
  rclcpp::Time t_servoj_left_{0, 0, RCL_ROS_TIME};
  rclcpp::Time t_servoj_right_{0, 0, RCL_ROS_TIME};
  bool has_teleop_left_{false};
  bool has_teleop_right_{false};
  geometry_msgs::msg::Pose last_teleop_left_{};
  geometry_msgs::msg::Pose last_teleop_right_{};
  rclcpp::Time t_teleop_left_{0, 0, RCL_ROS_TIME};
  rclcpp::Time t_teleop_right_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr teleop_watchdog_;

  // Pico front trigger (axes[2]) → open/close latch; matches test_rt_teleop TickGripper.
  struct GripperLatch {
    bool trigger_held{false};
    bool closed{false};
    bool seeded{false};
  };
  bool subscribe_gripper_joy_{true};
  double gripper_press_{0.9};
  double gripper_release_{0.85};
  GripperLatch grip_l_{};
  GripperLatch grip_r_{};
  rclcpp::TimerBase::SharedPtr gripper_refresh_timer_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js_global_;
  rclcpp::Publisher<nimo_arm_msgs::msg::DualArmStatus>::SharedPtr pub_status_;
  /** Mirrors servoj_teleop_enabled → rokea_drag_bridge (true=drag, false=power off). */
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_rokea_drag_enable_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_left_resp_js_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_right_resp_js_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_left_ref_js_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_right_ref_js_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_left_resp_tcp_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_right_resp_tcp_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_left_ref_tcp_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_right_ref_tcp_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_init_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_shutdown_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_go_home_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_go_work_;
  std::vector<rclcpp::ServiceBase::SharedPtr> arm_services_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> stream_subs_;

  rclcpp_action::Server<MovJ>::SharedPtr action_mov_j_;
  rclcpp_action::Server<MovL>::SharedPtr action_mov_l_;

  rclcpp::Service<nimo_arm_msgs::srv::WaypointSave>::SharedPtr srv_wp_save_;
  rclcpp::Service<nimo_arm_msgs::srv::WaypointDelete>::SharedPtr srv_wp_delete_;
  rclcpp::Service<nimo_arm_msgs::srv::WaypointList>::SharedPtr srv_wp_list_;
  rclcpp::Service<nimo_arm_msgs::srv::WaypointGoto>::SharedPtr srv_wp_goto_;
  rclcpp::Service<nimo_arm_msgs::srv::RecordStart>::SharedPtr srv_rec_start_;
  rclcpp::Service<nimo_arm_msgs::srv::RecordStop>::SharedPtr srv_rec_stop_;
  rclcpp::Service<nimo_arm_msgs::srv::SetTeleopMotion>::SharedPtr srv_teleop_motion_;
  rclcpp::Service<nimo_arm_msgs::srv::TrajRecordStart>::SharedPtr srv_traj_rec_start_;
  rclcpp::Service<nimo_arm_msgs::srv::TrajRecordStop>::SharedPtr srv_traj_rec_stop_;
  rclcpp::Service<nimo_arm_msgs::srv::TrajPostprocess>::SharedPtr srv_traj_post_;
  rclcpp::Service<nimo_arm_msgs::srv::TrajList>::SharedPtr srv_traj_list_;
  rclcpp::Service<nimo_arm_msgs::srv::TrajReplayStart>::SharedPtr srv_traj_replay_start_;
  rclcpp::Service<nimo_arm_msgs::srv::TrajReplayStop>::SharedPtr srv_traj_replay_stop_;

  rclcpp::TimerBase::SharedPtr pub_timer_;
};

}  // namespace nimo_arm_ros
