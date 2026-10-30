#include "nimo_arm_ros/dual_arm_node.hpp"

#include <rcutils/logging.h>
#include <yaml-cpp/yaml.h>

#include <cstdlib>

using namespace std::chrono_literals;

namespace nimo_arm_ros
{

DualArmNode::DualArmNode()
  : Node("nimo_arm_ros")
  {
    declare_parameter<std::string>("config_path", "");
    declare_parameter<bool>("use_sim", true);
    declare_parameter<bool>("auto_init", true);
    declare_parameter<double>("publish_rate_hz", 50.0);
    declare_parameter<std::string>("waypoint_file", "");
    declare_parameter<std::string>("ns_prefix", "nimo_arm");
    // 500 Hz Run thread: SCHED_FIFO + CPU affinity (needs cap_sys_nice / ensure_rt).
    declare_parameter<bool>("rt_enable", true);
    declare_parameter<bool>("rt_fifo", true);
    declare_parameter<int>("rt_priority", 80);
    // -2 = last online CPU, -1 = no affinity, >=0 = pin that CPU.
    declare_parameter<int>("rt_cpu", -2);
    declare_parameter<bool>("rt_lock_memory", true);
    declare_parameter<bool>("rt_abort_on_fail", false);
    // Debug/diagnostics: structured Run stats + op edges (see run_diagnostics.hpp).
    declare_parameter<bool>("debug", false);
    declare_parameter<std::string>("debug_dir", "/tmp/nimo_arm_diag");
    // 0 = summary + op edges only; e.g. 50 ≈ 10 Hz CSV sample @ 500 Hz control.
    declare_parameter<int>("debug_csv_stride", 0);
    // Motion form: subscribe nimo_teleop world-frame poses (Occam: reuse rebase).
    declare_parameter<bool>("subscribe_teleop_poses", true);
    declare_parameter<std::string>("teleop_left_pose_topic", "arm/left/target_pose");
    declare_parameter<std::string>("teleop_right_pose_topic", "arm/right/target_pose");
    declare_parameter<double>("teleop_pose_timeout_s", 0.25);
    // ServoJ teleop: JointState → ServoJ(q, servoj_interp_time_s).
    declare_parameter<bool>("subscribe_teleop_joints", true);
    declare_parameter<std::string>("teleop_left_joint_topic", "arm/left/target_joints");
    declare_parameter<std::string>("teleop_right_joint_topic", "arm/right/target_joints");
    declare_parameter<double>("servoj_interp_time_s", 0.02);
    declare_parameter<double>("servoj_align_tol_rad", 0.25);
    declare_parameter<double>("servoj_joint_timeout_s", 0.25);
    // Pico front trigger → DualArm::SetGripperGoal (0=open, 1=closed).
    declare_parameter<bool>("subscribe_gripper_joy", true);
    declare_parameter<std::string>("gripper_left_joy_topic", "xr/joy/left");
    declare_parameter<std::string>("gripper_right_joy_topic", "xr/joy/right");
    declare_parameter<double>("gripper_press", 0.9);
    declare_parameter<double>("gripper_release", 0.85);

    const std::string prefix = get_parameter("ns_prefix").as_string();
    ns_ = prefix.empty() ? "" : ("/" + prefix);

    {
      RunDiagConfig dcfg;
      dcfg.enable = get_parameter("debug").as_bool();
      dcfg.dir = get_parameter("debug_dir").as_string();
      const int64_t stride = get_parameter("debug_csv_stride").as_int();
      dcfg.csv_stride = static_cast<uint32_t>(stride > 0 ? stride : 0);
      diag_.Configure(dcfg);
      if (dcfg.enable) {
        const rcutils_ret_t log_rc = rcutils_logging_set_logger_level(
          get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        if (log_rc != RCUTILS_RET_OK) {
          RCLCPP_WARN(get_logger(), "failed to set DEBUG log level");
        }
        RCLCPP_INFO(
          get_logger(), "debug ON dir=%s csv_stride=%u session=%s", dcfg.dir.c_str(),
          dcfg.csv_stride, diag_.session_path().c_str());
      }
    }

    pub_js_ = create_publisher<sensor_msgs::msg::JointState>(ns_ + "/joint_states", 10);
    // Also publish on /joint_states for robot_state_publisher default remaps
    pub_js_global_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    pub_status_ = create_publisher<nimo_arm_msgs::msg::DualArmStatus>(ns_ + "/status", 10);
    pub_rokea_drag_enable_ = create_publisher<std_msgs::msg::Bool>(
      "rokea/drag_bridge/enable", rclcpp::QoS(1).reliable().transient_local());

    pub_left_resp_js_ = create_publisher<sensor_msgs::msg::JointState>(ns_ + "/left/resp/joint", 10);
    pub_right_resp_js_ = create_publisher<sensor_msgs::msg::JointState>(ns_ + "/right/resp/joint", 10);
    pub_left_ref_js_ = create_publisher<sensor_msgs::msg::JointState>(ns_ + "/left/ref/joint", 10);
    pub_right_ref_js_ = create_publisher<sensor_msgs::msg::JointState>(ns_ + "/right/ref/joint", 10);
    pub_left_resp_tcp_ = create_publisher<geometry_msgs::msg::PoseStamped>(ns_ + "/left/resp/tcp", 10);
    pub_right_resp_tcp_ = create_publisher<geometry_msgs::msg::PoseStamped>(ns_ + "/right/resp/tcp", 10);
    pub_left_ref_tcp_ = create_publisher<geometry_msgs::msg::PoseStamped>(ns_ + "/left/ref/tcp", 10);
    pub_right_ref_tcp_ = create_publisher<geometry_msgs::msg::PoseStamped>(ns_ + "/right/ref/tcp", 10);

    srv_init_ = create_service<std_srvs::srv::Trigger>(
      ns_ + "/init",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) { HandleInit(res); });
    srv_shutdown_ = create_service<std_srvs::srv::Trigger>(
      ns_ + "/shutdown",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) { HandleShutdown(res); });

    BindArmTriggers("left", true);
    BindArmTriggers("right", false);

    srv_go_home_ = create_service<std_srvs::srv::Trigger>(
      ns_ + "/go_home",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          HandleGoPreset(true, res);
        });
    srv_go_work_ = create_service<std_srvs::srv::Trigger>(
      ns_ + "/go_work",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          HandleGoPreset(false, res);
        });

    servoj_interp_time_s_ = get_parameter("servoj_interp_time_s").as_double();
    if (servoj_interp_time_s_ <= 0.0) {
      servoj_interp_time_s_ = 0.02;
    }
    servoj_align_tol_rad_ = get_parameter("servoj_align_tol_rad").as_double();
    if (servoj_align_tol_rad_ < 0.0) {
      servoj_align_tol_rad_ = 0.25;
    }
    servoj_joint_timeout_s_ = get_parameter("servoj_joint_timeout_s").as_double();
    if (servoj_joint_timeout_s_ <= 0.0) {
      servoj_joint_timeout_s_ = 0.25;
    }

    BindStreaming("left", true);
    BindStreaming("right", false);
    BindTeleopTargetPoses();
    BindTeleopTargetJoints();
    BindGripperJoy();

    srv_teleop_motion_ = create_service<nimo_arm_msgs::srv::SetTeleopMotion>(
      ns_ + "/set_teleop_motion",
      [this](const std::shared_ptr<nimo_arm_msgs::srv::SetTeleopMotion::Request> req,
        std::shared_ptr<nimo_arm_msgs::srv::SetTeleopMotion::Response> res) {
          HandleSetTeleopMotion(req, res);
        });

    action_mov_j_ = rclcpp_action::create_server<MovJ>(
      this, ns_ + "/mov_j",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const MovJ::Goal> goal) {
        (void)goal;
        return initialized_ ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                            : rclcpp_action::GoalResponse::REJECT;
      },
      [this](const std::shared_ptr<GoalHandleMovJ>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleMovJ> gh) {
        std::thread([this, gh]() { ExecuteMovJ(gh); }).detach();
      });

    action_mov_l_ = rclcpp_action::create_server<MovL>(
      this, ns_ + "/mov_l",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const MovL::Goal> goal) {
        (void)goal;
        return initialized_ ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                            : rclcpp_action::GoalResponse::REJECT;
      },
      [this](const std::shared_ptr<GoalHandleMovL>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleMovL> gh) {
        std::thread([this, gh]() { ExecuteMovL(gh); }).detach();
      });

    srv_wp_save_ = create_service<nimo_arm_msgs::srv::WaypointSave>(
      ns_ + "/waypoint/save",
      [this](const std::shared_ptr<nimo_arm_msgs::srv::WaypointSave::Request> req,
        std::shared_ptr<nimo_arm_msgs::srv::WaypointSave::Response> res) {
          HandleWaypointSave(req, res);
        });
    srv_wp_delete_ = create_service<nimo_arm_msgs::srv::WaypointDelete>(
      ns_ + "/waypoint/delete",
      [this](const std::shared_ptr<nimo_arm_msgs::srv::WaypointDelete::Request> req,
        std::shared_ptr<nimo_arm_msgs::srv::WaypointDelete::Response> res) {
          HandleWaypointDelete(req, res);
        });
    srv_wp_list_ = create_service<nimo_arm_msgs::srv::WaypointList>(
      ns_ + "/waypoint/list",
      [this](const std::shared_ptr<nimo_arm_msgs::srv::WaypointList::Request>,
        std::shared_ptr<nimo_arm_msgs::srv::WaypointList::Response> res) {
          HandleWaypointList(res);
        });
    srv_wp_goto_ = create_service<nimo_arm_msgs::srv::WaypointGoto>(
      ns_ + "/waypoint/goto",
      [this](const std::shared_ptr<nimo_arm_msgs::srv::WaypointGoto::Request> req,
        std::shared_ptr<nimo_arm_msgs::srv::WaypointGoto::Response> res) {
          HandleWaypointGoto(req, res);
        });

    srv_rec_start_ = create_service<nimo_arm_msgs::srv::RecordStart>(
      ns_ + "/record/start",
      [this](const std::shared_ptr<nimo_arm_msgs::srv::RecordStart::Request> req,
        std::shared_ptr<nimo_arm_msgs::srv::RecordStart::Response> res) {
          HandleRecordStart(req, res);
        });
    srv_rec_stop_ = create_service<nimo_arm_msgs::srv::RecordStop>(
      ns_ + "/record/stop",
      [this](const std::shared_ptr<nimo_arm_msgs::srv::RecordStop::Request>,
        std::shared_ptr<nimo_arm_msgs::srv::RecordStop::Response> res) {
          HandleRecordStop(res);
        });

    srv_traj_rec_start_ = create_service<nimo_arm_msgs::srv::TrajRecordStart>(
      ns_ + "/traj/record/start",
      [this](const std::shared_ptr<nimo_arm_msgs::srv::TrajRecordStart::Request> req,
        std::shared_ptr<nimo_arm_msgs::srv::TrajRecordStart::Response> res) {
          HandleTrajRecordStart(req, res);
        });
    srv_traj_rec_stop_ = create_service<nimo_arm_msgs::srv::TrajRecordStop>(
      ns_ + "/traj/record/stop",
      [this](const std::shared_ptr<nimo_arm_msgs::srv::TrajRecordStop::Request>,
        std::shared_ptr<nimo_arm_msgs::srv::TrajRecordStop::Response> res) {
          HandleTrajRecordStop(res);
        });
    srv_traj_post_ = create_service<nimo_arm_msgs::srv::TrajPostprocess>(
      ns_ + "/traj/postprocess",
      [this](const std::shared_ptr<nimo_arm_msgs::srv::TrajPostprocess::Request> req,
        std::shared_ptr<nimo_arm_msgs::srv::TrajPostprocess::Response> res) {
          HandleTrajPostprocess(req, res);
        });
    srv_traj_list_ = create_service<nimo_arm_msgs::srv::TrajList>(
      ns_ + "/traj/list",
      [this](const std::shared_ptr<nimo_arm_msgs::srv::TrajList::Request>,
        std::shared_ptr<nimo_arm_msgs::srv::TrajList::Response> res) {
          HandleTrajList(res);
        });
    srv_traj_replay_start_ = create_service<nimo_arm_msgs::srv::TrajReplayStart>(
      ns_ + "/traj/replay/start",
      [this](const std::shared_ptr<nimo_arm_msgs::srv::TrajReplayStart::Request> req,
        std::shared_ptr<nimo_arm_msgs::srv::TrajReplayStart::Response> res) {
          HandleTrajReplayStart(req, res);
        });
    srv_traj_replay_stop_ = create_service<nimo_arm_msgs::srv::TrajReplayStop>(
      ns_ + "/traj/replay/stop",
      [this](const std::shared_ptr<nimo_arm_msgs::srv::TrajReplayStop::Request>,
        std::shared_ptr<nimo_arm_msgs::srv::TrajReplayStop::Response> res) {
          HandleTrajReplayStop(res);
        });

    ResolvePaths();
    LoadWaypointsFile();

    const double hz = get_parameter("publish_rate_hz").as_double();
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, hz));
    pub_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { PublishState(); });

    if (get_parameter("auto_init").as_bool()) {
      auto res = std::make_shared<std_srvs::srv::Trigger::Response>();
      HandleInit(res);
      if (!res->success) {
        RCLCPP_ERROR(get_logger(), "auto_init failed: %s", res->message.c_str());
      }
    }

    RCLCPP_INFO(get_logger(), "nimo_arm_ros ready (ns=%s, config=%s, sim=%s)",
      ns_.c_str(), config_path_.c_str(),
      get_parameter("use_sim").as_bool() ? "true" : "false");
  }

DualArmNode::~DualArmNode()
  {
    StopRunThread();
    if (initialized_) {
      dual_.Shutdown();
      initialized_ = false;
    }
    StopRecordingUnlocked();
    StopTrajRecordingUnlocked();
    StopTrajReplayUnlocked();
  }

void DualArmNode::ResolvePaths()
  {
    config_path_ = get_parameter("config_path").as_string();
    if (config_path_.empty()) {
#ifdef NIMO_ARM_DEFAULT_CONFIG
      config_path_ = NIMO_ARM_DEFAULT_CONFIG;
#else
      config_path_ = "/home/mundane/nimo_arm/config/arm.yaml";
#endif
    }

    waypoint_file_ = get_parameter("waypoint_file").as_string();
    if (waypoint_file_.empty()) {
      const char * home = std::getenv("HOME");
      fs::path dir = home ? fs::path(home) / ".nimo_robot" : fs::path("/tmp/nimo_robot");
      fs::create_directories(dir);
      waypoint_file_ = (dir / "waypoints.yaml").string();
    }
  }

std::string DualArmNode::PrepareConfigForInit()
  {
    try {
      YAML::Node root = YAML::LoadFile(config_path_);
      if (!root["connect"]) {
        root["connect"] = YAML::Node(YAML::NodeType::Map);
      }
      if (!root["control"]) {
        root["control"] = YAML::Node(YAML::NodeType::Map);
      }
      if (get_parameter("use_sim").as_bool()) {
        root["connect"]["sim"] = true;
      }
      // Cross-PC HMI: arm.yaml default 50 ms jog watchdog is too tight for DDS.
      const int timeout_ms = root["control"]["jog_command_timeout_ms"]
        ? root["control"]["jog_command_timeout_ms"].as<int>()
        : 50;
      if (timeout_ms < 200) {
        root["control"]["jog_command_timeout_ms"] = 200;
      }
      // Overlay is written under /tmp; nimo_arm resolves tool.path relative to the
      // *overlay* directory, so rewrite relative paths against the original config dir.
      if (root["tool"] && root["tool"]["path"]) {
        const fs::path tool_rel = root["tool"]["path"].as<std::string>();
        if (!tool_rel.empty() && !tool_rel.is_absolute()) {
          root["tool"]["path"] =
            (fs::path(config_path_).parent_path() / tool_rel).lexically_normal().string();
        }
      }
      const fs::path out = fs::temp_directory_path() / "nimo_arm_hmi_runtime.yaml";
      std::ofstream ofs(out);
      ofs << root;
      return out.string();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to write config overlay: %s", e.what());
      return config_path_;
    }
  }

std::string DualArmNode::RejectMsg(const nimo_arm::Arm & arm)
  {
    return std::string(nimo_arm::RejectReasonMessage(arm.GetLastRejectReason()));
  }

bool DualArmNode::RequireInit(const std::shared_ptr<std_srvs::srv::Trigger::Response> & res)
  {
    if (!initialized_) {
      res->success = false;
      res->message = "not initialized";
      return false;
    }
    return true;
  }

void DualArmNode::HandleInit(const std::shared_ptr<std_srvs::srv::Trigger::Response> & res)
  {
    std::lock_guard<std::mutex> lock(api_mutex_);
    if (initialized_) {
      res->success = true;
      res->message = "already initialized";
      return;
    }
    const std::string path = PrepareConfigForInit();
    if (!dual_.Init(path.c_str())) {
      res->success = false;
      res->message = "DualArm::Init failed for " + path;
      return;
    }
    if (nimo_arm::LoadConfig(path.c_str(), cfg_)) {
      home_left_ = cfg_.left.home_q;
      home_right_ = cfg_.right.home_q;
      work_left_ = cfg_.left.work_q;
      work_right_ = cfg_.right.work_q;
      has_presets_ = true;
    }
    initialized_ = true;
    StartRunThread();
    RCLCPP_INFO(
      get_logger(), "gripper connected L=%d R=%d",
      dual_.GripperConnected(0) ? 1 : 0, dual_.GripperConnected(1) ? 1 : 0);
    res->success = true;
    res->message = "initialized ts=" + std::to_string(dual_.Ts());
  }

void DualArmNode::HandleShutdown(const std::shared_ptr<std_srvs::srv::Trigger::Response> & res)
  {
    std::lock_guard<std::mutex> lock(api_mutex_);
    StopRunThread();
    {
      std::lock_guard<std::mutex> rlock(record_mutex_);
      StopRecordingUnlocked();
    }
    {
      std::lock_guard<std::mutex> tlock(traj_mutex_);
      StopTrajRecordingUnlocked();
      StopTrajReplayUnlocked();
    }
    teleop_motion_enabled_ = false;
    servoj_teleop_enabled_ = false;
    ResetServoJAlign();
    if (initialized_) {
      dual_.Shutdown();
      initialized_ = false;
    }
    res->success = true;
    res->message = "shutdown complete";
  }

void DualArmNode::HandleGoPreset(bool home, const std::shared_ptr<std_srvs::srv::Trigger::Response> & res)
  {
    if (!RequireInit(res)) {return;}
    if (!has_presets_) {
      res->success = false;
      res->message = "home/work presets not loaded";
      return;
    }
    const nimo_arm::V7d & lq = home ? home_left_ : work_left_;
    const nimo_arm::V7d & rq = home ? home_right_ : work_right_;
    const bool ok_l = dual_.Left().MovJ(lq);
    const bool ok_r = dual_.Right().MovJ(rq);
    res->success = ok_l && ok_r;
    res->message = res->success ? "ok" :
      ("left=" + RejectMsg(dual_.Left()) + " right=" + RejectMsg(dual_.Right()));
  }

void DualArmNode::StartRunThread()
  {
    if (run_alive_) {return;}
    run_stop_ = false;
    run_alive_ = true;
    run_thread_ = std::thread([this]() {
        if (get_parameter("rt_enable").as_bool()) {
          RtThreadOptions opt;
          opt.use_fifo = get_parameter("rt_fifo").as_bool();
          opt.sched_priority = get_parameter("rt_priority").as_int();
          opt.cpu = get_parameter("rt_cpu").as_int();
          opt.lock_memory = get_parameter("rt_lock_memory").as_bool();
          opt.abort_on_rt_failure = get_parameter("rt_abort_on_fail").as_bool();
          const RtThreadStatus st = ApplyRtThreadOptions(opt);
          RCLCPP_INFO(
            get_logger(),
            "Run RT: fifo=%s prio=%d cpu=%d mlock=%s slack=%s",
            (opt.use_fifo ? (st.fifo_ok ? "ok" : "FAIL") : "off"),
            opt.use_fifo ? opt.sched_priority : 0, st.bound_cpu,
            (opt.lock_memory ? (st.memory_locked ? "ok" : "FAIL") : "off"),
            st.timer_slack_ok ? "ok" : "no");
          if (opt.use_fifo && !st.fifo_ok) {
            RCLCPP_WARN(
              get_logger(),
              "SCHED_FIFO not applied — run ensure_rt + setcap on dual_arm_node");
          }
        }

        using clock = std::chrono::steady_clock;
        using dur = std::chrono::duration<double>;
        auto next = clock::now();
        auto period_start = next;
        while (!run_stop_) {
          const auto t_work0 = clock::now();
          if (initialized_) {
            dual_.Run();
            MaybeRecordSample();
            MaybeTrajRecordSample();
          }
          const auto t_work1 = clock::now();
          const double work_s = std::chrono::duration<double>(t_work1 - t_work0).count();
          const double ts = initialized_ ? dual_.Ts() : 0.002;
          next += std::chrono::duration_cast<clock::duration>(dur(ts));
          const auto now = clock::now();
          bool overrun = false;
          if (now < next) {
            std::this_thread::sleep_until(next);
          } else {
            overrun = true;
            while (next <= now) {
              next += std::chrono::duration_cast<clock::duration>(dur(ts));
            }
          }
          const auto period_end = clock::now();
          const double dt_s = std::chrono::duration<double>(period_end - period_start).count();
          period_start = period_end;
          diag_.OnCycle(dt_s, work_s, overrun);
        }
        run_alive_ = false;
      });
  }

void DualArmNode::StopRunThread()
  {
    run_stop_ = true;
    if (run_thread_.joinable()) {
      run_thread_.join();
    }
    run_alive_ = false;
    diag_.Close();
  }

void DualArmNode::PublishState()
  {
    if (!initialized_) {
      if (servoj_teleop_enabled_.load()) {
        AbortServoJTeleop("not initialized");
      }
      nimo_arm_msgs::msg::DualArmStatus st;
      st.header.stamp = now();
      st.initialized = false;
      st.run_thread_alive = run_alive_;
      st.ts = 0.0;
      st.teleop_motion_enabled = teleop_motion_enabled_.load();
      st.servoj_teleop_enabled = servoj_teleop_enabled_.load();
      st.traj_recording = false;
      st.traj_replaying = false;
      {
        std::lock_guard<std::mutex> lock(last_event_mu_);
        st.last_event = last_event_;
        st.last_event_seq = last_event_seq_;
      }
      pub_status_->publish(st);
      {
        std_msgs::msg::Bool en;
        en.data = st.servoj_teleop_enabled;
        pub_rokea_drag_enable_->publish(en);
      }
      return;
    }

    MaybeAbortServoJTeleop();

    nimo_arm::RobotState left_resp{}, right_resp{}, left_ref{}, right_ref{};
    dual_.Left().GetResp(left_resp);
    dual_.Right().GetResp(right_resp);
    dual_.Left().GetRef(left_ref);
    dual_.Right().GetRef(right_ref);

    const auto stamp = now();
    // RViz follows commanded ref (avoids sim meas glitches during JogJ); resp stays on */resp/*.
    sensor_msgs::msg::JointState js;
    FillJointState(js, left_ref, right_ref, stamp);
    pub_js_->publish(js);
    pub_js_global_->publish(js);

    sensor_msgs::msg::JointState tmp;
    FillArmJointState(tmp, "_L", left_resp.joint, stamp);
    pub_left_resp_js_->publish(tmp);
    FillArmJointState(tmp, "_R", right_resp.joint, stamp);
    pub_right_resp_js_->publish(tmp);
    FillArmJointState(tmp, "_L", left_ref.joint, stamp);
    pub_left_ref_js_->publish(tmp);
    FillArmJointState(tmp, "_R", right_ref.joint, stamp);
    pub_right_ref_js_->publish(tmp);

    auto pub_tcp = [&](auto & pub, const nimo_arm::Pose & pose, const std::string & frame) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = stamp;
        ps.header.frame_id = frame;
        ps.pose = ToMsgPose(pose);
        pub->publish(ps);
      };
    pub_tcp(pub_left_resp_tcp_, left_resp.tcp, "Base_L");
    pub_tcp(pub_right_resp_tcp_, right_resp.tcp, "Base_R");
    pub_tcp(pub_left_ref_tcp_, left_ref.tcp, "Base_L");
    pub_tcp(pub_right_ref_tcp_, right_ref.tcp, "Base_R");

    nimo_arm_msgs::msg::DualArmStatus st;
    st.header.stamp = stamp;
    st.initialized = true;
    st.run_thread_alive = run_alive_;
    st.ts = dual_.Ts();
    st.teleop_motion_enabled = teleop_motion_enabled_.load();
    st.servoj_teleop_enabled = servoj_teleop_enabled_.load();
    st.traj_recording = traj_recording_.load();
    st.traj_replaying = traj_replaying_.load();
    {
      std::lock_guard<std::mutex> lock(last_event_mu_);
      st.last_event = last_event_;
      st.last_event_seq = last_event_seq_;
    }
    FillArmStatus(st.left, dual_.Left(), stamp);
    FillArmStatus(st.right, dual_.Right(), stamp);
    if (diag_.enabled()) {
      diag_.OnOpEdge(
        "left", static_cast<int>(dual_.Left().GetOpState()), st.left.error_message.c_str(),
        st.left.reject_message.c_str());
      diag_.OnOpEdge(
        "right", static_cast<int>(dual_.Right().GetOpState()), st.right.error_message.c_str(),
        st.right.reject_message.c_str());
    }
    pub_status_->publish(st);
    {
      std_msgs::msg::Bool en;
      en.data = st.servoj_teleop_enabled;
      pub_rokea_drag_enable_->publish(en);
    }
  }

}  // namespace nimo_arm_ros
