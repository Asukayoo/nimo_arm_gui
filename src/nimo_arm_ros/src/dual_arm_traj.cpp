#include "nimo_arm_ros/dual_arm_node.hpp"

#include <algorithm>
#include <cstdlib>
#include <iomanip>

using namespace std::chrono_literals;

namespace nimo_arm_ros
{

fs::path DualArmNode::DefaultTrajDir()
  {
    const char * home = std::getenv("HOME");
    return home ? fs::path(home) / ".nimo_robot" / "trajectories"
                : fs::path("/tmp/nimo_robot/trajectories");
  }

void DualArmNode::HandleTrajRecordStart(
    const std::shared_ptr<nimo_arm_msgs::srv::TrajRecordStart::Request> & req,
    const std::shared_ptr<nimo_arm_msgs::srv::TrajRecordStart::Response> & res)
  {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    if (traj_recording_) {
      res->success = false;
      res->message = "already traj-recording";
      res->path = traj_path_;
      return;
    }
    if (!initialized_) {
      res->success = false;
      res->message = "not initialized";
      return;
    }
    std::string path = req->path;
    const fs::path dir = DefaultTrajDir();
    fs::create_directories(dir);
    if (path.empty()) {
      const auto t = std::chrono::system_clock::now().time_since_epoch().count();
      path = (dir / ("traj_" + std::to_string(t) + ".traj")).string();
    } else {
      fs::path p(path);
      // Relative / bare name → under traj dir; strip path tricks to basename.
      if (!p.is_absolute()) {
        std::string name = p.filename().string();
        if (name.empty() || name == "." || name == "..") {
          res->success = false;
          res->message = "invalid traj name";
          return;
        }
        if (name.size() < 5 || name.substr(name.size() - 5) != ".traj") {
          name += ".traj";
        }
        p = dir / name;
      } else {
        fs::create_directories(p.parent_path());
      }
      path = p.string();
    }
    traj_file_.open(path, std::ios::out | std::ios::trunc);
    if (!traj_file_) {
      res->success = false;
      res->message = "cannot open " + path;
      return;
    }
    traj_file_ << "# nimo_arm_traj v1\n";
    traj_file_ << "t_sec,arm,q0,q1,q2,q3,q4,q5,q6\n";
    traj_path_ = path;
    traj_rows_ = 0;
    traj_period_ = req->sample_period > 0.0 ? req->sample_period : 0.02;
    traj_t0_ = now().seconds();
    traj_next_ = std::chrono::steady_clock::now();
    traj_recording_ = true;
    res->success = true;
    res->message = "traj recording";
    res->path = path;
  }

void DualArmNode::HandleTrajRecordStop(
    const std::shared_ptr<nimo_arm_msgs::srv::TrajRecordStop::Response> & res)
  {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    if (!traj_recording_) {
      res->success = false;
      res->message = "not traj-recording";
      res->path = traj_path_;
      res->rows = traj_rows_;
      return;
    }
    const uint64_t rows = traj_rows_;
    const std::string path = traj_path_;
    StopTrajRecordingUnlocked();
    res->success = true;
    res->message = "stopped";
    res->path = path;
    res->rows = rows;
  }

void DualArmNode::StopTrajRecordingUnlocked()
  {
    traj_recording_ = false;
    if (traj_file_.is_open()) {
      traj_file_.flush();
      traj_file_.close();
    }
  }

void DualArmNode::MaybeTrajRecordSample()
  {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    if (!traj_recording_ || !traj_file_) {return;}
    const auto now_tp = std::chrono::steady_clock::now();
    if (now_tp < traj_next_) {return;}
    traj_next_ = now_tp + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(traj_period_));

    const double t = now().seconds() - traj_t0_;
    auto write_arm = [&](const char * name, bool left, nimo_arm::Arm & arm) {
        nimo_arm::RobotState resp{};
        arm.GetResp(resp);
        traj_file_ << std::setprecision(9) << t << ',' << name;
        for (int i = 0; i < 7; ++i) {traj_file_ << ',' << resp.joint.q(i);}
        traj_file_ << '\n';
        ++traj_rows_;
        (void)left;
      };
    write_arm("left", true, dual_.Left());
    write_arm("right", false, dual_.Right());
  }

void DualArmNode::HandleTrajList(
    const std::shared_ptr<nimo_arm_msgs::srv::TrajList::Response> & res)
  {
    res->paths.clear();
    const fs::path dir = DefaultTrajDir();
    std::error_code ec;
    if (!fs::exists(dir, ec)) {
      fs::create_directories(dir, ec);
      res->success = true;
      res->message = "empty";
      return;
    }
    std::vector<fs::path> files;
    for (const auto & ent : fs::directory_iterator(dir, ec)) {
      if (ec) {break;}
      if (!ent.is_regular_file()) {continue;}
      if (ent.path().extension() == ".traj") {files.push_back(ent.path());}
    }
    std::sort(files.begin(), files.end());
    for (const auto & p : files) {res->paths.push_back(p.string());}
    res->success = true;
    res->message = std::to_string(res->paths.size()) + " traj(s)";
  }

void DualArmNode::HandleTrajPostprocess(
    const std::shared_ptr<nimo_arm_msgs::srv::TrajPostprocess::Request> & req,
    const std::shared_ptr<nimo_arm_msgs::srv::TrajPostprocess::Response> & res)
  {
    if (req->input_path.empty()) {
      res->success = false;
      res->message = "input_path required";
      return;
    }
    TrajFile in;
    const std::string load_err = LoadTraj(req->input_path, in);
    if (!load_err.empty()) {
      res->success = false;
      res->message = load_err;
      return;
    }
    std::string out_path = req->output_path;
    if (out_path.empty()) {
      const fs::path p(req->input_path);
      out_path = (p.parent_path() / (p.stem().string() + "_smooth.traj")).string();
    } else {
      fs::create_directories(fs::path(out_path).parent_path());
    }
    TrajFile out;
    const std::string pp_err =
      PostprocessTraj(in, out, req->sg_window, req->resample_dt);
    if (!pp_err.empty()) {
      res->success = false;
      res->message = pp_err;
      return;
    }
    const std::string save_err = SaveTraj(out_path, out);
    if (!save_err.empty()) {
      res->success = false;
      res->message = save_err;
      return;
    }
    res->success = true;
    res->message = "ok";
    res->output_path = out_path;
    res->rows = out.rows.size();
  }

void DualArmNode::HandleTrajReplayStart(
    const std::shared_ptr<nimo_arm_msgs::srv::TrajReplayStart::Request> & req,
    const std::shared_ptr<nimo_arm_msgs::srv::TrajReplayStart::Response> & res)
  {
    if (!initialized_) {
      res->success = false;
      res->message = "not initialized";
      return;
    }
    if (AnyTeleopMotion()) {
      res->success = false;
      res->message = "teleop motion on; disable Pico/ServoJ teleop first";
      return;
    }
    if (req->path.empty()) {
      res->success = false;
      res->message = "path required";
      return;
    }
    TrajFile file;
    const std::string err = LoadTraj(req->path, file);
    if (!err.empty()) {
      res->success = false;
      res->message = err;
      return;
    }
    auto frames = ToFrames(file);
    if (frames.empty()) {
      res->success = false;
      res->message = "no frames";
      return;
    }

    if (req->approach_first) {
      const auto & fr0 = frames.front();
      if (fr0.has_left && !dual_.Left().MovJ(ToV7(fr0.left_q))) {
        res->success = false;
        res->message = "left approach: " + RejectMsg(dual_.Left());
        return;
      }
      if (fr0.has_right && !dual_.Right().MovJ(ToV7(fr0.right_q))) {
        res->success = false;
        res->message = "right approach: " + RejectMsg(dual_.Right());
        return;
      }
      const auto deadline = std::chrono::steady_clock::now() + 30s;
      while (std::chrono::steady_clock::now() < deadline) {
        const bool ld = !fr0.has_left || dual_.Left().IsMotionDone();
        const bool rd = !fr0.has_right || dual_.Right().IsMotionDone();
        if (ld && rd) {break;}
        std::this_thread::sleep_for(10ms);
      }
    }

    std::string auto_rec_path;
    if (req->auto_record) {
      std::lock_guard<std::mutex> rlock(record_mutex_);
      if (!recording_) {
        const fs::path traj_p(req->path);
        const fs::path dir = DefaultRecordDir();
        fs::create_directories(dir);
        const auto t = std::chrono::system_clock::now().time_since_epoch().count();
        const std::string stem = traj_p.stem().string();
        const std::string path =
          (dir / ("replay_" + stem + "_" + std::to_string(t) + ".csv")).string();
        std::string err;
        if (!StartRecordUnlocked(path, 0.02, err)) {
          res->success = false;
          res->message = "auto_record: " + err;
          return;
        }
        replay_owns_record_ = true;
        auto_rec_path = record_path_;
      } else {
        // Manual Record already running — do not own/stop it.
        replay_owns_record_ = false;
        auto_rec_path = record_path_;
      }
    }

    {
      std::lock_guard<std::mutex> lock(traj_mutex_);
      if (traj_replaying_) {
        if (req->auto_record) {
          std::lock_guard<std::mutex> rlock(record_mutex_);
          StopReplayAutoRecordUnlocked();
        }
        res->success = false;
        res->message = "already replaying";
        return;
      }
      replay_frames_ = std::move(frames);
      replay_rate_ = req->rate_scale > 0.0 ? req->rate_scale : 1.0;
      replay_index_ = 0;
      replay_auto_record_ = req->auto_record;
      replay_record_path_ = auto_rec_path;
      traj_replaying_ = true;
      const double dt =
        replay_frames_.size() >= 2
          ? std::max(
            1e-3, (replay_frames_[1].t_sec - replay_frames_[0].t_sec) / replay_rate_)
          : 0.02 / replay_rate_;
      replay_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(dt)),
        [this]() { TrajReplayTick(); });
    }
    res->success = true;
    res->message = "replaying " + req->path;
    res->record_path = auto_rec_path;
  }

void DualArmNode::HandleTrajReplayStop(
    const std::shared_ptr<nimo_arm_msgs::srv::TrajReplayStop::Response> & res)
  {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    if (!traj_replaying_) {
      res->success = false;
      res->message = "not replaying";
      return;
    }
    StopTrajReplayUnlocked();
    res->success = true;
    res->message = "stopped";
  }

void DualArmNode::StopTrajReplayUnlocked()
  {
    traj_replaying_ = false;
    if (replay_timer_) {
      replay_timer_->cancel();
      replay_timer_.reset();
    }
    replay_frames_.clear();
    replay_index_ = 0;
    replay_auto_record_ = false;
    // traj_mutex_ held; lock order traj → record
    std::lock_guard<std::mutex> rlock(record_mutex_);
    StopReplayAutoRecordUnlocked();
  }

void DualArmNode::TrajReplayTick()
  {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    if (!traj_replaying_ || replay_index_ >= replay_frames_.size()) {
      StopTrajReplayUnlocked();
      return;
    }
    const auto & fr = replay_frames_[replay_index_++];
    if (fr.has_left) {dual_.Left().ServoJ(ToV7(fr.left_q));}
    if (fr.has_right) {dual_.Right().ServoJ(ToV7(fr.right_q));}
    if (replay_index_ >= replay_frames_.size()) {
      StopTrajReplayUnlocked();
    }
  }

}  // namespace nimo_arm_ros
