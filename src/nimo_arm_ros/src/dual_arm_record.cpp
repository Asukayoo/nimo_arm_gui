#include "nimo_arm_ros/dual_arm_node.hpp"

#include <cstdlib>
#include <iomanip>

using namespace std::chrono_literals;

namespace nimo_arm_ros
{

fs::path DualArmNode::DefaultRecordDir()
{
  const char * home = std::getenv("HOME");
  return home ? fs::path(home) / ".nimo_robot" / "records"
              : fs::path("/tmp/nimo_robot/records");
}

bool DualArmNode::StartRecordUnlocked(
  const std::string & path_in, double sample_period, std::string & err)
{
  if (recording_) {
    err = "already recording";
    return false;
  }
  if (!initialized_) {
    err = "not initialized";
    return false;
  }

  std::string path = path_in;
  if (path.empty()) {
    const fs::path dir = DefaultRecordDir();
    fs::create_directories(dir);
    const auto t = std::chrono::system_clock::now().time_since_epoch().count();
    path = (dir / ("record_" + std::to_string(t) + ".csv")).string();
  } else {
    fs::create_directories(fs::path(path).parent_path());
  }

  record_file_.open(path, std::ios::out | std::ios::trunc);
  if (!record_file_) {
    err = "cannot open " + path;
    return false;
  }
  record_file_ <<
    "t_sec,arm,"
    "ref_q0,ref_q1,ref_q2,ref_q3,ref_q4,ref_q5,ref_q6,"
    "resp_q0,resp_q1,resp_q2,resp_q3,resp_q4,resp_q5,resp_q6,"
    "ref_dq0,ref_dq1,ref_dq2,ref_dq3,ref_dq4,ref_dq5,ref_dq6,"
    "resp_dq0,resp_dq1,resp_dq2,resp_dq3,resp_dq4,resp_dq5,resp_dq6,"
    "ref_px,ref_py,ref_pz,ref_qx,ref_qy,ref_qz,ref_qw,"
    "resp_px,resp_py,resp_pz,resp_qx,resp_qy,resp_qz,resp_qw,"
    "tau0,tau1,tau2,tau3,tau4,tau5,tau6\n";

  record_path_ = path;
  record_rows_ = 0;
  record_period_ = sample_period > 0.0 ? sample_period : 0.02;
  record_next_ = std::chrono::steady_clock::now();
  recording_ = true;
  err.clear();
  return true;
}

void DualArmNode::HandleRecordStart(
  const std::shared_ptr<nimo_arm_msgs::srv::RecordStart::Request> & req,
  const std::shared_ptr<nimo_arm_msgs::srv::RecordStart::Response> & res)
{
  std::lock_guard<std::mutex> lock(record_mutex_);
  if (recording_) {
    res->success = false;
    res->message = "already recording";
    res->path = record_path_;
    return;
  }
  std::string err;
  if (!StartRecordUnlocked(req->path, req->sample_period, err)) {
    res->success = false;
    res->message = err;
    return;
  }
  replay_owns_record_ = false;
  res->success = true;
  res->message = "recording";
  res->path = record_path_;
}

void DualArmNode::HandleRecordStop(
  const std::shared_ptr<nimo_arm_msgs::srv::RecordStop::Response> & res)
{
  std::lock_guard<std::mutex> lock(record_mutex_);
  if (!recording_) {
    res->success = false;
    res->message = "not recording";
    res->path = record_path_;
    res->rows = record_rows_;
    return;
  }
  const uint64_t rows = record_rows_;
  const std::string path = record_path_;
  StopRecordingUnlocked();
  replay_owns_record_ = false;
  res->success = true;
  res->message = "stopped";
  res->path = path;
  res->rows = rows;
}

void DualArmNode::StopRecordingUnlocked()
{
  recording_ = false;
  if (record_file_.is_open()) {
    record_file_.flush();
    record_file_.close();
  }
}

void DualArmNode::StopReplayAutoRecordUnlocked()
{
  // Caller must hold record_mutex_. Only stops if replay started this Record.
  if (!replay_owns_record_) {
    return;
  }
  if (recording_) {
    StopRecordingUnlocked();
  }
  replay_owns_record_ = false;
}

void DualArmNode::MaybeRecordSample()
{
  std::lock_guard<std::mutex> lock(record_mutex_);
  if (!recording_ || !record_file_) {return;}
  const auto now_tp = std::chrono::steady_clock::now();
  if (now_tp < record_next_) {return;}
  record_next_ = now_tp + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(record_period_));

  const double t = now().seconds();
  auto write_arm = [&](const char * name, nimo_arm::Arm & arm) {
      nimo_arm::RobotState ref{}, resp{};
      arm.GetRef(ref);
      arm.GetResp(resp);
      record_file_ << std::setprecision(9) << t << ',' << name;
      for (int i = 0; i < 7; ++i) {record_file_ << ',' << ref.joint.q(i);}
      for (int i = 0; i < 7; ++i) {record_file_ << ',' << resp.joint.q(i);}
      for (int i = 0; i < 7; ++i) {record_file_ << ',' << ref.joint.dq(i);}
      for (int i = 0; i < 7; ++i) {record_file_ << ',' << resp.joint.dq(i);}
      record_file_ << ',' << ref.tcp.pos.x() << ',' << ref.tcp.pos.y() << ',' << ref.tcp.pos.z();
      record_file_ << ',' << ref.tcp.quat.x() << ',' << ref.tcp.quat.y()
                   << ',' << ref.tcp.quat.z() << ',' << ref.tcp.quat.w();
      record_file_ << ',' << resp.tcp.pos.x() << ',' << resp.tcp.pos.y() << ',' << resp.tcp.pos.z();
      record_file_ << ',' << resp.tcp.quat.x() << ',' << resp.tcp.quat.y()
                   << ',' << resp.tcp.quat.z() << ',' << resp.tcp.quat.w();
      for (int i = 0; i < 7; ++i) {record_file_ << ',' << resp.joint.tau(i);}
      record_file_ << '\n';
      ++record_rows_;
    };
  write_arm("left", dual_.Left());
  write_arm("right", dual_.Right());
}

}  // namespace nimo_arm_ros
