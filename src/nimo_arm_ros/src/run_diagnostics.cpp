#include "nimo_arm_ros/run_diagnostics.hpp"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;

namespace nimo_arm_ros
{
namespace
{

std::string MakeSessionPath(const std::string & dir)
{
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  std::ostringstream oss;
  oss << dir << "/run_" << ms << ".csv";
  return oss.str();
}

}  // namespace

void RunDiagnostics::Configure(const RunDiagConfig & cfg)
{
  Close();
  cfg_ = cfg;
  cycles_ = 0;
  overruns_ = 0;
  dt_sum_ = dt_max_ = work_sum_ = work_max_ = 0.0;
  prev_op_l_ = prev_op_r_ = -1;
  session_path_.clear();
  if (!cfg_.enable) {
    return;
  }
  EnsureOpen();
}

void RunDiagnostics::Close()
{
  if (file_.is_open()) {
    WriteSummaryRow();
    file_.flush();
    file_.close();
  }
}

void RunDiagnostics::EnsureOpen()
{
  if (file_.is_open() || !cfg_.enable) {
    return;
  }
  std::error_code ec;
  fs::create_directories(cfg_.dir, ec);
  session_path_ = MakeSessionPath(cfg_.dir);
  file_.open(session_path_, std::ios::out | std::ios::trunc);
  if (!file_) {
    std::fprintf(stderr, "[nimo_arm_ros diag] open failed: %s\n", session_path_.c_str());
    session_path_.clear();
    return;
  }
  file_ << "# nimo_arm_ros run_diagnostics v1\n";
  file_ << "# kind,t_mono_s,cycle,dt_ms,work_ms,overrun,side,op,err,reject\n";
  file_ << std::setprecision(9);
}

void RunDiagnostics::OnCycle(double dt_s, double work_s, bool overrun)
{
  if (!cfg_.enable) {
    return;
  }
  ++cycles_;
  if (overrun) {
    ++overruns_;
  }
  dt_sum_ += dt_s;
  work_sum_ += work_s;
  if (dt_s > dt_max_) {
    dt_max_ = dt_s;
  }
  if (work_s > work_max_) {
    work_max_ = work_s;
  }

  if (cfg_.csv_stride > 0 && file_.is_open() && (cycles_ % cfg_.csv_stride) == 0) {
    using clock = std::chrono::steady_clock;
    const double t = std::chrono::duration<double>(clock::now().time_since_epoch()).count();
    file_ << "cycle," << t << ',' << cycles_ << ',' << (dt_s * 1e3) << ',' << (work_s * 1e3) << ','
          << (overrun ? 1 : 0) << ",,,,\n";
  }

  if (cfg_.summary_stride > 0 && (cycles_ % cfg_.summary_stride) == 0) {
    WriteSummaryRow();
    const double n = static_cast<double>(cfg_.summary_stride);
    const double dt_avg = dt_sum_ / n;
    const double work_avg = work_sum_ / n;
    std::fprintf(
      stderr,
      "[nimo_arm_ros diag] cycles=%llu overruns=%llu dt_avg=%.3fms dt_max=%.3fms "
      "work_avg=%.3fms work_max=%.3fms file=%s\n",
      static_cast<unsigned long long>(cycles_), static_cast<unsigned long long>(overruns_),
      dt_avg * 1e3, dt_max_ * 1e3, work_avg * 1e3, work_max_ * 1e3,
      session_path_.empty() ? "-" : session_path_.c_str());
    dt_sum_ = work_sum_ = 0.0;
    dt_max_ = work_max_ = 0.0;
  }
}

void RunDiagnostics::OnOpEdge(const char * side, int op, const char * err, const char * reject)
{
  if (!cfg_.enable || side == nullptr) {
    return;
  }
  int & prev = (side[0] == 'l' || side[0] == 'L') ? prev_op_l_ : prev_op_r_;
  if (op == prev) {
    return;
  }
  prev = op;
  EnsureOpen();
  if (!file_.is_open()) {
    return;
  }
  using clock = std::chrono::steady_clock;
  const double t = std::chrono::duration<double>(clock::now().time_since_epoch()).count();
  file_ << "op," << t << ',' << cycles_ << ",,,," << side << ',' << op << ','
        << (err ? err : "") << ',' << (reject ? reject : "") << '\n';
}

void RunDiagnostics::WriteSummaryRow()
{
  if (!file_.is_open() || cycles_ == 0) {
    return;
  }
  using clock = std::chrono::steady_clock;
  const double t = std::chrono::duration<double>(clock::now().time_since_epoch()).count();
  file_ << "summary," << t << ',' << cycles_ << ',' << (dt_max_ * 1e3) << ',' << (work_max_ * 1e3)
        << ',' << overruns_ << ",,,,\n";
  file_.flush();
}

}  // namespace nimo_arm_ros
