// Optional Run-loop diagnostics (debug mode).
// Design rules:
//  - Off by default; zero file I/O / logging in the hot path when disabled.
//  - Hot path only updates counters + a fixed ring (no heap, no printf).
//  - Periodic summary → ROS logger; optional CSV for offline analysis.
//  - No scattered debug prints in motion/FSM code — extend this module instead.
#pragma once

#include <cstdint>
#include <fstream>
#include <string>

namespace nimo_arm_ros
{

struct RunDiagConfig {
  bool enable = false;
  std::string dir = "/tmp/nimo_arm_diag";
  // Write one CSV row every N cycles when enable (0 = summary-only, no per-cycle file).
  uint32_t csv_stride = 0;
  // Logger summary every N cycles (~5 s @ 500 Hz when 2500).
  uint32_t summary_stride = 2500;
};

class RunDiagnostics
{
public:
  void Configure(const RunDiagConfig & cfg);
  void Close();

  // Call once per Run period after work. dt_s = measured period; work_s = dual_.Run duration.
  void OnCycle(double dt_s, double work_s, bool overrun);

  // Edge-triggered op/err (from publish thread is fine).
  void OnOpEdge(const char * side, int op, const char * err, const char * reject);

  bool enabled() const {return cfg_.enable;}
  const std::string & session_path() const {return session_path_;}

private:
  void EnsureOpen();
  void WriteSummaryRow();

  RunDiagConfig cfg_{};
  std::ofstream file_;
  std::string session_path_;
  uint64_t cycles_{0};
  uint64_t overruns_{0};
  double dt_sum_{0.0};
  double dt_max_{0.0};
  double work_sum_{0.0};
  double work_max_{0.0};
  int prev_op_l_{-1};
  int prev_op_r_{-1};
};

}  // namespace nimo_arm_ros
