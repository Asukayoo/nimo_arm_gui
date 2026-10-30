#pragma once

// Trajectory file I/O + offline SG smooth / resample.
// Contract: .traj only (not status Record CSV under records/).

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace nimo_arm_ros
{

struct TrajRow
{
  double t_sec{0.0};
  bool left{true};  // false = right
  std::array<double, 7> q{};
};

struct TrajFile
{
  std::vector<TrajRow> rows;
};

// Returns empty message on success; otherwise error text.
std::string LoadTraj(const std::string & path, TrajFile & out);
std::string SaveTraj(const std::string & path, const TrajFile & in);

// Savitzky–Golay (quadratic) per joint, then uniform resample.
// sg_window must be odd >= 5; resample_dt > 0.
std::string PostprocessTraj(
  const TrajFile & in, TrajFile & out, int sg_window, double resample_dt);

// Group interleaved left/right rows into synchronized frames (same t).
struct TrajFrame
{
  double t_sec{0.0};
  bool has_left{false};
  bool has_right{false};
  std::array<double, 7> left_q{};
  std::array<double, 7> right_q{};
};

std::vector<TrajFrame> ToFrames(const TrajFile & file);

}  // namespace nimo_arm_ros
