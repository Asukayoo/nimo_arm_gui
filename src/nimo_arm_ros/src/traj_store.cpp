#include "nimo_arm_ros/traj_store.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace nimo_arm_ros
{
namespace
{

constexpr const char * kMagic = "# nimo_arm_traj v1";

std::vector<double> SgCoeffs(int window)
{
  // Precompute quadratic SG coefficients for common odd windows via normal equations
  // on indices i = -m..m, fit a0+a1*i+a2*i^2, take a0 row (value at center).
  const int m = window / 2;
  // Gram: sum i^p for p=0..4
  double s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0;
  for (int i = -m; i <= m; ++i) {
    const double ii = static_cast<double>(i);
    const double i2 = ii * ii;
    s0 += 1.0;
    s1 += ii;
    s2 += i2;
    s3 += i2 * ii;
    s4 += i2 * i2;
  }
  // Solve 3x3 for basis [1,i,i^2]; we only need the first row of inv(G)*ones_basis
  // G = [[s0,s1,s2],[s1,s2,s3],[s2,s3,s4]]
  const double det =
    s0 * (s2 * s4 - s3 * s3) - s1 * (s1 * s4 - s3 * s2) + s2 * (s1 * s3 - s2 * s2);
  std::vector<double> c(static_cast<size_t>(window), 0.0);
  if (std::abs(det) < 1e-12) {
    // Fallback: uniform moving average
    const double w = 1.0 / static_cast<double>(window);
    for (double & v : c) {v = w;}
    return c;
  }
  // Inverse first row (for a0): Cramer's / adjugate row 0
  const double i00 = (s2 * s4 - s3 * s3) / det;
  const double i01 = (s3 * s2 - s1 * s4) / det;
  const double i02 = (s1 * s3 - s2 * s2) / det;
  for (int i = -m; i <= m; ++i) {
    const double ii = static_cast<double>(i);
    c[static_cast<size_t>(i + m)] = i00 + i01 * ii + i02 * ii * ii;
  }
  return c;
}

void SmoothArm(
  const std::vector<TrajRow> & src, std::vector<TrajRow> & dst, int window)
{
  const auto coeff = SgCoeffs(window);
  const int m = window / 2;
  const int n = static_cast<int>(src.size());
  dst.resize(src.size());
  for (int k = 0; k < n; ++k) {
    dst[static_cast<size_t>(k)] = src[static_cast<size_t>(k)];
    for (int j = 0; j < 7; ++j) {
      double acc = 0.0;
      for (int di = -m; di <= m; ++di) {
        int idx = k + di;
        if (idx < 0) {idx = 0;}
        if (idx >= n) {idx = n - 1;}
        acc += coeff[static_cast<size_t>(di + m)] *
          src[static_cast<size_t>(idx)].q[static_cast<size_t>(j)];
      }
      dst[static_cast<size_t>(k)].q[static_cast<size_t>(j)] = acc;
    }
  }
}

std::vector<TrajRow> ResampleArm(const std::vector<TrajRow> & src, double dt)
{
  std::vector<TrajRow> out;
  if (src.empty() || dt <= 0.0) {return out;}
  const double t0 = src.front().t_sec;
  const double t1 = src.back().t_sec;
  if (t1 <= t0) {
    out.push_back(src.front());
    return out;
  }
  size_t i = 0;
  for (double t = t0; t <= t1 + 1e-12; t += dt) {
    while (i + 1 < src.size() && src[i + 1].t_sec < t) {++i;}
    TrajRow row;
    row.t_sec = t - t0;  // relative time from start
    row.left = src.front().left;
    if (i + 1 >= src.size() || src[i + 1].t_sec <= src[i].t_sec) {
      row.q = src[i].q;
    } else {
      const double a =
        (t - src[i].t_sec) / (src[i + 1].t_sec - src[i].t_sec);
      for (int j = 0; j < 7; ++j) {
        row.q[static_cast<size_t>(j)] =
          src[i].q[static_cast<size_t>(j)] * (1.0 - a) +
          src[i + 1].q[static_cast<size_t>(j)] * a;
      }
    }
    out.push_back(row);
  }
  return out;
}

}  // namespace

std::string LoadTraj(const std::string & path, TrajFile & out)
{
  out.rows.clear();
  std::ifstream in(path);
  if (!in) {return "cannot open " + path;}
  std::string line;
  bool saw_header = false;
  while (std::getline(in, line)) {
    if (line.empty()) {continue;}
    if (line[0] == '#') {
      if (line.find("nimo_arm_traj") != std::string::npos) {saw_header = true;}
      continue;
    }
    if (line.rfind("t_sec,", 0) == 0) {continue;}  // column header
    std::stringstream ss(line);
    std::string arm;
    TrajRow row;
    char comma;
    if (!(ss >> row.t_sec)) {return "bad t_sec in " + path;}
    if (!(ss >> comma) || comma != ',') {return "bad csv in " + path;}
    if (!std::getline(ss, arm, ',')) {return "bad arm in " + path;}
    if (arm == "left") {row.left = true;}
    else if (arm == "right") {row.left = false;}
    else {return "unknown arm '" + arm + "' in " + path;}
    // After arm field: q0,q1,... (no leading comma to consume before q0)
    for (int j = 0; j < 7; ++j) {
      if (j > 0) {
        if (!(ss >> comma) || comma != ',') {return "bad q csv in " + path;}
      }
      if (!(ss >> row.q[static_cast<size_t>(j)])) {return "bad q in " + path;}
    }
    out.rows.push_back(row);
  }
  if (out.rows.empty()) {return "empty traj " + path;}
  if (!saw_header) {
    // Accept files without magic if schema matches (HMI may strip comments).
  }
  return {};
}

std::string SaveTraj(const std::string & path, const TrajFile & in)
{
  std::ofstream out(path, std::ios::out | std::ios::trunc);
  if (!out) {return "cannot open " + path;}
  out << kMagic << '\n';
  out << "t_sec,arm,q0,q1,q2,q3,q4,q5,q6\n";
  out.setf(std::ios::fixed);
  out.precision(9);
  for (const auto & row : in.rows) {
    out << row.t_sec << ',' << (row.left ? "left" : "right");
    for (double v : row.q) {out << ',' << v;}
    out << '\n';
  }
  return {};
}

std::string PostprocessTraj(
  const TrajFile & in, TrajFile & out, int sg_window, double resample_dt)
{
  if (in.rows.empty()) {return "empty input";}
  int win = sg_window > 0 ? sg_window : 9;
  if (win % 2 == 0) {++win;}
  if (win < 5) {win = 5;}
  const double dt = resample_dt > 0.0 ? resample_dt : 0.02;

  std::vector<TrajRow> left, right;
  for (const auto & r : in.rows) {
    if (r.left) {left.push_back(r);}
    else {right.push_back(r);}
  }
  auto process = [&](std::vector<TrajRow> & arm) {
      if (arm.size() < static_cast<size_t>(win)) {return arm;}
      std::vector<TrajRow> smoothed;
      SmoothArm(arm, smoothed, win);
      return ResampleArm(smoothed, dt);
    };
  left = process(left);
  right = process(right);

  out.rows.clear();
  // Interleave by index (same resample grid from each arm's own t0).
  const size_t n = std::max(left.size(), right.size());
  for (size_t i = 0; i < n; ++i) {
    if (i < left.size()) {out.rows.push_back(left[i]);}
    if (i < right.size()) {out.rows.push_back(right[i]);}
  }
  if (out.rows.empty()) {return "postprocess produced empty traj";}
  return {};
}

std::vector<TrajFrame> ToFrames(const TrajFile & file)
{
  std::unordered_map<long long, TrajFrame> by_key;
  auto key_of = [](double t) -> long long {
      return static_cast<long long>(std::llround(t * 1e6));  // us
    };
  for (const auto & r : file.rows) {
    auto & fr = by_key[key_of(r.t_sec)];
    fr.t_sec = r.t_sec;
    if (r.left) {
      fr.has_left = true;
      fr.left_q = r.q;
    } else {
      fr.has_right = true;
      fr.right_q = r.q;
    }
  }
  std::vector<TrajFrame> frames;
  frames.reserve(by_key.size());
  for (auto & kv : by_key) {frames.push_back(kv.second);}
  std::sort(frames.begin(), frames.end(),
    [](const TrajFrame & a, const TrajFrame & b) {return a.t_sec < b.t_sec;});
  return frames;
}

}  // namespace nimo_arm_ros
