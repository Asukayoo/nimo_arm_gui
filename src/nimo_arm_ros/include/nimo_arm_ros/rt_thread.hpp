// POSIX RT helpers for the dual_arm 500 Hz Run thread.
#pragma once

inline constexpr int kRtCpuOff = -1;       // no affinity
inline constexpr int kRtCpuAutoLast = -2;  // bind last online CPU

struct RtThreadOptions {
  int cpu = kRtCpuAutoLast;
  int sched_priority = 80;  // SCHED_FIFO 1–99
  bool use_fifo = true;
  bool lock_memory = true;  // mlockall
  int timer_slack_ns = 1;   // prctl PR_SET_TIMERSLACK
  bool abort_on_rt_failure = false;
};

struct RtThreadStatus {
  bool timer_slack_ok = false;
  bool affinity_ok = false;
  bool memory_locked = false;
  bool fifo_ok = false;
  int bound_cpu = -1;
};

// Apply to the calling thread. Idempotent process-wide (first call wins).
RtThreadStatus ApplyRtThreadOptions(const RtThreadOptions& opt = {});
