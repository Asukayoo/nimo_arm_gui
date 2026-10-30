#include "nimo_arm_ros/rt_thread.hpp"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <unistd.h>

namespace {

bool g_rt_applied = false;

int ResolveCpu(int cpu)
{
  if (cpu == kRtCpuOff) {
    return -1;
  }
  const int ncpu = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
  if (ncpu <= 0) {
    return 0;
  }
  if (cpu == kRtCpuAutoLast) {
    return ncpu - 1;
  }
  return cpu;
}

void AbortRtSetup(const char * reason)
{
  std::fprintf(stderr, "[nimo_arm_ros rt] FATAL: %s\n", reason);
  std::fprintf(
    stderr,
    "[nimo_arm_ros rt] grant: sudo setcap cap_sys_nice,cap_ipc_lock+ep <dual_arm_node>\n"
    "  or: bash /home/mundane/deploy/ensure_rt.sh && rebuild with setcap\n");
  std::exit(1);
}

}  // namespace

RtThreadStatus ApplyRtThreadOptions(const RtThreadOptions & opt)
{
  RtThreadStatus status{};
  if (g_rt_applied) {
    return status;
  }
  g_rt_applied = true;

  if (prctl(PR_SET_TIMERSLACK, opt.timer_slack_ns) == 0) {
    status.timer_slack_ok = true;
  } else {
    std::fprintf(stderr, "[nimo_arm_ros rt] PR_SET_TIMERSLACK failed: %s\n", std::strerror(errno));
  }

  const int target_cpu = ResolveCpu(opt.cpu);
  const bool want_affinity = target_cpu >= 0;
  if (want_affinity) {
    const int ncpu = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
    if (target_cpu < ncpu) {
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(target_cpu, &cpuset);
      if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) == 0) {
        status.affinity_ok = true;
        status.bound_cpu = target_cpu;
      } else {
        std::fprintf(
          stderr, "[nimo_arm_ros rt] affinity cpu=%d failed: %s\n", target_cpu,
          std::strerror(errno));
      }
    } else {
      std::fprintf(stderr, "[nimo_arm_ros rt] cpu=%d out of range (ncpu=%d)\n", target_cpu, ncpu);
    }
  } else {
    status.affinity_ok = true;
  }

  if (opt.lock_memory) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
      status.memory_locked = true;
    } else {
      std::fprintf(
        stderr, "[nimo_arm_ros rt] mlockall failed: %s\n", std::strerror(errno));
    }
  }

  if (opt.use_fifo) {
    sched_param sp{};
    sp.sched_priority = opt.sched_priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) == 0) {
      status.fifo_ok = true;
    } else {
      std::fprintf(
        stderr, "[nimo_arm_ros rt] SCHED_FIFO prio=%d failed: %s\n", opt.sched_priority,
        std::strerror(errno));
    }
  } else {
    status.fifo_ok = true;
  }

  std::fprintf(
    stderr,
    "[nimo_arm_ros rt] slack=%s cpu=%d affinity=%s mlock=%s fifo=%s prio=%d\n",
    status.timer_slack_ok ? "ok" : "no", status.bound_cpu,
    want_affinity ? (status.affinity_ok ? "ok" : "no") : "skip",
    opt.lock_memory ? (status.memory_locked ? "ok" : "no") : "skip",
    opt.use_fifo ? (status.fifo_ok ? "ok" : "no") : "skip",
    opt.use_fifo ? opt.sched_priority : 0);

  if (!opt.abort_on_rt_failure) {
    return status;
  }
  if (want_affinity && !status.affinity_ok) {
    AbortRtSetup("CPU affinity required but not applied");
  }
  if (opt.use_fifo && !status.fifo_ok) {
    AbortRtSetup("SCHED_FIFO required but not granted");
  }
  return status;
}
