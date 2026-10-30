std_msgs/Header header
bool initialized
bool run_thread_alive
float64 ts
# Motion-form gates (orthogonal to ArmStatus.control_mode); Pico ⟂ ServoJ.
bool teleop_motion_enabled
bool servoj_teleop_enabled
bool traj_recording
bool traj_replaying
# Latest teleop/lifecycle event for HMI (edge-detect via last_event_seq).
string last_event
uint32 last_event_seq
ArmStatus left
ArmStatus right
