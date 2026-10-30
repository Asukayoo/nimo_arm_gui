# Per-arm HMI snapshot: OpState + ref/resp (joints SI, TCP planning frame, arm angle φ).
std_msgs/Header header

# OpState: 0 Unconfigured 1 Standby 2 Enabling 3 Ready 4 Moving
#          5 Guiding 6 Disabling 7 Fault 8 EStop 9 Finalized
uint8 OP_UNCONFIGURED=0
uint8 OP_STANDBY=1
uint8 OP_ENABLING=2
uint8 OP_READY=3
uint8 OP_MOVING=4
uint8 OP_GUIDING=5
uint8 OP_DISABLING=6
uint8 OP_FAULT=7
uint8 OP_ESTOP=8
uint8 OP_FINALIZED=9
uint8 op_state

# ControlMode: 0 Position 1 JointImp 2 CartImp 3 Drag
uint8 MODE_POSITION=0
uint8 MODE_JOINT_IMP=1
uint8 MODE_CART_IMP=2
uint8 MODE_DRAG=3
uint8 control_mode
bool control_mode_settled

# MotionKind: 0 None 1 ServoJ 2 ServoP 3 ServoPByPico 4 MovJ 5 MovL
#             6 JogJ 7 JogCart 8 Stopping 9 ContactRetract
uint8 KIND_NONE=0
uint8 KIND_SERVO_J=1
uint8 KIND_SERVO_P=2
uint8 KIND_SERVO_P_BY_PICO=3
uint8 KIND_MOV_J=4
uint8 KIND_MOV_L=5
uint8 KIND_JOG_J=6
uint8 KIND_JOG_CART=7
uint8 KIND_STOPPING=8
uint8 KIND_CONTACT_RETRACT=9
uint8 motion_kind

uint8 error_code
string error_message
uint8 reject_reason
string reject_message
uint8 alarm_source
bool is_alarm
bool is_joint_vel_alarm
bool motion_done

uint8 BRAKE_HOLD=1
uint8 BRAKE_RELEASE=2
uint8 brake

# Measured (resp) / commanded (ref), SI: rad, m, quat wxyz
float64[7] q_resp
float64[7] q_ref
geometry_msgs/Pose tcp_resp
geometry_msgs/Pose tcp_ref
# Redundant arm angle φ [rad] from Kinematics::ComputeConfig
float64 phi_resp
float64 phi_ref
bool phi_resp_valid
bool phi_ref_valid
