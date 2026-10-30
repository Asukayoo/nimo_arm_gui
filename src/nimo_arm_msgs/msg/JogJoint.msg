# Single-joint jog: joint in [0,6].
# dq is a unitless direction in [-1, 1]; backend scales by arm.yaml limits.jog.joint.max_v.
# 0 = decelerate (do not call Stop for jog release).
int32 joint
float64 dq

