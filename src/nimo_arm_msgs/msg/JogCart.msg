# Cartesian jog: axis 0..2 lin, 3..5 ang, 6 Phi.
# rate is unitless direction in [-1, 1]; backend scales by arm.yaml limits.jog.cart.
# 0 = decelerate (do not call Stop for jog release).
uint8 AXIS_VX=0
uint8 AXIS_VY=1
uint8 AXIS_VZ=2
uint8 AXIS_WX=3
uint8 AXIS_WY=4
uint8 AXIS_WZ=5
uint8 AXIS_PHI=6
uint8 axis
float64 rate

