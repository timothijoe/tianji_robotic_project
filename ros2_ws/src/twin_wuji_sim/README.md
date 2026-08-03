# twin_wuji_sim

This ROS 2 package owns message transport and simulation bridging for the Wuji
hand. It must not import or connect to the vendor SDK.

Real-device lifecycle and SDK calls belong to
`tianji_robotics.hardware.wuji_hand`. The current `tianji-robot hardware
wuji-sdk preflight` command validates an offline trajectory only; it never
discovers, connects, arms, or commands a physical hand.
