# MotorCommandStamped.msg
# This message is used to publish the thrust values computed by the MPC controller.

# Created by:
# Benjamin Armentor
# benarmentor@gmail.com

# Message Fields
Header header # Generic Stamp

float32 Tpb	# Float32 prescribing thrust/input to Port-Bow Thruster
float32 Tps # Float32 prescribing thrust/input to Port-Stern Thruster
float32 Tsb # Float32 prescribing thrust/input to Starboard-Bow Thruster
float32 Tss # Float32 prescribing thrust/input to Starboard-Stern Thruster