# Gui Commands to Onboard Node
bool quad_arm #Arm the quadcopter
bool manipulator_arm #Arm the manipulator
bool enable_quadcontrol # Enable controller for quadcopter
bool enable_armcontrol #Enable Arm Control for Quadcopter

uint8 arm_controlmode #Mode for arm control
uint8 quad_controlmode #Mode for quadcopter control
uint16 mask #Mask to specify which commands to us 

uint8 OPTIMAL_CONTROL=1
uint8 POSITION_CONTROL=2

uint8 MARKER_CONTROL=1
uint8 JOINT_CONTROL=2
uint8 ENDEFFECTOR_CONTROL=3
