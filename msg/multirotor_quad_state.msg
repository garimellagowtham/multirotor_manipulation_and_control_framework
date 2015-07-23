# State Msg to be given to GUI

time quad_time # Quadcopter Autopilot time
float64 battery_voltage # Voltage of battery 

string quad_state # State of Quadcopter (Quad Status + Control Mode + Arm status )
uint8 position_controller_state # 0: ENABLED 1: DISABLED 2: CRITICAL
