# Manipulator joint state, fed from outside (Raspberry Pi 5 on hardware, Python script in SITL)
# Logged to the .ulg file together with all other default topics.

uint64 timestamp          # time since system start (microseconds), set on reception in PX4
uint64 timestamp_sample   # sender-side time of the sample (microseconds), 0 if unknown

uint8 NUM_JOINTS = 4

float32[4] angle          # [rad]
float32[4] velocity       # [rad/s]
float32[4] acceleration   # [rad/s^2]
float32[3] ff_torque      # Feedforward-Torque
