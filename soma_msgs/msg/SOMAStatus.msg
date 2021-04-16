Header header

# ATV hardware status
# STATUS = {0:'Stop', 1:'Starting', 2:'Travelling', 3:'Braking'}
uint16 status
string status_str

# motor positions (degree)
float32 steering_pos
float32 rear_pos
float32 front_pos
float32 throttle_pos

# cluch status
# {0:'Free', 1:'Foward', 2:'Backward'}
uint16 clutch_status
string clutch_status_str

# Verocity control
float32 wheel_vel     # wheel velocity (m/s)
float32 target_vel    # target velocity set by user (m/s)
float32[] vel_errors  # velocity error (t, t-1, t-2)
float32 PGain # P gain value
float32 DGain # D gain value
