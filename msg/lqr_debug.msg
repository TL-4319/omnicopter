# Data to be logged from the LQR controller for post flight Debugging and evaluation

uint64 timestamp          # time since system start (microseconds)

float32[4] body_forces

float32 batt_volt

float32 pos_dt
float32 att_dt

float32[9] pos_state
float32[9] att_state

float32[3] pos_sp
float32[3] att_sp
