# Data to be logged from the omnicopter controller for post flight Debugging and evaluation

uint64 timestamp          # time since system start (microseconds)

uint64 dt		# Position and Attitude control loop Ts

float32[3] pos		# ENU Position
float32[3] pos_sp		# ENU Position setpoint

float32[3] pos_error	# Position error
float32[3] pos_error_int	# Position error integral
float32[3] pos_error_der	# Position error derivative

float32[4] q_error	# Attitude Quaternion error
float32[4] q_error_int	# Attitude Quaternion error integral

float32[4] attq		# ENU Attitude Quaternion
float32[4] attq_sp	# ENU Attitude Quaternion setpoint
float32[3] omega_cmd	# ENU body commanded rotation rates.

float32[3] omega	# ENU body rotation rates
float32[3] omega_sp	# ENU desired body rotation rates

float32[3] f_cmd	# commanded body force in Newtons
float32[3] t_cmd	# comanded body force in Newton-meters

float32 batt_volt

float32[2] phi		# Motor optimization value
float32[2] f_rot_lim	# Rotor force limits
float32[8] f_rot_cmd	# commanded rotor force in Newtons
float32[8] f_rot_filt
