# Data to be logged from the omnicopter controller for post flight Debugging and evaluation

uint64 timestamp          # time since system start (microseconds)

uint64 dt		# Position and Attitude control loop Ts

float32[3] pos		# ENU Position
float32[3] vel
float32[3] pos_sp		# ENU Position setpoint
float32[3] vel_sp

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
float32[3] omega_dot_sp

float32[3] f_cmd	# commanded body force in Newtons
float32[3] t_cmd	# comanded body force in Newton-meters

float32 batt_volt

float32[2] phi		# Motor optimization value
float32[2] f_rot_lim	# Rotor force limits
float32[8] f_rot_cmd	# commanded rotor force in Newtons
float32[8] f_rot_filt
float32[8] motor_tau
float32[2] mot_filt

float32[9] xr
float32[9] x
float32[9] xd
float32[9] xdbar
float32[3] theta_pid_one
float32[3] theta_pid_two
float32[3] theta_pid_three
float32[3] fbl
float32[3] fad
float32[3] u

float32[3] psihat
float32[3] theta_ab_one
float32[3] theta_ab_two
float32[3] theta_ab_three

float32[3] theta_att_one
float32[3] theta_att_two
float32[3] theta_att_three
float32[3] lambdahat_one
float32[3] lambdahat_two
float32[3] lambdahat_three
float32[3] tdhat
float32[3] tbl
float32[3] tad
float32[3] tcest
float32[3] wref
float32[3] logv

float32 pos_sat_corr
float32 att_sat_corr
float32 u_norm_max
