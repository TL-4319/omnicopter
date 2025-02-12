/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

#include <float.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/controllib/blocks.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <matrix/Vector.hpp>
#include <matrix/Scalar.hpp>
#include <matrix/PseudoInverse.hpp>
//#include <lib/matrix/matrix/math.hpp>


#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <systemlib/err.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/omni_adap_debug.h>



using matrix::Eulerf;
using matrix::Quatf;
using matrix::Matrix3f;
using matrix::Vector3f;
using matrix::Vector;
using matrix::Matrix;
using matrix::Scalarf;
using matrix::Dcmf;
using uORB::SubscriptionData;


using namespace time_literals;

extern "C" __EXPORT int omni_pid_aback_main(int argc, char *argv[]);

class omni_controller : public ModuleBase<omni_controller>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	omni_controller(int example_param, bool example_flag);

	omni_controller();
	~omni_controller() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	static int custom_command(int argc, char *argv[]);

	bool init();

private:

	/** @see ModuleBase::run() */
	void Run() override;
	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);

	void update_position_control();
	void update_attitude_control();
	void update_motor_throttle();

	void update_pos_and_att();
	void pos_setpoint_poll();
	void att_setpoint_poll();

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::OMNI_POS_TAU>) _param_pos_tau,
		(ParamFloat<px4::params::OMNI_POS_TAU_I>) _param_pos_tau_i,
		(ParamFloat<px4::params::OMNI_POS_ETA>) _param_pos_eta,
		(ParamFloat<px4::params::OMNI_MASS>) _param_omni_m,
		(ParamFloat<px4::params::OMNI_ATT_K1>) _param_att_k1,
		(ParamFloat<px4::params::OMNI_ATT_K2>) _param_att_k2,
		(ParamFloat<px4::params::OMNI_ATT_GTHETA>) _param_att_gtheta,
		// (ParamFloat<px4::params::OMNI_ATT_GPSI>) _param_att_gpsi,
		(ParamFloat<px4::params::OMNI_J_XX>) _param_J_xx,
		(ParamFloat<px4::params::OMNI_J_YY>) _param_J_yy,
		(ParamFloat<px4::params::OMNI_J_ZZ>) _param_J_zz,
		(ParamFloat<px4::params::OMNI_CTRL_EPS>) _param_ctrl_eps,
		(ParamFloat<px4::params::OMNI_THROT_MIN>) _param_throt_min,
		(ParamFloat<px4::params::OMNI_THROT_MAX>) _param_throt_max,
		(ParamFloat<px4::params::OMNI_TAU_HYST>) _param_tau_hyst,
		(ParamFloat<px4::params::OMNI_TAU_F>) _param_tau_f,
		(ParamInt<px4::params::OMNI_DEBUG>) _param_debug_bool
	)

	// Global Variables
	perf_counter_t	_loop_perf;

	struct vehicle_local_position_setpoint_s _pos_sp;
	struct vehicle_local_position_s _pos;

	struct vehicle_attitude_s _att;
	struct vehicle_attitude_setpoint_s _att_sp;
	struct vehicle_angular_velocity_s _att_rate;
	struct vehicle_rates_setpoint_s _att_rate_sp;

	struct battery_status_s _batt_stats;
	struct actuator_controls_s _motor_cmd;
	struct omni_adap_debug_s _debug_file;

	bool _debug = false;
	int loop_count = 0;

	// Run() function
	bool fly_veh = false;

	uint64_t _t_old = 0;
	uint64_t _dt;
	// update_pos_control()
	Vector3f _pos_error_int;
	Vector3f _pos_error_der;
	Vector3f _pos_error_der_old;
	Vector3f _pos_error_der_filt;
	Vector3f _pos_error_old;
	Vector3f _f_cmd;

	// update_att_control()
	Quatf _q_error_old;

	// update_angular_rate_control()
	Vector3f _t_cmd;

	// update_motor_throttle()
	float _B_coefs[48] = {-0.7887,    0.2113,   -0.2113,    0.7887,    0.7887,   -0.2113,    0.2113,   -0.7887,
			0.2113,    0.7887,   -0.7887,   -0.2113,   -0.2113,   -0.7887,    0.7887,    0.2113,
			0.5774,   -0.5774,   -0.5774,    0.5774,    0.5774,   -0.5774,   -0.5774,    0.5774,
			0.0254,   -0.1386,    0.1386,   -0.0254,    0.0254,   -0.1386,    0.1386,   -0.0254,
			-0.1386,   -0.0254,    0.0254,    0.1386,   -0.1386,   -0.0254,    0.0254,    0.1386,
			0.1131,   -0.1131,   -0.1131,    0.1131,   -0.1131,    0.1131,    0.1131,   -0.1131};


	Matrix<float, 8, 6> _B_pinv;
	Vector<float, 8> _f_rot_cmd;


	// update_pos_and_att()
	Vector3f _pos_enu;
	Quatf _attq_enu;
	Matrix3f R_q;

	// update_pos_sp()
	Vector3f _pos_sp_enu;
	Vector3f _acc_sp_enu;

	// update_att_sp()
	Quatf _attq_sp_enu;
	Vector3f _att_rate_sp_enu;
	Vector3f _w_des_dot_sp_enu;
	Matrix3f thetahat;
	Vector3f _w_cmd_old;

	// parameter_update()
	Vector3f K_pos;
	Matrix3f K1;
	Matrix3f K2;
	Matrix3f Gtheta;
	Matrix3f _J;
	float m_omni;
	float Tau_f;
	float eps;
	float throt_max;
	float throt_min;
	float tau_hyst;


	// Publishers
	uORB::Publication<actuator_controls_s> _actuator_pub {ORB_ID(actuator_controls_0)};
	uORB::Publication<omni_adap_debug_s> _debug_pub {ORB_ID(omni_adap_debug)};

	// Subscribers
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionInterval _att_rate_sub{ORB_ID(vehicle_angular_velocity), 5000};
	uORB::SubscriptionInterval _pos_sub{ORB_ID(vehicle_local_position), 5000};
	uORB::SubscriptionInterval _att_sub{ORB_ID(vehicle_attitude), 5000};

	uORB::Subscription _pos_sp_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _att_rate_sp_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _batt_stat_sub{ORB_ID(battery_status)};
};
