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
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/lqr_debug.h>



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

extern "C" __EXPORT int lqr_controller_main(int argc, char *argv[]);


class lqr_controller : public ModuleBase<lqr_controller>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	lqr_controller(int example_param, bool example_flag);

	lqr_controller();
	~lqr_controller() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:

	/** @see ModuleBase::run() */
	void Run() override;

	perf_counter_t	_loop_perf;
	int loop_counter = 0;
	//*** LQR Controller functions ***//

	// Updates postiion controller (running at 50 Hz) based on vehicles
	// position state (x,y,z,u,v,w) and reference error integral.
	void update_position_control();

	// Updates attitude controller (running at 400 Hz) based on vehicles
	// attitude state (phi,theta,psi,phi_dot,theta_dot,psi_dot) and reference
	// error integral.
	void update_attitude_control();

	// Updates motor throttle setpoints based on desired Body force, F_t, and body
	// torques, tau_x, tau_y, and tau_z
	void update_motor_throttle();

	// Checks if trajectory_setpoint topic has been updated and if so, copies new setpoint
	// to the structure _pos.
	void vehicle_local_position_setpoint_poll();

	// Updates parameter based variables whenever a parameter has been updated
	void update_param_based_vars();

	// Global Variables


	struct vehicle_local_position_setpoint_s _pos_sp;
	struct vehicle_local_position_s _pos;
	struct vehicle_attitude_s _att;
	struct vehicle_odometry_s _vehicle_odom;

	struct actuator_controls_s _motor_cmd;
	struct lqr_debug_s _debug_file;

	float pos_t_old = 0;
	float att_t_old = 0;
	Vector3f att_err_old;
	Vector3f pos_err_int;
	Vector3f att_err_int;
	Matrix<float, 3, 1> pos_control;
	Matrix<float, 3, 1> att_control;
	Matrix<float, 4, 4> C_inv;

	Matrix<float, 3, 9> K_att;
	Matrix<float, 3, 9> K_pos;

	bool _debug_;

	bool fly_veh = false;
	int16_t loop_count = 0;
	float z_int_init = 0;

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::C_T>) _param_c_T,
		(ParamFloat<px4::params::C_TAU>) _param_c_tau,
		(ParamFloat<px4::params::LQR_VEH_WT>) _param_veh_wt,
		// K_pos parameters
		(ParamFloat<px4::params::K_POS_COEF_0>) _param_K_pos_1,
		(ParamFloat<px4::params::K_POS_COEF_1>) _param_K_pos_2,
		(ParamFloat<px4::params::K_POS_COEF_2>) _param_K_pos_3,
		(ParamFloat<px4::params::K_POS_COEF_3>) _param_K_pos_4,
		(ParamFloat<px4::params::K_POS_COEF_4>) _param_K_pos_5,
		(ParamFloat<px4::params::K_POS_COEF_5>) _param_K_pos_6,
		(ParamFloat<px4::params::K_POS_COEF_6>) _param_K_pos_7,
		(ParamFloat<px4::params::K_POS_COEF_7>) _param_K_pos_8,
		(ParamFloat<px4::params::K_POS_COEF_8>) _param_K_pos_9,
		// K_att parameters
		(ParamFloat<px4::params::K_ATT_COEF_0>) _param_K_att_1,
		(ParamFloat<px4::params::K_ATT_COEF_1>) _param_K_att_2,
		(ParamFloat<px4::params::K_ATT_COEF_2>) _param_K_att_3,
		(ParamFloat<px4::params::K_ATT_COEF_3>) _param_K_att_4,
		(ParamFloat<px4::params::K_ATT_COEF_4>) _param_K_att_5,
		(ParamFloat<px4::params::K_ATT_COEF_5>) _param_K_att_6,
		(ParamFloat<px4::params::K_ATT_COEF_6>) _param_K_att_7,
		(ParamFloat<px4::params::K_ATT_COEF_7>) _param_K_att_8,
		(ParamFloat<px4::params::K_ATT_COEF_8>) _param_K_att_9,
		// Debuging parameter
		(ParamInt<px4::params::LQR_DEBUG>) _param_debug
	)
	// Publications

	uORB::Publication<actuator_controls_s> _actuator_pub {ORB_ID(actuator_controls_0)};
	uORB::Publication<lqr_debug_s> _debug_pub {ORB_ID(lqr_debug)};

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _pos_sp_sub{ORB_ID(trajectory_setpoint)};
	uORB::SubscriptionInterval _pos_sub{ORB_ID(vehicle_local_position), 20000};
	uORB::SubscriptionInterval _att_sub{ORB_ID(vehicle_attitude), 2000};
	uORB::Subscription _odom_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _batt_stat_sub{ORB_ID(battery_status)};
};

