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

/***************************************************************************
 *
 * 	Quadrotor LQR Servo controller
 *
 * 	This Program implements an LQR servo controller for a quad copter.
 * 	The program is a position controller running at 50 Hz with a nested
 * 	attitude controller running at 400 Hz.
 *
 * 	Author: Patrick Thomas
 * 	Date: 10/28/2021
 *
 * ************************************************************************/

#include "lqr_controller.hpp"

lqr_controller::lqr_controller():
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	parameters_update(true);
}


lqr_controller::~lqr_controller()
{
	perf_free(_loop_perf);
}


int lqr_controller::print_status()
{
	PX4_INFO("Running");

	return 0;
}



int lqr_controller::task_spawn(int argc, char *argv[])
{

	lqr_controller *instance = new lqr_controller();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;

}

bool lqr_controller::init(){
	ScheduleOnInterval(2500_us);
	z_int_init = -1.1f*(_param_veh_wt.get()/1000.0f)*9.81f/_param_K_pos_3.get();
	pos_err_int(2) = z_int_init;
	_pos_sp.x = 0;
	_pos_sp.y = 0;
	_pos_sp.z = 0;
	_pos_sp.yaw = 0;
	for(int i = 0; i<4; i++){
		_motor_cmd.control[i] = -0.6;
	}
	_actuator_pub.publish(_motor_cmd);
	return true;
}



void lqr_controller::Run()
{
	if(should_exit()){
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	// Check if parameters have been updated
	parameters_update();

	actuator_armed_s arm_check;
	armed_sub.copy(&arm_check);

	// Verify Vehicle is armed
	if(arm_check.armed && fly_veh){
		bool update_actuators = false;
		if(_pos_sub.updated()){
			update_position_control();
			update_actuators = true;
		}
		if(_att_sub.updated()){
			update_attitude_control();
			update_actuators = true;
		}
		if(update_actuators){
			update_motor_throttle();
		}
	}
	// Reset error inegrators to 0 when vehicle is disarmed
	else if (!arm_check.armed && fly_veh){
		pos_err_int = {0.0f, 0.0f, z_int_init};
		att_err_int = {0.0f, 0.0f, 0.0f};
		fly_veh = false;
		loop_count = 0;
	}
	else if (arm_check.armed && !fly_veh){
		if(loop_count < 10){
			for(int i = 0; i<4; i++){
				_motor_cmd.control[i] = -0.6;
			}
			_actuator_pub.publish(_motor_cmd);
		}
		else if(loop_count > 400){
			fly_veh = true;
		}
		loop_count++;
	}

	perf_end(_loop_perf);

	// struct vehicle_odometry_s vehicle_velocity_frame;
	// memset(&vehicle_velocity_frame, 0, sizeof(vehicle_velocity_frame));
	// vehicle_velocity_frame.velocity_frame = 3;		// Set velocity frame to be vehicle body FRD
	// orb_advert_t vehicle_odom_pub = orb_advertise(ORB_ID(vehicle_odometry), &vehicle_velocity_frame);
	// orb_publish(ORB_ID(vehicle_odometry), vehicle_odom_pub, &vehicle_velocity_frame);
}

void lqr_controller::update_position_control(){
	_pos_sub.copy(&_pos);
	_odom_sub.copy(&_vehicle_odom);
	vehicle_local_position_setpoint_poll();
	Vector3f xyz(_pos.x, _pos.y, _pos.z);
	Vector3f xyz_sp(_pos_sp.x, _pos_sp.y, _pos_sp.z);
	Vector3f pos_err = xyz_sp - xyz;

	// position error integrator
	float pos_dt = (hrt_absolute_time() - pos_t_old) / 1000000.0f;
	pos_t_old += pos_dt*1000000.0f;
	if(pos_dt > 0.03f){
		PX4_WARN("Position dT high: %f", (double)pos_dt);
		pos_dt = 0.03f;
	}
	pos_err_int += pos_err*pos_dt;

	// position state matrix compositon
	Matrix<float, 9, 1> pos_state;
	pos_state(0,0) = _pos.x;
	pos_state(1,0) = _pos.y;
	pos_state(2,0) = _pos.z;
	pos_state(3,0) = _vehicle_odom.vx;
	pos_state(4,0) = _vehicle_odom.vy;
	pos_state(5,0) = _vehicle_odom.vz;
	pos_state(6,0) = pos_err_int(0);
	pos_state(7,0) = pos_err_int(1);
	pos_state(8,0) = pos_err_int(2);

	// positon control calculaton
	pos_control = -K_pos*pos_state;

	if(_debug_){
		for (int i = 0; i<9; i++){
			_debug_file.pos_state[i] = pos_state(i,0);
		}
		for (int i = 0; i<3; i++){
			_debug_file.pos_sp[i] = xyz_sp(i);
		}
		_debug_file.pos_dt = pos_dt;
	}
}

void lqr_controller::update_attitude_control(){
	_att_sub.copy(&_att);
	_odom_sub.copy(&_vehicle_odom);
	vehicle_local_position_setpoint_poll();
	Quatf q_att(_att.q[0], _att.q[1], _att.q[2], _att.q[3]);
	Eulerf att_euler_angles(q_att);
	Vector3f att_sp;

	// Rotate phi and theta setpoints from NED frame to FRD body fixed frame
	att_sp(0) = (float)cos(att_euler_angles(2))*pos_control(1, 0) + (float)sin(att_euler_angles(2))*pos_control(2, 0);
	att_sp(1) = -(float)sin(att_euler_angles(2))*pos_control(1, 0) + (float)cos(att_euler_angles(2))*pos_control(2, 0);
	att_sp(2) = _pos_sp.yaw;

	// attitude error integrator
	float att_dt = (hrt_absolute_time() - att_t_old) / 1000000.0f;
	att_t_old += att_dt*1000000.0f;
	if(att_dt > 0.015f){
		PX4_WARN("Attitude dT high: %f", (double)att_dt);
		att_dt = 0.015f;
	}
	Vector3f att_err = att_sp - att_euler_angles;
	for (int i = 0; i < 3; i++){
		if(_pos.z > -0.25f && att_err(i) > 0.98f*att_err_old(i) && att_err(i) < 1.02f*att_err_old(i)){
			att_err_old(i) = att_err(i);
			att_err(i) = 0;
		}
		else{
			att_err_old(i) = att_err(i);
		}
	}
	att_err_int += att_err*att_dt;

	// attitude state matrix composition
	Matrix<float, 9, 1> att_state;
	att_state(0,0) = att_euler_angles(0);
	att_state(1,0) = att_euler_angles(1);
	att_state(2,0) = att_euler_angles(2);
	att_state(3,0) = _vehicle_odom.rollspeed;
	att_state(4,0) = _vehicle_odom.pitchspeed;
	att_state(5,0) = _vehicle_odom.yawspeed;
	att_state(6,0) = att_err_int(0);
	att_state(7,0) = att_err_int(1);
	att_state(8,0) = att_err_int(2);

	// attitude control calulation
	att_control = -K_att*att_state;

	if(_debug_){
		for (int i = 0; i<9; i++){
			_debug_file.att_state[i] = att_state(i,0);
		}
		for (int i = 0; i<3; i++){
			_debug_file.att_sp[i] = att_sp(i);
		}
		_debug_file.att_dt = att_dt;
	}
}

void lqr_controller::update_motor_throttle(){
	//float max_rpm = 1150;	// rad/s
	struct battery_status_s _batt_stat;
	_batt_stat_sub.copy(&_batt_stat);
	float batt_volt = _batt_stat.voltage_filtered_v;

	// Compose f_cmd as [f_T, Tau_x, Tau_y, Tau_z]'
	Matrix<float, 4, 1> f_cmd;
	f_cmd(0,0) = pos_control(0, 0);
	f_cmd(1,0) = att_control(0, 0);
	f_cmd(2,0) = att_control(1, 0);
	f_cmd(3,0) = att_control(2, 0);

	// Calculate and publish desired motor rotational speeds on scale of 0-1
	Matrix<float, 4, 1> motor_n;
	motor_n = C_inv*f_cmd;
	for (int i = 0; i<4; i++)
	{
		if (motor_n(i, 0) < 0) motor_n(i, 0) = 0;
		_motor_cmd.control[i] = (float)sqrt(motor_n(i, 0))/batt_volt - 1.0f;

		if (PX4_ISFINITE(_motor_cmd.control[i]))
		{
			_motor_cmd.control[i] = math::constrain(_motor_cmd.control[i], -1.0f, 1.0f);
		}
		else
		{
			_motor_cmd.control[i] = -1.0f;
		}
	}

	// Publish actuator throttles
	_actuator_pub.publish(_motor_cmd);

	if(_debug_){
		for (int i = 0; i<4; i++){
			_debug_file.body_forces[i] = f_cmd(i,0);
		}
		_debug_file.timestamp = hrt_absolute_time();
		_debug_file.batt_volt = batt_volt;
		_debug_pub.publish(_debug_file);
	}
}

void lqr_controller::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();

		float c_T = _param_c_T.get();
		float c_tau = _param_c_tau.get();
		float M_arm = 0.1355;
		float C_inv_coefs[16] = {1/(4*c_T), 1/(4*c_T*M_arm), 1/(4*c_T*M_arm), -1/(4*c_tau),
					1/(4*c_T), -1/(4*c_T*M_arm), 1/(4*c_T*M_arm), 1/(4*c_tau),
					1/(4*c_T), 1/(4*c_T*M_arm), -1/(4*c_T*M_arm), 1/(4*c_tau),
					1/(4*c_T), -1/(4*c_T*M_arm), -1/(4*c_T*M_arm), -1/(4*c_tau)};
		Matrix<float, 4, 4> temp(C_inv_coefs);
		C_inv = temp;

		z_int_init = -1.1f*(_param_veh_wt.get()/1000.0f)*9.81f/_param_K_pos_3.get();

		// Update K_pos
		K_pos(0,2) = _param_K_pos_1.get();
		K_pos(0,5) = _param_K_pos_2.get();
		K_pos(0,8) = _param_K_pos_3.get();
		K_pos(1,1) = _param_K_pos_4.get();
		K_pos(1,4) = _param_K_pos_5.get();
		K_pos(1,7) = _param_K_pos_6.get();
		K_pos(2,0) = _param_K_pos_7.get();
		K_pos(2,3) = _param_K_pos_8.get();
		K_pos(2,6) = _param_K_pos_9.get();

		// Update K_att
		K_att(0,0) = _param_K_att_1.get();
		K_att(0,3) = _param_K_att_2.get();
		K_att(0,6) = _param_K_att_3.get();
		K_att(1,1) = _param_K_att_4.get();
		K_att(1,4) = _param_K_att_5.get();
		K_att(1,7) = _param_K_att_6.get();
		K_att(2,2) = _param_K_att_7.get();
		K_att(2,5) = _param_K_att_8.get();
		K_att(2,8) = _param_K_att_9.get();

		_debug_ = _param_debug.get();
	}
}

void lqr_controller::vehicle_local_position_setpoint_poll()
{
	if (_pos_sp_sub.updated()) {
		_pos_sp_sub.copy(&_pos_sp);
	}
}

int lqr_controller::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module implements a PI controller designed using an LQR strategy

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("lqr_controller", "contoller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int lqr_controller::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int lqr_controller_main(int argc, char *argv[])
{
	return lqr_controller::main(argc, argv);
}
