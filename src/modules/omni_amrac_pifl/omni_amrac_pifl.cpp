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
 * 	Omnicopter controller
 *
 *	// Put Info here//
 *
 * 	Author: Patrick Thomas
 * 	Date: 10/28/2021
 *
 * ************************************************************************/

#include "omni_amrac_pifl.hpp"

omni_controller::omni_controller():
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	parameters_update(true);
}


omni_controller::~omni_controller()
{
	perf_free(_loop_perf);
}


int omni_controller::print_status()
{
	PX4_INFO("Running");

	return 0;
}

int omni_controller::task_spawn(int argc, char *argv[])
{

	omni_controller *instance = new omni_controller();

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

bool omni_controller::init(){
	ScheduleOnInterval(5000_us);
	Matrix<float, 6, 8> _B(_B_coefs);
	geninv(_B, _B_pinv);
	Bref.setZero();
	for(int i = 0; i<3; i++){
		Bref(6+i,i) = -1;
	}
	return true;
}

void omni_controller::Run(){

	if(should_exit()){
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have been updated
	parameters_update();

	actuator_armed_s arm_check;
	_armed_sub.copy(&arm_check);

	if(arm_check.armed && fly_veh){
		// Main Control functions here
		bool update_actuators = false;
		bool w_cmd_updated = false;
		// Update Position and Attitude Control at ~50 Hz
		if(_pos_sub.updated() && _att_sub.updated()){
			_dt = hrt_absolute_time() - _t_old;
			_t_old += _dt;
			if(_dt > 25000){
				PX4_WARN("dT high: %f", (double)(_dt/1000000.0f));
				_dt = 25000;
			}
			pos_setpoint_poll();
			att_setpoint_poll();
			update_pos_and_att();
			update_position_control();
			update_attitude_control();
			update_actuators = true;
			w_cmd_updated = true;
		}
		// Update Angular rate control at ~200 Hz
		if(_att_rate_sub.updated() || w_cmd_updated){
			update_angular_rate_control();
			update_actuators = true;
		}
		// Update Motor commands at ~200Hz
		if(update_actuators){
			update_motor_throttle();
		}
	}
	else if(!arm_check.armed && fly_veh){
		_pos_error_int = {0, 0, 0};
		_pos_error_der = {0, 0, 0};
		_q_error_int = {0, 0, 0, 0};

		loop_count = 0;
		fly_veh = false;
	}
	else if(arm_check.armed && !fly_veh){
		// Let motors 'warm up' for ~1 second
		if(loop_count < 10){
			_motor_cmd.control[0] = 0.1;
			_motor_cmd.control[1] = -0.1;
			_motor_cmd.control[2] = -0.1;
			_motor_cmd.control[3] = 0.1;
			_motor_cmd.control[4] = 0.1;
			_motor_cmd.control[5] = -0.1;
			_motor_cmd.control[6] = -0.1;
			_motor_cmd.control[7] = 0.1;// Warmup motors for vertical takeoff
			_actuator_pub.publish(_motor_cmd);
		}
		else if(loop_count > 250){
			fly_veh = true;
			pos_setpoint_poll(); // is this even published yet?
			xdbar.setZero();
			xdbar(2) = 0.2;
			xr = xd;
			xdbarmax = xd;
			thetahat.identity();
		}
		loop_count++;
	}

	perf_end(_loop_perf);
}

void omni_controller::update_position_control(){

	//input saturation
	xd = xdbarmax;

	float tau = 0.1;
	Vector<float,9> xdbardot = (1.0f/tau)*(xd-xdbar);

	xdbar += (_dt/1000000.0f)*xdbardot;

	Vector3f _pos_sp_bar(xdbar(0),xdbar(1),xdbar(2));
	Vector3f _vel_sp_bar(xdbar(3),xdbar(4),xdbar(5));

	//baseline controller
	Vector3f g_vec(0, 0, 9.81);
	Vector3f _pos_error = _pos_enu - _pos_sp_bar;
	_pos_error_int += _dt/2000000.0f*(_pos_error + _pos_error_old);
	_pos_error_der = _vel_enu - _vel_sp_bar;

	Vector3f _f_bl = -m_omni*(K_pos(0)*_pos_error + K_pos(1)*_pos_error_int + K_pos(2)*_pos_error_der);

	//reference model
	Vector<float,9> xrdot = Aref*xr+Bref*_pos_sp_enu+B*Kpid.T()*xdbar;
	xr += _dt/1000000.0f*xrdot;

	// adaptive update law
	x(6) = _pos_error_int(0); x(7) = _pos_error_int(1); x(8) = _pos_error_int(2);
	Vector<float,9> ea = x-xr;

	Matrix3f thetaHatDot = GammaPID*_f_bl*ea.T()*Ppid*B;

		//Projection Operator
		Matrix3f I;
		I.identity();
		Matrix3f ytotal = I*_f_bl*ea.T()*Ppid*B;

		float Tmax = 50;
		float epsilon = 0.5;
		for(int i = 0; i<3; i++){
			Vector3f theta;
			theta(0) = thetahat(0,i);
			theta(1) = thetahat(1,i);
			theta(2) = thetahat(2,i);


			float Tmax2 = pow(Tmax,2);


			Matrix<float,3,1> y;
			y(0,0) = ytotal(0,i);
			y(1,0) = ytotal(1,i);
			y(2,0) = ytotal(2,i);

			float fval = ((1+epsilon)*theta.norm_squared() - Tmax2)/(epsilon*Tmax2);
			Matrix<float,3,1> gradf = (2.0f*(1+epsilon)*theta)/(epsilon*Tmax2); //Tmax2 or Tmax???

			Scalarf comp = y.T()*GammaPID*gradf;

			if (fval>0.0f && comp>0.0f)
			{
			Scalarf gradfGnorm2 = gradf.T()*GammaPID*gradf;
			Vector3f newVals = GammaPID*(y - ((gradf*gradf.T()*GammaPID*y*fval)/gradfGnorm2));

			thetaHatDot(0,i) =  newVals(0);
			thetaHatDot(1,i) =  newVals(1);
			thetaHatDot(2,i) =  newVals(2);
			}
		}

	thetahat += thetaHatDot*_dt/1000000.0f;

	Vector3f _f_ad = -thetahat.T()*_f_bl;

	Vector3f u = _f_bl + _f_ad + 1.4f*g_vec;

	// Constrain commanded force
	if( u.norm() > 100.0f){
		u = (u/u.norm())*100.0f;
		Matrix<float,9,3> _Pinv;
		Matrix<float,3,9> Mat;
		Mat = Kpid.T()-(thetahat.T()*Kpid.T());
		Matrix3f inMat = Mat*Mat.T();
		_Pinv = Mat.T()*inv(inMat);
		xdbarmax = _Pinv*(u-(-Kpid.T()+thetahat.T()*Kpid.T())*x);
		if(xdbarmax(2) < 0.0f){ //floor constraint
			xdbarmax(2) = 0.0;
		}
	}
	else{
		for(int i=0;i<3;i++){
			xdbarmax(i) = _pos_sp_enu(i);
			xdbarmax(i+3) = _vel_sp_enu(i);
			xdbarmax(i+6) = 0;
		}
	}

	_f_cmd = R_q.T()*u;

	if(_debug){	// Position Control loop logging
		for (int i = 0; i <3; i++){
			_debug_file.pos[i] = _pos_enu(i);
			_debug_file.vel[i] = _vel_enu(i);
			_debug_file.pos_sp[i] = _pos_sp_enu(i);
			_debug_file.vel_sp[i] = _vel_sp_enu(i);
			_debug_file.pos_error[i] = _pos_error(i);
			_debug_file.pos_error_int[i] = _pos_error_int(i);
			_debug_file.pos_error_der[i] = _pos_error_der(i);

			_debug_file.x[i] = x(i);
			_debug_file.x[i+3] = x(i+3);
			_debug_file.x[i+6] = x(i+6);

			_debug_file.xr[i] = xr(i);
			_debug_file.xr[i+3] = xr(i+3);
			_debug_file.xr[i+6] = xr(i+6);

			_debug_file.xd[i] = xd(i);
			_debug_file.xd[i+3] = xd(i+3);
			_debug_file.xd[i+6] = xd(i+6);

			_debug_file.xdbar[i] = xdbar(i);
			_debug_file.xdbar[i+3] = xdbar(i+3);
			_debug_file.xdbar[i+6] = xdbar(i+6);

			_debug_file.theta_pid_one[i] = thetahat(i,0);
			_debug_file.theta_pid_two[i] = thetahat(i,1);
			_debug_file.theta_pid_three[i] = thetahat(i,2);
			_debug_file.fbl[i] = _f_bl(i);
			_debug_file.fad[i] = _f_ad(i);
			_debug_file.u[i] = u(i);
		}
		_debug_file.dt = _dt;
	}

	_pos_error_old = _pos_error;
}

void omni_controller::update_attitude_control(){

	Quatf _attq_enu_inv(_attq_enu(0), -_attq_enu(1), -_attq_enu(2), -_attq_enu(3));
	Quatf _q_error = _attq_sp_enu*_attq_enu_inv;
	_q_error = _q_error.normalized();
	_q_error_int += _dt/2000000.0f*(_q_error + _q_error_old);
	_q_error_int.normalized();
	int sign_q_0 = (_q_error(0) > 0) - (_q_error(0) < 0);
	_w_cmd = R_q.T()*(K_att(0)*sign_q_0*Vector3f(_q_error(1), _q_error(2), _q_error(3)) + K_att(1)*sign_q_0*Vector3f(_q_error_int(1), _q_error_int(2), _q_error_int(3)))
	 + Dcmf(_q_error)*_att_rate_sp_enu;

	if(_debug){
		for (int i = 0; i < 4; i++){
			_debug_file.attq[i] = _attq_enu(i);
			_debug_file.attq_sp[i] = _attq_sp_enu(i);
			_debug_file.q_error[i] = _q_error(i);
			_debug_file.q_error_int[i] = _q_error_int(i);
		}
	}

	_q_error_old = _q_error;
}

void omni_controller::update_angular_rate_control(){
	_att_rate_sub.copy(&_att_rate);
	Vector3f _w_enu(_att_rate.xyz[1], _att_rate.xyz[0], -_att_rate.xyz[2]);
	_t_cmd = K_tau_w*_J*(_w_cmd - _w_enu) + _w_enu.cross(_J*_w_enu);

	if (_debug){
		for (int i = 0; i < 3; i++){
			_debug_file.omega_cmd[i] = _w_cmd(i);
			_debug_file.omega[i] = _w_enu(i);
			_debug_file.omega_sp[i] = _att_rate_sp_enu(i);
		}
	}

}

void omni_controller::update_motor_throttle(){

	Vector<float, 6> _nu;

	_batt_stat_sub.copy(&_batt_stats);
	float _battery_voltage = _batt_stats.voltage_filtered_v;

	float _f_rot_max = Tau_f*(powf(throt_max*_battery_voltage, 2.0f));
	float _f_rot_min = Tau_f*(powf(throt_min*_battery_voltage, 2.0f));

	for (int i = 0; i < 3; i++){
		_nu(i) = _f_cmd(i);
		_nu(i+3) = _t_cmd(i);
	}

	_f_rot_cmd = _B_pinv*_nu;

	float _f_eval = _f_rot_max - 2*_f_rot_min;
	float _f_rot_cmd_max = (_f_rot_cmd.abs()).max();
	if (_f_rot_cmd_max > _f_eval){
		_nu*=_f_eval/_f_rot_cmd_max;
		_f_rot_cmd = _B_pinv*_nu;
	}

	// Add motor optimization with phi here

	for (int i = 0; i < 8; i++){
		if (PX4_ISFINITE(_f_rot_cmd(i))){
			_motor_cmd.control[i] = ((_f_rot_cmd(i) > 0) - (_f_rot_cmd(i) < 0)) * ( (float)sqrt( fabsf( _f_rot_cmd(i) ) / Tau_f ) )/((float)_battery_voltage);
		}
		else{
			_motor_cmd.control[i] = 0.0f;
		}
	}

	if (_debug){
		for (int i = 0; i < 8; i++){
			_debug_file.f_rot_cmd[i] = _f_rot_cmd(i);
		}
		for (int i = 0; i < 3; i++){
			_debug_file.f_cmd[i] = _nu(i);
			_debug_file.t_cmd[i] = _nu(i+3);
		}
		_debug_file.batt_volt = _battery_voltage;
		// Debug data is only published at every motor update
		_debug_file.timestamp = hrt_absolute_time();
		_debug_pub.publish(_debug_file);
	}

	_actuator_pub.publish(_motor_cmd);
}

void omni_controller::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();

		//Position Control
		K_pos(0) = 1/powf(_param_pos_tau.get(), 2.0f) + 2*_param_pos_eta.get()/(_param_pos_tau.get()*_param_pos_tau_i.get());
		K_pos(1) = 1/(powf(_param_pos_tau.get(), 2)*_param_pos_tau_i.get());
		K_pos(2) = 2*_param_pos_eta.get()/_param_pos_tau.get() + 1/_param_pos_tau_i.get();
		m_omni = _param_omni_m.get();

		Aref.setZero();
		B.setZero();
		GammaPID.setZero();
		Ppid.setZero();

		for(int i = 0; i<3; i++){
			Kpid(i,i) = m_omni*K_pos(0);
			Kpid(i+3,i) = m_omni*K_pos(2);
			Kpid(i+6,i) = m_omni*K_pos(1);

			GammaPID(i,i) = _param_gamma.get();

			B(i+3,i) = 1/m_omni;

			Aref(i,i+3) = 1;
			Aref(i+6,i) = 1;

			Ppid(i,i) = _param_p1.get();
			Ppid(i+3,i+3) = _param_p2.get();
			Ppid(i+6,i+6) = _param_p3.get();
			Ppid(i,i+3) = _param_p4.get();
			Ppid(i+3,i) = _param_p4.get();
			Ppid(i+3,i+6) = _param_p5.get();
			Ppid(i+6,i+3) = _param_p5.get();
			Ppid(i,i+6) = _param_p6.get();
			Ppid(i+6,i) = _param_p6.get();
		}
		Aref -= B*Kpid.T();


		//Attitude Control
		K_att(0) = 2/_param_att_tau.get() + 2/_param_att_tau_i.get();
		K_att(1) = 2/(_param_att_tau.get()*_param_att_tau_i.get());

		K_tau_w = 1/_param_att_tau_w.get();

		_J(0,0) = _param_J_xx.get();
		_J(0,1) = 0;
		_J(0,2) = 0;
		_J(1,0) = 0;
		_J(1,1) = _param_J_yy.get();
		_J(1,2) = 0;
		_J(2,0) = 0;
		_J(2,1) = 0;
		_J(2,2) = _param_J_zz.get();

		Tau_f = _param_tau_f.get();
		eps = _param_ctrl_eps.get();
		throt_max = _param_throt_max.get();
		throt_min = _param_throt_min.get();
		tau_hyst = _param_tau_hyst.get();

		_debug = _param_debug_bool.get();

	}
}

void omni_controller::update_pos_and_att(){
	_pos_sub.copy(&_pos);
	_att_sub.copy(&_att);
	// Convert from NED to ENU
	_pos_enu = Vector3f(_pos.y, _pos.x, -_pos.z);
	_vel_enu = Vector3f(_pos.vy, _pos.vx, -_pos.vz);

	for(int i=0;i<3;i++){
			x(i) = _pos_enu(i);
			x(i+3) = _vel_enu(i);
		}

	_attq_enu = Quatf(_att.q[0], _att.q[2], _att.q[1], -_att.q[3]);
	R_q = Dcmf(_attq_enu);
}

void omni_controller::pos_setpoint_poll()
{
	if (_pos_sp_sub.updated()) {
		_pos_sp_sub.copy(&_pos_sp);
		// Convert Position setpoint NED to ENU
		_pos_sp_enu = Vector3f(_pos_sp.y, _pos_sp.x, -_pos_sp.z);	// Due to Mavlink nature, position is passed in through velocity setpoint
		_vel_sp_enu = Vector3f(_pos_sp.vy, _pos_sp.vx, -_pos_sp.vz);
		for(int i=0;i<3;i++){
			xd(i) = _pos_sp_enu(i);
			xd(i+3) = _vel_sp_enu(i);
			xd(i+6) = 0;
		}

	}
}

void omni_controller::att_setpoint_poll()
{
	if (_att_sp_sub.updated()) {
		_att_sp_sub.copy(&_att_sp);
		// Convert Attitude Setpoint from NED to ENU
		Quatf attq_sp_raw(_att_sp.q_d[0], _att_sp.q_d[1], _att_sp.q_d[2], _att_sp.q_d[3]);
		Quatf ENUinNED(-0.7071068, 0, 0, 0.7071068);
		Quatf attq_sp_temp = attq_sp_raw * ENUinNED;
		_attq_sp_enu = Quatf(attq_sp_temp(0), attq_sp_temp(2), attq_sp_temp(1), -attq_sp_temp(3));
	}
	if (_att_rate_sp_sub.updated()){
		_att_rate_sp_sub.copy(&_att_rate_sp);
		// Convert NED to ENU
		_att_rate_sp_enu = Vector3f(_att_rate_sp.roll, -_att_rate_sp.pitch, -_att_rate_sp.yaw);
	}
}

int omni_controller::print_usage(const char *reason)
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

	PRINT_MODULE_USAGE_NAME("omni_amrac_pifl", "contoller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int omni_controller::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int omni_amrac_pifl_main(int argc, char *argv[])
{
	return omni_controller::main(argc, argv);
}
