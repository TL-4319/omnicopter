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

#include "omnicopter_controller.hpp"

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
		_pos_error_int.setZero();
		_pos_error_der.setZero();
		_pos_error_der_old.setZero();
		_pos_error_der_filt.setZero();
		_q_error_int.setZero();
		_phi_1 = 0;
		_phi_2 = 0;
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
		else if(loop_count > 350){
			uint64_t current_time = hrt_absolute_time() - 1000000;	// set current time to be 1 second previous
			for (int i = 0; i < 4; i++){
				_motor_tau_hyst_1(i) = current_time;
				_motor_tau_hyst_2(i) = current_time;
			}
			_f_rot_cmd_old(0) = 0.1;
			_f_rot_cmd_old(1) = -0.1;
			_f_rot_cmd_old(2) = -0.1;
			_f_rot_cmd_old(3) = 0.1;
			_f_rot_cmd_old(4) = 0.1;
			_f_rot_cmd_old(5) = -0.1;
			_f_rot_cmd_old(6) = -0.1;
			_f_rot_cmd_old(7) = 0.1;
			_f_rot_filt = _f_rot_cmd_old;
			fly_veh = true;
		}
		loop_count++;
	}

	perf_end(_loop_perf);
}

void omni_controller::update_position_control(){
	float b1 = 0.2452;
	float b2 = 0.2452;
	float a1 = 0.5095;

	Vector3f g_vec(0, 0, 9.81);
	Vector3f _pos_error = _pos_sp_enu - _pos_enu;
	// Calculate position error integral and derivative using 1st order DT approximation
	_pos_error_int += _dt/2000000.0f*(_pos_error + _pos_error_old);
	_pos_error_der = 1000000.0f/_dt*(_pos_error - _pos_error_old); // - _pos_error_der;

	_pos_error_der_filt = a1*_pos_error_der_filt + b1*_pos_error_der + b2*_pos_error_der_old;
	_pos_error_der_old = _pos_error_der;

	_f_cmd = m_omni*R_q.T()*(K_pos(0)*_pos_error + K_pos(1)*_pos_error_int + K_pos(2)*_pos_error_der_filt + _acc_sp_enu + g_vec);

	if(_debug){	// Position Control loop logging
		for (int i = 0; i <3; i++){
			_debug_file.pos[i] = _pos_enu(i);
			_debug_file.pos_sp[i] = _pos_sp_enu(i);
			_debug_file.pos_error[i] = _pos_error(i);
			_debug_file.pos_error_int[i] = _pos_error_int(i);
			_debug_file.pos_error_der[i] = _pos_error_der_filt(i);
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
	float a1 = 0.02703;
	float a2 = 0.02703;
	float b1 = 0.9459;

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
	/*
	// Add motor optimization with phi here

	Vector<float, 4> _f_hat_1;
	Vector<float, 4> _f_hat_2;

	for (int i = 0; i < 4; i++){
		_f_hat_1(i) = _f_rot_cmd(i);
		_f_hat_2(i) = _f_rot_cmd(i+4);

		_f_rot_1(i) = _f_rot_filt(i);
		_f_rot_2(i) = _f_rot_filt(i+4);
	}

	// // Debuging
	// PX4_INFO("Phi 1:");
	// //
	_phi_1 = phi_optimizer(_f_hat_1, _f_rot_1, _phi_1, _f_rot_max, _f_rot_min, _motor_tau_hyst_1);
	// // Debuging
	// PX4_INFO("Phi 2:");
	// //
	_phi_2 = phi_optimizer(_f_hat_2, _f_rot_2, _phi_2, _f_rot_max, _f_rot_min, _motor_tau_hyst_2);

	float eta_1_coef[8] = {1, 1, 1, 1, 0, 0, 0, 0};
	Vector<float, 8> _eta_1(eta_1_coef);
	float eta_2_coef[8] = {0, 0, 0, 0, 1, 1, 1, 1};
	Vector<float, 8> _eta_2(eta_2_coef);

	_f_rot_cmd += _phi_1*_eta_1 + _phi_2*_eta_2;
	*/

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
			_debug_file.f_rot_filt[i] = _f_rot_filt(i);
		}
		for (int i = 0; i < 3; i++){
			_debug_file.f_cmd[i] = _nu(i);
			_debug_file.t_cmd[i] = _nu(i+3);
		}
		_debug_file.batt_volt = _battery_voltage;
		_debug_file.phi[0] = _phi_1;
		_debug_file.phi[1] = _phi_2;
		_debug_file.f_rot_lim[0] = _f_rot_min;
		_debug_file.f_rot_lim[1] = _f_rot_max;
		// Debug data is only published at every motor update
		_debug_file.timestamp = hrt_absolute_time();
		_debug_pub.publish(_debug_file);
	}

	_actuator_pub.publish(_motor_cmd);

	// Update actual rotor force;
	_f_rot_filt = b1*_f_rot_filt + a1*_f_rot_cmd + a2*_f_rot_cmd_old;
	_f_rot_cmd_old = _f_rot_cmd;
}

float omni_controller::phi_optimizer(const Vector<float, 4> &fhat, const Vector<float, 4> &f_old, const float &phi, const float &f_max, const float &f_min, Vector<float, 4> &motor_tau)
{
	// Find optimum phi
	//float phi_temp = newt_raph(fhat, f_old, phi);
	float phi_temp = bisection(fhat, f_old, phi);
	// // Debuging
	//PX4_INFO("Optim: %f", (double)phi_temp);
	// //

	// Compute set Phi_hyst
	Vector<float, 4> phi_min_lim;
	Vector<float, 4> phi_max_lim;
	uint64_t current_time = hrt_absolute_time();
	Vector<float, 4> motor_d_tau;
	for (int i = 0; i<4; i++){
		motor_d_tau(i) = (current_time - motor_tau(i))/1000000.0f;
	}
	Vector<float, 4> motor_d_tau_sorted = motor_d_tau;
	Vector<float, 4> exclusion_min;
	Vector<float, 4> exclusion_max;
	float tau_lim = tau_hyst;
	int tau_count = 0;

	// Sort motor tau in order of descending times since motor direction flip and
	for(int i = 0; i<4; i++){
		for (int j = 0; j<i; j++){
			if(motor_d_tau_sorted(i) > motor_d_tau_sorted(j)){
				float temp = motor_d_tau_sorted(j);
				motor_d_tau_sorted(j) = motor_d_tau_sorted(i);
				motor_d_tau_sorted(i) = temp;
			}
			if(i==3 && motor_d_tau_sorted(j) > tau_lim){
				tau_count++;
			}
		}
	}

	float phi_min = 1;
	float phi_max = -1;
	while(phi_min > phi_max && tau_count < 5){
		for(int i = 0; i<4; i++){
			if(motor_d_tau(i) >= tau_lim){
				phi_min_lim(i) = -1.0f*(f_max + fhat(i));
				phi_max_lim(i) = f_max - fhat(i);
			}
			else if (f_old(i) > 0){ // if the previous commanded force was positive
				phi_min_lim(i) = f_min - fhat(i);
				phi_max_lim(i) = f_max - fhat(i);
			}
			else{
				phi_min_lim(i) = -1.0f*(f_max + fhat(i));
				phi_max_lim(i) = -1.0f*(f_min + fhat(i));
			}
			exclusion_min(i) = -1.0f*(f_min + fhat(i));
		}
		phi_min = phi_min_lim.max();
		phi_max = phi_max_lim.min();
		if(tau_count < 5){
			tau_lim = motor_d_tau_sorted(tau_count);
			tau_count++;
		}
	}

	// // Debuging
	// PX4_INFO("min:%f max:%f", (double)phi_min, (double)phi_max);
	// PX4_INFO("Ex: %f, %f, %f, %f", (double)exclusion_min(0), (double)exclusion_min(1), (double)exclusion_min(2), (double)exclusion_min(3));
	// //

	// Sort exclusion min in ascending order
	for(int i = 0; i<4; i++){
		for (int j = 0; j<i; j++){
			if(exclusion_min(i) < exclusion_min(j)){
				float temp = exclusion_min(j);
				exclusion_min(j) = exclusion_min(i);
				exclusion_min(i) = temp;
			}
		}
	}

	bool temp[4] = {0, 0, 0, 0};
	Vector<bool, 4> valid_exclusion(temp);

	// Create set of valid exclusion zone(s) that lie within phi min and phi max
	int j = 0;
	for(int i = 0; i<4; i++){
		if(exclusion_min(i) > phi_min){
			if(exclusion_min(i) + 2*f_min < phi_max){
				valid_exclusion(i) = true;
				exclusion_max(i) = exclusion_min(i) + 2*f_min;
				// Overlapping exclusions
				if(i < 3 && exclusion_min(i+1) < exclusion_max(i)){
					j = i;
					exclusion_max(j) = exclusion_min(i+1) + 2*f_min;
					i++;
					if (i < 3 && exclusion_min(i+1) < exclusion_max(j)){
						exclusion_max(j) = exclusion_min(i+1) + 2*f_min;
						i++;
						if (i < 3 && exclusion_min(i+1) < exclusion_max(j)){
							exclusion_max(j) = exclusion_min(i+1) + 2*f_min;
							i++;
						}
					}
					// if combined exclusions zone max is bigger than phi_max, lower phi_max and remove exclusion zone
					if(exclusion_max(j) > phi_max){
						phi_max = exclusion_min(j);
						valid_exclusion(i) = false;
					}
				}
			}
			else if (exclusion_min(i) < phi_max){
				phi_max = exclusion_min(i);
				break;
			}
			else{
				break;
			}
		}
		else if (exclusion_min(i) + 2*f_min > phi_min){
			phi_min = exclusion_min(i)+2*f_min;
		}
	}

	// Check if phi lies in the set Phi_hyst, if not place phi at closest boundary
	if (phi > phi_min){
		for(int i = 0; i<4; i++){
			if(valid_exclusion(i)){
				if(phi < exclusion_min(i)){
					break;
				}
				else if(phi < exclusion_max(i)){
					if(phi > (exclusion_max(i) - exclusion_min(i))/2){
						phi_temp = exclusion_max(i);
					}
					else{
						phi_temp = exclusion_min(i);
					}
					break;
				}
			}
		}
		if(phi > phi_max){
			phi_temp = phi_max;
		}
	}
	else{
		phi_temp = phi_min;
	}

	// Check if motor direction has changed and if so, update timestamp of motor diretion change.
	// Vector<float, 4> fcmd = fhat + phi;
	float fcmd;
	for(int i = 0; i<4; i++){
		fcmd = fhat(i) + phi;
		if ( (fcmd > 0)^(f_old(i) > 0) ){
			motor_tau(i) = current_time;
		}
	}
	// return phi
	return phi_temp;
}

float omni_controller::bisection(const Vector<float,4> &f_hat, const Vector<float, 4> &f_old, const float &phi)
{
	float phi_temp = phi;
	float low = phi_temp - 1.5f;
	float high = phi_temp + 1.5f;
	float J_phi;
	uint8_t bisect_loop_count = 0;
	float err = high - low;

	while(err > 0.01f && bisect_loop_count < 10){
		J_phi = objective_func(phi_temp, f_hat, f_old);
		if (fabs(J_phi) < 0.01){
			break;
		}
		else if (J_phi < 0){
			low = phi_temp;
			phi_temp = (low+high)/2;
		}
		else if (J_phi > 0){
			high = phi_temp;
			phi_temp = (low+high)/2;
		}
		err = high - low;
		bisect_loop_count++;
	}

	return phi_temp;
}

float omni_controller::newt_raph(const Vector<float, 4>&fhat, const Vector<float, 4> &f_old, const float &phi)
{
	int nr_loop_count = 0;
	float phi_temp = phi;
	float err = -1.0f*objective_func(phi_temp, fhat, f_old)/d_objective_func(phi_temp, fhat, f_old);
	phi_temp += err;
	while(fabsf(err) > 0.01f && nr_loop_count < 5){
		err = -1.0f*objective_func(phi_temp, fhat, f_old)/d_objective_func(phi_temp, fhat, f_old);
		phi_temp += err;
		nr_loop_count++;
	}
	return phi_temp;
}

float omni_controller::objective_func(const float &phi, const Vector<float, 4> &fhat, const Vector<float, 4> &frot)
{
	float J = 0.0f;

	for(size_t i=0; i<4; i++) {
		J += 1.5f * (1.0f - eps) * (fhat(i) + phi) / powf(fabsf(fhat(i) + phi), 0.5f) + 2.0f * eps * (fhat(i) + phi - frot(i));
	}

	return J;

}

float omni_controller::d_objective_func(const float &phi, const Vector<float, 4> &fhat, const Vector<float, 4> &frot)
{
	float J = 0.0f;

	for(size_t i=0; i<4; i++) {
		J += 0.75f * (1.0f - eps) * powf(fabsf(fhat(i) + phi), -0.5f) + 2.0f * eps;
	}

	return J;

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

		K_pos(0) = 1/powf(_param_pos_tau.get(), 2.0f) + 2*_param_pos_eta.get()/(_param_pos_tau.get()*_param_pos_tau_i.get());
		K_pos(1) = 1/(powf(_param_pos_tau.get(), 2)*_param_pos_tau_i.get());
		K_pos(2) = 2*_param_pos_eta.get()/_param_pos_tau.get() + 1/_param_pos_tau_i.get();
		m_omni = _param_omni_m.get();

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

	_attq_enu = Quatf(_att.q[0], _att.q[2], _att.q[1], -_att.q[3]);
	R_q = Dcmf(_attq_enu);
}

void omni_controller::pos_setpoint_poll()
{
	if (_pos_sp_sub.updated()) {
		_pos_sp_sub.copy(&_pos_sp);
		// Convert Position setpoint NED to ENU
		_pos_sp_enu = Vector3f(_pos_sp.y, _pos_sp.x, -_pos_sp.z);	// Due to Mavlink nature, position is passed in through velocity setpoint
		_acc_sp_enu = Vector3f(_pos_sp.acceleration[1], _pos_sp.acceleration[0], -_pos_sp.acceleration[2]);
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
		_att_rate_sp_enu = Vector3f(_att_rate_sp.pitch, _att_rate_sp.roll, -_att_rate_sp.yaw);
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

	PRINT_MODULE_USAGE_NAME("omni_controller", "contoller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int omni_controller::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int omnicopter_controller_main(int argc, char *argv[])
{
	return omni_controller::main(argc, argv);
}
