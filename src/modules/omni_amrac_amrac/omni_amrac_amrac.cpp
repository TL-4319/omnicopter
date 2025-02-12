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

#include "omni_amrac_amrac.hpp"

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
	ScheduleOnInterval(2500_us);

	// Compute B and B pseudoinverse based on omnicopter geometric parameters
	float a = 0.5f + powf(12.0f, -0.5f);
	float b = 0.5f - powf(12.0f, -0.5f);
	float c = powf(3.0f,-0.5f);
	_B.setZero();
	float N_coefs[24] = {-a, b, -b, a, a, -b, b, -a,
				b, c, -a, -b, -b, -a, a, b,
				c, -c, -c, c, c, -c, -c, c};
	float P_coefs[24] = {1, -1, 1, -1, 1, -1, 1, -1,
				1, 1, -1, -1, 1, 1, -1, -1,
				1, 1, 1, 1, -1, -1, -1, -1};
	Matrix<float, 3, 8> _N(N_coefs);
	Matrix<float, 3, 8> _P(P_coefs);
	_P = powf(3.0f,-0.5f)*_P;
	float Kappa = _param_kappa.get();
	float _K_diag_coefs[8] = {Kappa, Kappa, Kappa, Kappa, -Kappa, -Kappa, -Kappa, -Kappa};
	Vector<float, 8> _K_diag(_K_diag_coefs);
	Matrix<float, 8, 8> _K_temp;
	_K_temp = diag(_K_diag);
	_B.slice<3,8>(0,0) = _N;
	Matrix<float,3,8> _B_att_temp;
	Vector3f _P_slice;
	Vector3f _N_slice;
	for(int i = 0; i<8; i++){
		_P_slice = _P.slice<3,1>(0,i);
		_N_slice = _N.slice<3,1>(0,i);
		_B_att_temp.slice<3,1>(0,i) = _P_slice.cross(_N_slice);
	}
	_B_att_temp = _param_J_arm.get()*_B_att_temp + _N*_K_temp;
	_B.slice<3,8>(3,0) = _B_att_temp;
	geninv(_B, _B_pinv);

	Bref.setZero();
	xd.setZero();
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
		// Update Position and Attitude Control at ~200 Hz
		if(_pos_sub.updated() || _att_sub.updated()){
			_dt = hrt_absolute_time() - _t_old;
			_t_old += _dt;
			if(_dt > 10000){
				// PX4_WARN("dT high: %f", (double)(_dt/1000000.0f));
				_dt = 10000;
			}
			pos_setpoint_poll();
			att_setpoint_poll();
			update_pos_and_att();
			update_position_control();
			update_attitude_control();
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
		_phi_1 = 0;
		_phi_2 = 0;
		loop_count = 0;
		fly_veh = false;
	}
	else if(arm_check.armed && !fly_veh){
		// Let motors 'warm up' for ~1.5 second
		if(loop_count < 10){
			_motor_cmd.control[0] = 0.15;
			_motor_cmd.control[1] = -0.15;
			_motor_cmd.control[2] = -0.15;
			_motor_cmd.control[3] = 0.15;
			_motor_cmd.control[4] = 0.15;
			_motor_cmd.control[5] = -0.15;
			_motor_cmd.control[6] = -0.15;
			_motor_cmd.control[7] = 0.15;// Warmup motors for vertical takeoff
			_actuator_pub.publish(_motor_cmd);
		}
		else if(loop_count > 650){
			//IC for pos aMRAC
			pos_setpoint_poll(); // is this even published yet?
			xdbar.setZero();
			xdbar(2) = 0.2;
			xr.setZero();
			xdbarmax.setZero();
			thetahatPID.setIdentity();
			_pos_sat_corr = 1.0f;

			//IC for att aBack
			_w_des_dot_sp_enu = {0,0,0};
			thetaHatAtt = thetaHat0;
			lambdaHat = lambdaHat0;
			tdhat.setZero();
			wref.setZero();
			_att_sat_corr = 1.0f;

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

	//input saturation
	xd = xdbarmax;

	float tau = 0.01;
	Vector<float,9> xdbardot = (1.0f/tau)*(xd-xdbar);

	xdbar += (_dt/1000000.0f)*xdbardot;

	Vector3f _pos_sp_bar(xdbar(0),xdbar(1),xdbar(2));
	Vector3f _vel_sp_bar(xdbar(3),xdbar(4),xdbar(5));

	//baseline controller
	Vector3f g_vec(0, 0, 9.81);
	Vector3f _pos_error = _pos_sat_corr*(_pos_enu - _pos_sp_bar);
	_pos_error_int += _dt/2000000.0f*(_pos_error + _pos_error_old);
	_pos_error_der = _pos_sat_corr*(_vel_enu - _vel_sp_bar);

	Vector3f _f_bl = -m_omni*(K_pos(0)*_pos_error + K_pos(1)*_pos_error_int + K_pos(2)*_pos_error_der);

	//reference model
	Vector<float,9> xrdot = Aref*xr+Bref*_pos_sp_enu+B*Kpid.T()*xdbar;
	xr += _dt/1000000.0f*xrdot;

	// adaptive update law
	x(6) = _pos_error_int(0); x(7) = _pos_error_int(1); x(8) = _pos_error_int(2);
	Vector<float,9> ea = _pos_sat_corr*(x-xr);

	Matrix3f thetaHatDot = GammaPID*_f_bl*ea.T()*Ppid*B;

		//Projection Operator
		Matrix3f I;
		I.identity();
		Matrix3f ytotal = I*_f_bl*ea.T()*Ppid*B;

		float Tmax = 1.5;
		float epsilon = 0.5;
		for(int i = 0; i<3; i++){
			Vector3f theta;
			theta(0) = thetahatPID(0,i);
			theta(1) = thetahatPID(1,i);
			theta(2) = thetahatPID(2,i);


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

	thetahatPID += thetaHatDot*_dt/1000000.0f;

	Vector3f _f_ad = -thetahatPID.T()*_f_bl;

	Vector3f u = _f_bl + _f_ad;

	// Constrain commanded force
	if((u+1.4f*g_vec).norm() > F_max){
		int sat_loop_count = 0;
		float corr_factor = 0.9;
		while((u+1.4f*g_vec).norm() > F_max && sat_loop_count < 50){
			u*=corr_factor;
			sat_loop_count++;
		}

		_pos_sat_corr += _dt/(1000000.0f*tau)*(powf(corr_factor, sat_loop_count) - _pos_sat_corr);
		//u = (u/u.norm())*u_norm_max;
		// Change desired position and velocity setpoints s.t. computed
		// Matrix<float,6,3> _K_Pinv;
		// Matrix<float,3,6> _K_pd_mat;
		// Matrix3f _K_i_mat;
		// Vector3f x_bar_i;
		// Vector<float, 6> x_bar_pd;
		// Vector3f u_pos_vel;

		// _K_pd_mat = Kpid.T().slice<3,6>(0,0);
		// _K_pd_mat -= thetahatPID.T()*_K_pd_mat;
		// _K_i_mat = Kpid.T().slice<3,3>(0,6);
		// _K_i_mat -= thetahatPID.T()*_K_i_mat;
		// x_bar_pd = x.slice<6,1>(0,0);
		// x_bar_i = x.slice<3,1>(6,0);

		// geninv(-_K_pd_mat,_K_Pinv);
		// u_pos_vel = u + _K_i_mat*x_bar_i - 1.25f*g_vec;
		// xdbarmax.slice<6,1>(0,0) = x_bar_pd - _K_Pinv*u_pos_vel;

		// u = (u/u.norm())*F_max;
		// Matrix<float,9,3> _Pinv;
		// Matrix<float,3,9> Mat;
		// Mat = Kpid.T()-(thetahatPID.T()*Kpid.T());
		// Matrix3f inMat = Mat*Mat.T();
		// _Pinv = Mat.T()*inv(inMat);
		// xdbarmax = _Pinv*(u-(-Kpid.T()+thetahatPID.T()*Kpid.T())*x);
		// if(xdbarmax(2) < 0.0f){ //floor constraint
		// 	xdbarmax(2) = 0.0;
		// }
	}
	else{
		for(int i=0;i<3;i++){
			xdbarmax(i) = _pos_sp_enu(i);
			xdbarmax(i+3) = _vel_sp_enu(i);
			xdbarmax(i+6) = 0;
			_pos_sat_corr += _dt/(1000000.0f*tau)*(1.0f - _pos_sat_corr);
			if(_pos_sat_corr > 1.0f) _pos_sat_corr = 1.0f;
		}
	}
	u += 1.4f*g_vec;
	_f_cmd = u;

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

			_debug_file.theta_pid_one[i] = thetahatPID(i,0);
			_debug_file.theta_pid_two[i] = thetahatPID(i,1);
			_debug_file.theta_pid_three[i] = thetahatPID(i,2);
			_debug_file.fbl[i] = _f_bl(i);
			_debug_file.fad[i] = _f_ad(i);
			_debug_file.u[i] = u(i);
		}
		_debug_file.pos_sat_corr = _pos_sat_corr;
		//_debug_file.u_norm_max = u_norm_max;
		_debug_file.dt = _dt;
	}

	_pos_error_old = _pos_error;
}

void omni_controller::update_attitude_control(){
	Matrix3f I;
	I.setIdentity();

	Matrix3f P;
	P.setIdentity();

	float tau = 0.01;

	//read in angular velocity and compute error
	_att_rate_sub.copy(&_att_rate);
	Vector3f _w_enu(_att_rate.xyz[1], _att_rate.xyz[0], -_att_rate.xyz[2]);
	Vector3f phi(-_w_enu(1)*_w_enu(2), -_w_enu(0)*_w_enu(2), -_w_enu(0)*_w_enu(1));
	Vector3f wtilde = _att_sat_corr*(_w_enu - _att_rate_sp_enu); //check _w_sp is actually read in

	//skew symmetric matrix for attq_sp
	Matrix3f _attq_sp_cross;
	_attq_sp_cross.setZero();
	_attq_sp_cross(1,0) = -_attq_sp_enu(3);
	_attq_sp_cross(2,0) = _attq_sp_enu(2);
	_attq_sp_cross(2,1) = -_attq_sp_enu(1);
	Matrix<float,2,2> l_side = _attq_sp_cross.slice<2,2>(1,0);
	_attq_sp_cross.slice<2,2>(0,1) = -l_side.T();

	Matrix<float, 4, 4> Q;
	Vector3f neg_attq_vec = _attq_sp_enu.slice<3,1>(1,0);
	Q.slice<1,4>(0,0) = _attq_sp_enu;
	Q.slice<3,1>(1,0) = -neg_attq_vec;
	Q.slice<3,3>(1,1) = _attq_sp_enu(0)*I + _attq_sp_cross;

	//Compute quaternion error
	Quatf _q_error = Q*_attq_enu;
	_q_error = _q_error.normalized();

	//skew symmetric matrix for omega
	Matrix3f _w_cross;
	_w_cross.setZero();
	_w_cross(1,0) = _w_enu(2);
	_w_cross(2,0) = -_w_enu(1);
	_w_cross(2,1) = _w_enu(0);
	Matrix<float,2,2> left_side = _w_cross.slice<2,2>(1,0);
	_w_cross.slice<2,2>(0,1) = -left_side.T();

	//compute logv(qe)
	Vector3f logv;
	Vector3f qerr(_q_error(1), _q_error(2), _q_error(3));
	if(qerr.norm() <= 0.00001f){
		logv.setZero();
	}
	else{
		logv = _att_sat_corr*(atan2f(qerr.norm(),_q_error(0))*qerr/qerr.norm());
	}

	//Baseline Controller
	Vector<float,3> tbl;
	tbl = _w_cross*_J*_w_enu + _J*(_w_des_dot_sp_enu + _w_cross*_att_rate_sp_enu - kq*logv - kw*wtilde );

	//Reference Model
	Vector3f ew = _att_sat_corr*(wref - _w_enu);
	Vector<float,6> force_torque_est = _B*_f_rot_filt;
	Vector3f tcest(force_torque_est(3),force_torque_est(4),force_torque_est(5));
	Vector3f wrefdot = -ke*ew + thetaHatAtt.T()*phi + lambdaHat.T()*tcest + tdhat;
	wref += wrefdot*_dt/1000000.0f;

	//Compute Adaptive Parameters
	//Compute thetaHatDot
	Matrix3f thetaHatDot = -Gtheta*phi*ew.T()*P;
	proj(thetaHatDot, thetaHatAtt, I*phi*ew.T()*P, Gtheta, Tmaxtheta);
	//Compute lambdaHatDot
	Matrix3f lambdaHatDot = -Glambda*tcest*ew.T()*P;
	proj(lambdaHatDot, lambdaHat, I*tcest*ew.T()*P, Glambda, Tmaxlambda);

	//Compute tdhat
	Vector3f tdHatDot = -Gtau*(P*ew);
		//Projection for tdhat (different from the previous two b/c is a vector not matrix)
		Vector3f y = P*ew;

		float Tmax = Tmaxtd;
		float epsilon = 0.5;
		float Tmax2 = pow(Tmax,2);

		float fval = ((1+epsilon)*tdhat.norm_squared() - Tmax2)/(epsilon*Tmax2);
		Matrix<float,3,1> gradf = (2.0f*(1+epsilon)*tdhat)/(epsilon*Tmax2); //Tmax2 or Tmax???

		Scalarf comp = y.T()*Gtau*gradf;

		if (fval>0.0f && comp<0.0f)
		{
		Scalarf gradfGnorm2 = gradf.T()*Gtheta*gradf;
		Vector3f newVals = -Gtau*(y - ((gradf*gradf.T()*Gtau*y*fval)/gradfGnorm2));

		tdHatDot = newVals;
		}

		thetaHatAtt += thetaHatDot*_dt/1000000.0f;
		lambdaHat += lambdaHatDot*_dt/1000000.0f;
		tdhat += tdHatDot*_dt/1000000.0f;

	//compute adaptive torque
	Matrix3f Ltrans = lambdaHat.T();
	Vector3f _t_ad = inv(Ltrans)*(-(thetaHatAtt.T()-thetaHat0)*phi-tdhat) - (I-inv(Ltrans)*lambdaHat0)*tbl;

	//compute total torque
	_t_cmd = _t_ad + tbl;

	// Project _t_cmd into set of achievable torques
	if(_t_cmd.norm() > T_max){
		_att_sat_corr += _dt/(1000000.0f*tau)*(0.9f*T_max/_t_cmd.norm() - _att_sat_corr);
		_t_cmd = (_t_cmd/_t_cmd.norm())*T_max;
	}
	else{
		_att_sat_corr += _dt/(1000000.0f*tau)*(1.0f - _att_sat_corr);
		if(_att_sat_corr > 1.0f) _att_sat_corr = 1.0f;
	}

	//save data for debugging
	if(_debug){
		for (int i = 0; i < 4; i++){
			_debug_file.attq[i] = _attq_enu(i);
			_debug_file.attq_sp[i] = _attq_sp_enu(i);
			_debug_file.q_error[i] = _q_error(i);

		}
	}

	if(_debug){
		for (int i = 0; i < 3; i++){
			_debug_file.omega[i] = _w_enu(i);
			_debug_file.omega_sp[i] = _att_rate_sp_enu(i);
			_debug_file.omega_dot_sp[i] = _w_des_dot_sp_enu(i);
			_debug_file.theta_att_one[i] = thetaHatAtt(i,0);
			_debug_file.theta_att_two[i] = thetaHatAtt(i,1);
			_debug_file.theta_att_three[i] = thetaHatAtt(i,2);
			_debug_file.lambdahat_one[i] = lambdaHat(i,0);
			_debug_file.lambdahat_two[i] = lambdaHat(i,1);
			_debug_file.lambdahat_three[i] = lambdaHat(i,2);
			_debug_file.tdhat[i] = tdhat(i);
			_debug_file.tbl[i] = tbl(i);
			_debug_file.tad[i] = _t_ad(i);
			_debug_file.tcest[i] = tcest(i);
			_debug_file.wref[i] = wref(i);
			_debug_file.logv[i] = logv(i);
		}
		_debug_file.att_sat_corr = _att_sat_corr;
	}


}

void omni_controller::proj(Matrix3f &thetaDot, const Matrix3f &thetaHat, const Matrix3f &ytotal, const Matrix3f &G, const float &Tmax){
	float epsilon = 0.5;
	for(int i = 0; i<3; i++){
		Vector3f theta = thetaHat.slice<3,1>(0,i);

		float Tmax2 = pow(Tmax,2);

		Matrix<float,3,1> y = ytotal.slice<3,1>(0,i);

		float fval = ((1+epsilon)*theta.norm_squared() - Tmax2)/(epsilon*Tmax2);
		Matrix<float,3,1> gradf = (2.0f*(1+epsilon)*theta)/(epsilon*Tmax2); //Tmax2 or Tmax???

		Scalarf comp = y.T()*G*gradf;

		if (fval>0.0f && comp<0.0f)
		{
		Scalarf gradfGnorm2 = gradf.T()*G*gradf;
		Vector3f newVals = -G*(y - ((gradf*gradf.T()*G*y*fval)/gradfGnorm2));

		thetaDot.slice<3,1>(0,i) = newVals;
		}
	}

}

void omni_controller::update_motor_throttle(){

	Vector<float, 6> _nu;
	float a1 = _dt/(2*tau_mot + _dt);
	float a2 = a1;
	float b1 = (2*tau_mot - _dt)/(2*tau_mot + _dt);

	_batt_stat_sub.copy(&_batt_stats);
	float _battery_voltage = _batt_stats.voltage_filtered_v;

	float _f_rot_max = Tau_f*(powf(throt_max*_battery_voltage, 2.0f));
	float _f_rot_min = Tau_f*(powf(throt_min*_battery_voltage, 2.0f));

	for (int i = 0; i < 3; i++){
		_nu(i) = _f_cmd(i);
		_nu(i+3) = _t_cmd(i);
	}

	SquareMatrix<float, 6> F_inert_rot;
	F_inert_rot.setIdentity();
	F_inert_rot.slice<3,3>(0,0) = R_q.T();

	_f_rot_cmd = _B_pinv*F_inert_rot*_nu;

	// Project F_x, F_y, Tau_x, Tau_y, and Tau_z so that they are achievable with the current rotors
	float _f_eval = _f_rot_max - 2*_f_rot_min;
	float _f_rot_cmd_max = (_f_rot_cmd.abs()).max();
	int tmp_count = 0;
	while (_f_rot_cmd_max > _f_eval && tmp_count < 100){
		for (int i = 0; i<6; i++){
			if (i!=2){
				_nu(i)*=0.95f;
			}
		}
		_f_rot_cmd = _B_pinv*F_inert_rot*_nu;
		_f_rot_cmd_max = (_f_rot_cmd.abs()).max();
		tmp_count++;
	}
	// If still not achievable, project F_z too
	if (_f_rot_cmd_max > _f_eval){
		_nu*=_f_eval/_f_rot_cmd_max;
		_f_rot_cmd = _B_pinv*F_inert_rot*_nu;
	}

	// Motor optimization with phi

	Vector<float, 4> _f_hat_1;
	Vector<float, 4> _f_hat_2;

	for (int i = 0; i < 4; i++){
		_f_hat_1(i) = _f_rot_cmd(i);
		_f_hat_2(i) = _f_rot_cmd(i+4);

		_f_rot_1(i) = _f_rot_filt(i);
		_f_rot_2(i) = _f_rot_filt(i+4);
	}

	_phi_1 = phi_optimizer(_f_hat_1, _f_rot_1, _f_rot_cmd_old.slice<4,1>(0,0), _phi_1, _f_rot_max, _f_rot_min, _motor_tau_hyst_1);
	_phi_2 = phi_optimizer(_f_hat_2, _f_rot_2, _f_rot_cmd_old.slice<4,1>(4,0), _phi_2, _f_rot_max, _f_rot_min, _motor_tau_hyst_2);

	float eta_1_coef[8] = {1, 1, 1, 1, 0, 0, 0, 0};
	Vector<float, 8> _eta_1(eta_1_coef);
	float eta_2_coef[8] = {0, 0, 0, 0, 1, 1, 1, 1};
	Vector<float, 8> _eta_2(eta_2_coef);

	_f_rot_cmd += _phi_1*_eta_1 + _phi_2*_eta_2;


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
		for (int i = 0; i < 4; i++){
			_debug_file.motor_tau[i] = _motor_tau_hyst_1(i);
			_debug_file.motor_tau[i+4] = _motor_tau_hyst_2(i);
		}
		_debug_file.batt_volt = _battery_voltage;
		_debug_file.phi[0] = _phi_1;
		_debug_file.phi[1] = _phi_2;
		_debug_file.f_rot_lim[0] = _f_rot_min;
		_debug_file.f_rot_lim[1] = _f_rot_max;
		_debug_file.mot_filt[0] = a1;
		_debug_file.mot_filt[1] = b1;
		// Debug data is only published at every motor update
		_debug_file.timestamp = hrt_absolute_time();
		_debug_pub.publish(_debug_file);
	}

	_actuator_pub.publish(_motor_cmd);

	// Update modeled actual rotor force;
	_f_rot_filt = b1*_f_rot_filt + a1*_f_rot_cmd + a2*_f_rot_cmd_old;
	_f_rot_cmd_old = _f_rot_cmd;
}

float omni_controller::phi_optimizer(const Vector<float, 4> &fhat, const Vector<float, 4> &f_old, const Vector<float, 4> &f_cmd_old, const float &phi, const float &f_max, const float &f_min, Vector<float, 4> &motor_tau)
{
	// Find optimum phi
	float phi_opt = bisection(fhat, f_old, f_cmd_old, phi);

	// Compute set Phi_hyst
	Vector<float, 4> phi_min_lim;
	Vector<float, 4> phi_max_lim;
	uint64_t current_time = hrt_absolute_time();
	Vector<float, 4> motor_d_tau;
	for (int i = 0; i<4; i++){
		motor_d_tau(i) = (current_time - motor_tau(i))/1000000.0f;
	}
	Vector<float, 4> motor_d_tau_sorted = motor_d_tau;
	Matrix<float, 4, 3> phi_exclusion;
	Vector<bool, 4> valid_exclusion;

	phi_exclusion.setZero();
	valid_exclusion.setZero();

	float tau_lim = tau_hyst;
	int tau_count = 0;

	float phi_min = 1;
	float phi_max = -1;
	// Sort time sinc motor direction flip in order of descending times since motor direction flip and
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
	// Compute set phi_hysteris boundaries
	PHI_LIM_RESET:
	// Find phi min, max, and exclusion zones such that phi min < phi max
	while(phi_min > phi_max && tau_count <= 4){
		for(int i = 0; i<4; i++){
			if(motor_d_tau(i) >= tau_lim){
				phi_min_lim(i) = -1.0f*(f_max + fhat(i));
				phi_max_lim(i) = f_max - fhat(i);

				phi_exclusion(i,0) = -1.0f*(f_min + fhat(i));
				phi_exclusion(i,1) = f_min - fhat(i);
				valid_exclusion(i) = true;
			}
			else if (f_cmd_old(i) > 0){ // if the previous commanded force was positive
				phi_min_lim(i) = f_min - fhat(i);
				phi_max_lim(i) = f_max - fhat(i);

				phi_exclusion(i,0) = 10.0f;
				valid_exclusion(i) = false;
			}
			else{
				phi_min_lim(i) = -1.0f*(f_max + fhat(i));
				phi_max_lim(i) = -1.0f*(f_min + fhat(i));

				phi_exclusion(i,0) = 10.0f;
				valid_exclusion(i) = false;
			}
		}
		phi_min = phi_min_lim.max();
		phi_max = phi_max_lim.min();
		if(tau_count < 4){
			tau_lim = motor_d_tau_sorted(tau_count);
		}
		tau_count++;
	}
	// Sort phi exclusion in order of increasing minimum values
	Vector2f excl_temp;
	bool valid_temp;
	for(int i = 0; i<4; i++){
		for (int j = 0; j<i; j++){
			if(phi_exclusion(i,0) > phi_exclusion(j,0)){
				excl_temp = phi_exclusion.slice<1,2>(j,0);
				valid_temp = valid_exclusion(j);
				phi_exclusion.slice<1,2>(j,0) = phi_exclusion.slice<1,2>(i,0);
				valid_exclusion(j) = valid_exclusion(i);
				phi_exclusion.slice<1,2>(i,0) = excl_temp;
				valid_exclusion(i) = valid_temp;
			}
		}
	}
	// Combine overlapping phi exclusion zones
	for(int i = 1; i<4; i++){
		if(valid_exclusion(i) && valid_exclusion(i-1) && phi_exclusion(i,0) < phi_exclusion(i-1,1)){
			phi_exclusion(i,0) = phi_exclusion(i-1,0);
			valid_exclusion(i-1) = false;
		}
	}
	// Compare phi exlusion limits with phi min and max limits
	for(int i = 0; i<4; i++){
		if(valid_exclusion(i)){
			if(phi_exclusion(i,1) < phi_min){
				valid_exclusion(i) = false;
			}
			else if((phi_exclusion(i,0) < phi_min) && (phi_exclusion(i,1) > phi_min)){
				phi_min = phi_exclusion(i,1);
				valid_exclusion(i) = false;
			}
			else if((phi_exclusion(i,0) < phi_max) && (phi_exclusion(i,1) > phi_max)){
				phi_max = phi_exclusion(i,0);
				valid_exclusion(i) = false;
			}
			else if(phi_exclusion(i,0) > phi_max){
				valid_exclusion(i) = false;
			}
		}
	}
	// If the phi_max is less than phi_min, decrease tau hsyteris and recompute limits.
	if(phi_max < phi_min && tau_count <= 4){
		goto PHI_LIM_RESET;
	}
	// Check if phi lies in the set Phi_hyst, if not place phi at closest boundary
	if (phi_opt <= phi_min){
		phi_opt = phi_min;
	}
	else if(phi_opt >= phi_max){
		phi_opt = phi_max;
	}
	else{
		for(int i = 0; i<4; i++){
			if(valid_exclusion(i)){
				if(phi_opt < phi_exclusion(i,0)){
					break;
				}
				else if(phi_opt < phi_exclusion(i,1)){
					if(phi_opt > (phi_exclusion(i,0) + phi_exclusion(i,1))/2){
						phi_opt = phi_exclusion(i,1);
					}
					else{
						phi_opt = phi_exclusion(i,0);
					}
					break;
				}
			}
		}
	}
	// Check if commanded motor direction has changed and if so, update timestamp of motor diretion change.
	float fcmd;
	for(int i = 0; i<4; i++){
		fcmd = fhat(i) + phi_opt;
		if ( (fcmd > 0)^(f_cmd_old(i) > 0) ){
			motor_tau(i) = current_time;
		}
	}
	// return phi
	return phi_opt;
}

float omni_controller::bisection(const Vector<float,4> &f_hat, const Vector<float, 4> &f_old , const Vector<float, 4> &f_cmd_old, const float &phi)
{
	float phi_temp = phi;
	float low = phi_temp - 1.5f;
	float high = phi_temp + 1.5f;
	float J_phi;
	uint8_t bisect_loop_count = 0;
	float err = 3.0f;

	while(err > 0.01f && bisect_loop_count < 10){
		J_phi = objective_func(phi_temp, f_hat, f_old, f_cmd_old);
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

float omni_controller::objective_func(const float &phi, const Vector<float, 4> &fhat, const Vector<float, 4> &frot, const Vector<float, 4> &f_cmd_old)
{
	float J = 0.0f;
	float sgn;
	for(size_t i=0; i<4; i++) {
		sgn = ((fhat(i) + phi) > 0) - ((fhat(i) + phi) < 0);
		J += 1.5f * (1.0f - eps) * sgn * powf(fabsf(fhat(i) + phi), 0.5f) + eps * (fhat(i) + phi - frot(i)) + eps * (fhat(i) + phi - f_cmd_old(i));
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
		F_max = _param_F_max.get();
		T_max = _param_T_max.get();

		//Attitude aMRAC Control
		kq = _param_att_kq.get();
		kw = _param_att_kw.get();
		ke = _param_att_ke.get();

		Gtheta.setIdentity();
		Gtheta *= _param_att_gtheta.get();
		Glambda.setIdentity();
		Glambda *= _param_att_glambda.get();
		Gtau.setIdentity();
		Gtau *= _param_att_gtau.get();

		Tmaxtheta = _param_att_tmaxtheta.get();
		Tmaxlambda = _param_att_tmaxlambda.get();
		Tmaxtd = _param_att_tmaxtd.get();

		_J(0,0) = _param_J_xx.get();
		_J(0,1) = 0;
		_J(0,2) = 0;
		_J(1,0) = 0;
		_J(1,1) = _param_J_yy.get();
		_J(1,2) = 0;
		_J(2,0) = 0;
		_J(2,1) = 0;
		_J(2,2) = _param_J_zz.get();

		lambdaHat0 = inv(_J);

		thetaHat0.setZero();
		thetaHat0(0,0) = (_param_J_zz.get() - _param_J_yy.get())/_param_J_xx.get();
		thetaHat0(1,1) = (_param_J_xx.get() - _param_J_zz.get())/_param_J_yy.get();
		thetaHat0(2,2) = (_param_J_yy.get() - _param_J_xx.get())/_param_J_zz.get();

		Tau_f = _param_tau_f.get();
		eps = _param_ctrl_eps.get();
		throt_max = _param_throt_max.get();
		throt_min = _param_throt_min.get();
		tau_hyst = _param_tau_hyst.get();
		tau_mot = _param_tau_mot.get()*1000;

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
		_pos_sp_enu = Vector3f(_pos_sp.y, _pos_sp.x, -_pos_sp.z);
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
		Vector3f _w_sp_enu(_att_rate_sp.roll, -_att_rate_sp.pitch, -_att_rate_sp.yaw);
		Vector3f _w_dot_sp_enu = (_att_rate_sp_enu - _w_cmd_old)/(0.02f);
		// Filter omega and omega_dot to remove sharp steps
		_att_rate_sp_enu = 0.818f*_att_rate_sp_enu + 0.091f*_w_sp_enu + 0.091f*_w_cmd_old;
		_w_des_dot_sp_enu = 0.818f*_w_des_dot_sp_enu + 0.091f*_w_dot_sp_enu + 0.091f*_w_dot_old;
		// Store old sp
		_w_cmd_old = _w_sp_enu;
		_w_dot_old = _w_dot_sp_enu;
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

	PRINT_MODULE_USAGE_NAME("omni_amrac_amrac", "contoller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int omni_controller::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int omni_amrac_amrac_main(int argc, char *argv[])
{
	return omni_controller::main(argc, argv);
}
