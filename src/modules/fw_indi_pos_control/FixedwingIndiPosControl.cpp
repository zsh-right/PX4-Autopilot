/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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

#include "FixedwingIndiPosControl.hpp"

using namespace time_literals;
using namespace matrix;

using math::constrain;
using math::interpolate;
using math::radians;

FixedwingIndiPosControl::FixedwingIndiPosControl(bool vtol) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	/* fetch initial parameter values */
	parameters_update();
}

FixedwingIndiPosControl::~FixedwingIndiPosControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingIndiPosControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int
FixedwingIndiPosControl::parameters_update()
{
	return PX4_OK;
}

void
FixedwingIndiPosControl::vehicle_attitude_poll()
{
	vehicle_attitude_s vehicle_attitude;

	if (_vehicle_attitude_sub.update(&vehicle_attitude)) {
		vehicle_attitude_ = Quatf(vehicle_attitude.q);
	}

}


void
FixedwingIndiPosControl::vehicle_local_position_poll()
{
	vehicle_local_position_s pos;

	if (_vehicle_local_position_sub.update(&pos)) {
		vehicle_position_ = Vector3f{pos.x, pos.y, pos.z};
		vehicle_velocity_ = Vector3f{pos.vx, pos.vy, pos.vz};
		// take accel from faster message, since 50Hz is too slow...
	}
}

Quatf
FixedwingIndiPosControl::get_flat_attitude(Vector3f vel, Vector3f f)
{

	Vector3f vel_air = vel - wind_estimate_;
	// compute force component projected onto lift axis
	Vector3f vel_normalized = vel_air.normalized();
	Vector3f f_lift = f - (f * vel_normalized) * vel_normalized;
	Vector3f lift_normalized = f_lift.normalized();
	Vector3f wing_normalized = -vel_normalized.cross(lift_normalized);
	// compute rotation matrix between ENU and FRD frame
	Dcmf R_bi;
	R_bi(0, 0) = vel_normalized(0);
	R_bi(0, 1) = vel_normalized(1);
	R_bi(0, 2) = vel_normalized(2);
	R_bi(1, 0) = wing_normalized(0);
	R_bi(1, 1) = wing_normalized(1);
	R_bi(1, 2) = wing_normalized(2);
	R_bi(2, 0) = lift_normalized(0);
	R_bi(2, 1) = lift_normalized(1);
	R_bi(2, 2) = lift_normalized(2);
	R_bi.renormalize();

	///TODO:
	float rho_corrected{1.0};
	float _stall_speed{1.0};
	float _rho{1.0};
	float _cal_airspeed{1.0};
	float _true_airspeed{1.0};
	float _area{1.0};

	if (_cal_airspeed >= _stall_speed) {
		rho_corrected = _rho * powf(_cal_airspeed / _true_airspeed, 2);

	} else {
		rho_corrected = _rho;
	}

	// compute required AoA
	Vector3f f_phi = R_bi * f_lift;
	float _C_L0{0.1};
	float _C_L1{0.1};
	float _aoa_offset{0.1};
	float AoA = ((2.f * f_phi(2)) / (rho_corrected * _area * (vel_air * vel_air) + 0.001f) - _C_L0) / _C_L1 - _aoa_offset;
	// compute final rotation matrix
	Eulerf e(0.f, AoA, 0.f);
	Dcmf R_pitch(e);
	Dcmf Rotation(R_pitch * R_bi);
	// switch from FRD to ENU frame
	Rotation(1, 0) *= -1.f;
	Rotation(1, 1) *= -1.f;
	Rotation(1, 2) *= -1.f;
	Rotation(2, 0) *= -1.f;
	Rotation(2, 1) *= -1.f;
	Rotation(2, 2) *= -1.f;

	Quatf q(Rotation.transpose());
	return q;
}

Vector3f
FixedwingIndiPosControl::computeIndi(Vector3f pos_ref, Vector3f vel_ref, Vector3f acc_ref)
{
	Dcmf R_ib(vehicle_attitude_);
	Dcmf R_bi(R_ib.transpose());

	// apply LP filter to acceleration & velocity
	Vector3f acc_filtered;
	///TODO: Poll acceleration
	Vector3f _acc;
	acc_filtered(0) = _lp_filter_accel[0].apply(_acc(0));
	acc_filtered(1) = _lp_filter_accel[1].apply(_acc(1));
	acc_filtered(2) = _lp_filter_accel[2].apply(_acc(2));

	//     Vector3f omega_filtered;
//     omega_filtered(0) = _lp_filter_omega[0].apply(_omega(0));
//     omega_filtered(1) = _lp_filter_omega[1].apply(_omega(1));
//     omega_filtered(2) = _lp_filter_omega[2].apply(_omega(2));

	// =========================================
	// apply PD control law on the body position
	// =========================================
	float _K_x{1.0};
	float _K_v{1.0};
	float _K_a{1.0};
	Vector3f acc_command_local = (_K_x * R_bi * (pos_ref - vehicle_position_) + _K_v * R_bi *
				      (vel_ref - vehicle_velocity_) + _K_a * R_bi *
				      (acc_ref - _acc));
	Vector3f acc_command = R_ib * acc_command_local + acc_ref; ///TODO: Why add acc_ref again?

	// ==================================
	// compute expected aerodynamic force
	// ==================================
	Vector3f _wind_estimate;
	Vector3f airvel_body = R_bi * (vehicle_velocity_ - _wind_estimate);

	// Vehicle parameters
	float _aoa_offset{0.0};
	float _C_L0{0.1};
	float _C_D0{0.1};
	float _C_L1{0.1};
	float _C_D1{0.1};
	float _C_D2{0.1};
	float _area{1.0};

	float angle_of_attack = atan2f(airvel_body(2), airvel_body(0)) + _aoa_offset;

	float C_l = _C_L0 + _C_L1 * angle_of_attack;
	float C_d = _C_D0 + _C_D1 * angle_of_attack + _C_D2 * powf(angle_of_attack, 2);
	// compute actual air density
	float rho_corrected{1.0};
//     if (_cal_airspeed>=_stall_speed) {
//         rho_corrected = _rho*powf(_cal_airspeed/_true_airspeed, 2);
//     }
//     else {
//         rho_corrected = _rho;
//     }
	float factor = -0.5f * rho_corrected * _area * sqrtf(airvel_body * airvel_body);

	// Wind axis
	Vector3f w_x = airvel_body;
	Vector3f w_z = w_x.cross(Vector3f{0.f, 1.f, 0.f});
	Vector3f f_current = R_ib * (factor * (C_l * w_z + C_d * w_x));
//     // apply LP filter to force
	Vector3f f_current_filtered;
	f_current_filtered(0) = _lp_filter_force[0].apply(f_current(0));
	f_current_filtered(1) = _lp_filter_force[1].apply(f_current(1));
	f_current_filtered(2) = _lp_filter_force[2].apply(f_current(2));

	// ================================
	// get force command in world frame
	// ================================
	float _mass{1.0};
	Vector3f f_command = _mass * (acc_command - acc_filtered) + f_current_filtered;

	// ============================================================================================================
	// apply some filtering to the force command. This introduces some time delay,
	// which is not desired for stability reasons, but it rejects some of the noise fed to the low-level controller
	// ============================================================================================================
	f_command(0) = _lp_filter_ctrl0[0].apply(f_command(0));
	f_command(1) = _lp_filter_ctrl0[1].apply(f_command(1));
	f_command(2) = _lp_filter_ctrl0[2].apply(f_command(2));
//     _f_command = f_command;
	// limit maximum lift force by the maximum lift force, the aircraft can produce (assume max force at 15° aoa)

	// ====================================================================
	// saturate force command to avoid overly agressive maneuvers and stall
	// ====================================================================
	if (_switch_saturation) {
		float speed = airvel_body * airvel_body;
//         // compute maximum achievable force
		float f_max{1.0};
		float _stall_speed{1.0};

		if (speed > _stall_speed) {
			f_max = -factor * sqrtf(airvel_body * airvel_body) * (_C_L0 + _C_L1 * 0.25f); // assume stall at 15° AoA

		} else {
			f_max = -factor * _stall_speed * (_C_L0 + _C_L1 * 0.25f); // assume stall at 15° AoA
		}

		// compute current command
		float f_now = sqrtf(f_command * f_command);

		// saturate current command
		if (f_now > f_max) {
			f_command = f_max / f_now * f_command;
		}
	}

	// ==========================================================================
	// get required attitude (assuming we can fly the target velocity), and error
	// ==========================================================================
	Dcmf R_ref(get_flat_attitude(vel_ref, f_command));
	// get attitude error
	Dcmf R_ref_true(R_ref.transpose()*R_ib);
	// get required rotation vector (in body frame)
	AxisAnglef q_err(R_ref_true);
	Vector3f w_err;

	// project rotation angle to [-pi,pi]
	if (q_err.angle()*q_err.angle() < M_PI_F * M_PI_F) {
		w_err = -q_err.angle() * q_err.axis();

	} else {
		if (q_err.angle() > 0.f) {
			w_err = (2.f * M_PI_F - (float)fmod(q_err.angle(), 2.f * M_PI_F)) * q_err.axis();

		} else {
			w_err = (-2.f * M_PI_F - (float)fmod(q_err.angle(), 2.f * M_PI_F)) * q_err.axis();
		}
	}

	// TODO: Reference rate to rate controller
	// =========================================
	// apply PD control law on the body attitude
	// =========================================
	Vector3f rot_acc_command;
//     Vector3f rot_acc_command = _K_q*w_err + _K_w*(omega_ref-_omega) + alpha_ref;

//     if (sqrtf(w_err*w_err)>M_PI_F){
//         PX4_ERR("rotation angle larger than pi: \t%.2f, \t%.2f, \t%.2f", (double)sqrtf(w_err*w_err), (double)q_err.angle(), (double)(q_err.axis()*q_err.axis()));
//     }

	// ====================================
	// manual attitude setpoint feedthrough
	// ====================================
//     if (_switch_manual){
//         // get an attitude setpoint from the current manual inputs
//         float roll_ref = 1.f * _manual_control_setpoint.y * 1.0f;
//         float pitch_ref = -1.f* _manual_control_setpoint.x * M_PI_4_F;
//         Eulerf E_current(Quatf(_attitude.q));
//         float yaw_ref = E_current.psi();
//         Dcmf R_ned_frd_ref(Eulerf(roll_ref, pitch_ref, yaw_ref));
//         Dcmf R_enu_frd_ref(_R_ned_to_enu*R_ned_frd_ref);
//         Quatf att_ref(R_enu_frd_ref);
//         R_ref = Dcmf(att_ref);

//         // get attitude error
//         R_ref_true = Dcmf(R_ref.transpose()*R_ib);
//         // get required rotation vector (in body frame)
//         q_err = AxisAnglef(R_ref_true);
//         // project rotation angle to [-pi,pi]
//         if (q_err.angle()*q_err.angle()<M_PI_F*M_PI_F){
//             w_err = -q_err.angle()*q_err.axis();
//         }
//         else{
//             if (q_err.angle()>0.f){
//                 w_err = (2.f*M_PI_F-(float)fmod(q_err.angle(),2.f*M_PI_F))*q_err.axis();
//             }
//             else{
//                 w_err = (-2.f*M_PI_F-(float)fmod(q_err.angle(),2.f*M_PI_F))*q_err.axis();
//             }
//         }
//         _w_err = w_err;

//         // compute rot acc command
//         rot_acc_command = _K_q*w_err + _K_w*(Vector3f{0.f,0.f,0.f}-_omega);

//     }

	// ==============================================================
	// overwrite rudder rot_acc_command with turn coordination values
	// ==============================================================
//     Vector3f vel_air = _vel - _wind_estimate;
//     Vector3f vel_normalized = vel_air.normalized();
//     Vector3f acc_normalized = acc_filtered.normalized();
//     // compute ideal angular velocity
//     Vector3f omega_turn_ref_normalized = vel_normalized.cross(acc_normalized);
//     Vector3f omega_turn_ref;
//     // constuct acc perpendicular to flight path
//     Vector3f acc_perp = acc_filtered - (acc_filtered*vel_normalized)*vel_normalized;
//     if (_airspeed_valid&&_cal_airspeed>_stall_speed) {
//         omega_turn_ref = sqrtf(acc_perp*acc_perp) / (_true_airspeed) * R_bi * omega_turn_ref_normalized.normalized();
//         //PX4_INFO("yaw rate ref, yaw rate: \t%.2f\t%.2f", (double)(omega_turn_ref(2)), (double)(omega_filtered(2)));
//     }
//     else {
//         omega_turn_ref = sqrtf(acc_perp*acc_perp) / (_stall_speed) * R_bi * omega_turn_ref_normalized.normalized();
//         //PX4_ERR("No valid airspeed message detected or airspeed too low");
//     }

//     // apply some smoothing since we don't want HF components in our rudder output
//     omega_turn_ref(2) = _lp_filter_rud.apply(omega_turn_ref(2));

//     // transform rate vector to body frame
//     float scaler = (_stall_speed*_stall_speed)/fmaxf(sqrtf(vel_body*vel_body)*vel_body(0), _stall_speed*_stall_speed);
//     // not really an accel command, rather a FF-P command
//     rot_acc_command(2) = _K_q(2,2)*omega_turn_ref(2)*scaler + _K_w(2,2)*(omega_turn_ref(2) - omega_filtered(2))* scaler*scaler;


	return rot_acc_command;
}

void FixedwingIndiPosControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// only run controller if angular velocity changed
	if (_vehicle_angular_velocity_sub.updated() || (hrt_elapsed_time(&_last_run) > 20_ms)) { //TODO rate!

		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}

		// Poll local position
		vehicle_local_position_poll();

		// Poll trajectory setpoints
		trajectory_setpoint_s trajectory_setpoint;

		if (_trajectory_setpoint_sub.update(&trajectory_setpoint)) {
			///TODO: Check if references are invalid
			pos_ref_ = Vector3f(trajectory_setpoint.position);
			vel_ref_ = Vector3f(trajectory_setpoint.velocity);
			acc_ref_ = Vector3f(trajectory_setpoint.acceleration);
		}

		// Compute bodyrate commands
		Vector3f rate_command = computeIndi(pos_ref_, vel_ref_, acc_ref_);
		rate_command(0) = rate_command(0);


		///TODO: Publish offboard mode

		///TODO: Publish bodyrate commands in offboard mode
		// if (_control_mode.flag_control_offboard_enabled) {
		// }

	}

	// backup schedule
	ScheduleDelayed(20_ms);

	perf_end(_loop_perf);
}

int FixedwingIndiPosControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingIndiPosControl *instance = new FixedwingIndiPosControl(vtol);

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

int FixedwingIndiPosControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingIndiPosControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_rate_control is the fixed-wing rate controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_indi_pos_control_main(int argc, char *argv[])
{
	return FixedwingIndiPosControl::main(argc, argv);
}
