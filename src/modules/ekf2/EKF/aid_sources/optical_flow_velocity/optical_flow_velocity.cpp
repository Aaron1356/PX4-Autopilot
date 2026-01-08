/****************************************************************************
 *
 *   Copyright (c) 2023-2025 PX4 Development Team. All rights reserved.
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

 #include "ekf.h"

 #include "aid_sources/optical_flow_velocity/optical_flow_velocity.hpp"

 #include "ekf_derivation/generated/compute_body_vel_innov_var_h.h"
 #include "ekf_derivation/generated/compute_body_vel_y_innov_var.h"
 #include "ekf_derivation/generated/compute_body_vel_z_innov_var.h"

 #if defined(CONFIG_EKF2_OPTICAL_FLOW_VELOCITY) && defined(MODULE_NAME)

 matrix::Dcmf OpticalFlowVelocity::calculateSensorToBodyRotation() const
 {
     // Get rotation parameters
     const float roll = math::radians(_ekf2_of_roll);
     const float pitch = math::radians(_ekf2_of_pitch);
     const float yaw = math::radians(_ekf2_of_yaw);

     // Create a rotation matrix using Euler angles
     return matrix::Dcmf(matrix::Eulerf(roll, pitch, yaw));
 }

 uint8_t OpticalFlowVelocity::getVelocityUpdateMask() const
 {
     uint8_t update_mask = 0;
     int type = _ekf2_of_mode;
     MountingType mounting_type = static_cast<MountingType>(type);

     // If not directly specified, determine from mounting type
     switch (mounting_type) {
	 case MountingType::FORWARD:
	     update_mask = 0b110; // Update Y and Z (bits 0 and 1)
	     break;

	 case MountingType::SIDEWAYS:
	     update_mask = 0b101; // Update X and Z (bits 0 and 2)
	     break;

	 case MountingType::UPWARD:
	     update_mask = 0b011; // Update X and Y (bits 0 and 1)
	     break;

	 case MountingType::DOWNWARD:
	     update_mask = 0b011; // Update X and Y (bits 0 and 1)
	     break;

	 case MountingType::CUSTOM:
	     update_mask = 0b111;
	     break;
     }

     return update_mask;
 }

 Vector3f OpticalFlowVelocity::flowToBodyVelocity(const Vector2f &flow_compensated_xy_rad, float range_m, float flow_dt) const
 {
     // Convert from flow rate to velocity in sensor frame
     Vector3f vel_sensor;
     vel_sensor(0) = -range_m * flow_compensated_xy_rad(1) / flow_dt;
     vel_sensor(1) = range_m * flow_compensated_xy_rad(0) / flow_dt;
     vel_sensor(2) = 0.f;  // Optical Flow Sensor velocity in the Z direction

     // Get rotation from sensor to body frame
     const matrix::Dcmf R_to_body = calculateSensorToBodyRotation();

     // Transform from sensor to body frame
     return R_to_body * vel_sensor;
 }

 void OpticalFlowVelocity::update(Ekf &ekf, const estimator::imuSample &imu_delayed)
 {
 #if defined(MODULE_NAME)
	if(!subOpticalInstanceSet){
		getOpticalFlowInstance(_ekf2_of_id);
	}
	if(!subDistanceInstanceSet){
		getDistanceSensorInstance(_ekf2_ds_id);
	}

	if(!subOpticalInstanceSet || !subDistanceInstanceSet){
		return;
	}

     if (_sensor_optical_flow_sub.updated()) {
	 sensor_optical_flow_s sensor_optical_flow{};
	 _sensor_optical_flow_sub.copy(&sensor_optical_flow);

	 float range_m = NAN;

	 {
	     distance_sensor_s distance_sensor{};

	     if (_distance_sensor_sub.copy(&distance_sensor)) {
		 if (PX4_ISFINITE(distance_sensor.current_distance)) {
		     range_m = distance_sensor.current_distance;
		 }
	     }
	 }

	 // NOTE: the EKF uses the reverse sign convention to the flow sensor. EKF assumes positive LOS rate
	 // is produced by a RH rotation of the image about the sensor axis.
	 Vector2f flow_xy_rad = Vector2f(-sensor_optical_flow.pixel_flow[0], -sensor_optical_flow.pixel_flow[1]);
	 Vector3f gyro_integral = Vector3f(-sensor_optical_flow.delta_angle[0], -sensor_optical_flow.delta_angle[1],
			       -sensor_optical_flow.delta_angle[2]);

	 const float flow_dt = 1e-6f * (float)sensor_optical_flow.integration_timespan_us;

	 // correct timestamp to midpoint of integration interval as the data is converted to rates
	 const int64_t time_us = sensor_optical_flow.timestamp_sample
		     - sensor_optical_flow.integration_timespan_us / 2
		     - static_cast<int64_t>(_ekf2_of_delay * 1000);

	 if (time_us > 0 && PX4_ISFINITE(range_m)) {
	     OpticalFlowSample sample{
		 .time_us = (uint64_t)time_us,
		 .flow_dt = flow_dt,
		 .flow_xy_rad = flow_xy_rad,
		 .gyro_integral = gyro_integral,
		 .range_m = range_m,
		 .flow_quality = sensor_optical_flow.quality
	     };

	     _ringbuffer.push(sample);
	     _time_last_buffer_push = imu_delayed.time_us;
	 }
     }

 #endif // MODULE_NAME

     OpticalFlowSample sample;

     if (_ringbuffer.pop_first_older_than(imu_delayed.time_us, &sample)) {
	 if (!_ekf2_of_ctrl) {
	     return;
	 }

	 estimator_aid_source3d_s &aid_src = _aid_src_optical_flow_velocity;

	 // compensate for body motion to give a LOS rate
	 const Vector2f flow_compensated_xy_rad = sample.flow_xy_rad - sample.gyro_integral.xy();

	Vector3f vel_sensor;
	vel_sensor(0) = -sample.range_m * flow_compensated_xy_rad(1) / sample.flow_dt;
	vel_sensor(1) = sample.range_m * flow_compensated_xy_rad(0) / sample.flow_dt;
	vel_sensor(2) = 0.f;  // Optical Flow Sensor velocity in the Z direction

	 // Transform from sensor to body frame considering arbitrary mounting
	 const Vector3f vel_body_raw = flowToBodyVelocity(flow_compensated_xy_rad, sample.range_m, sample.flow_dt);

	 // Get reference body rotation rate and correct for gyro bias
	 const Vector3f ref_body_rate = -(imu_delayed.delta_ang / imu_delayed.delta_ang_dt - ekf.getGyroBias());

	 // Get sensor position in body frame
	 Vector3f flow_pos_body = Vector3f(_ekf2_of_pos_x,
					_ekf2_of_pos_y,
					_ekf2_of_pos_z);

	 // Account for lever arm effect - angular velocity creates apparent velocity at sensor position
	 const Vector3f angular_velocity = imu_delayed.delta_ang / imu_delayed.delta_ang_dt - ekf._state.gyro_bias;
	 Vector3f position_offset_body = flow_pos_body - ekf._params.imu_pos_body;
	 const Vector3f velocity_offset_body = angular_velocity % position_offset_body; // Cross product
	 const Vector3f vel_body = vel_body_raw - velocity_offset_body;

	_flow_sensor_vel_lpf.update(Vector2f(vel_body_raw(0), vel_body_raw(1)));
	_flow_body_vel_lpf.update(Vector2f(vel_body(0), vel_body(1)));

	_flow_mean.update(sample.flow_xy_rad);
	_flow_sensor_vel_mean.update(Vector2f(vel_body_raw(0), vel_body_raw(1)));


	// Determine observation noise based on quality parameter
	// const float R = math::max(_ekf2_of_noise, 0.01f);

	const Vector3f measurement{vel_body};
	Vector3f measurement_var_scaled;

	// Get velocity update mask (which velocity components to use)
	const float base_noise = math::max(_ekf2_of_noise, 0.01f);
	const uint8_t vel_update_mask = getVelocityUpdateMask();

	const float quality_normalized = math::constrain((sample.flow_quality / 150.0f), 0.0f, 1.0f);
	const float quality_scale = 1.0f + 9.0f * (1.0f - quality_normalized);

	for (uint8_t i = 0 ; i < 3; i++){
		if(vel_update_mask & (1 << i)) {
			float scaled_noise = base_noise * quality_scale;
			measurement_var_scaled(i) = scaled_noise;
		} else {
			measurement_var_scaled(i) = 1e6f;
		}
	}
	const Vector3f measurement_var = measurement_var_scaled;

	// Calculate innovation: difference between predicted and measured body velocity
	Ekf::VectorState H[3];
	Vector3f innov_var;
	Vector3f innov = ekf._R_to_earth.transpose() * ekf._state.vel - vel_body;



	Vector3f observe_var;
	Vector3f range_scale = getAxisDependentRangeScale(sample.range_m);
	// observe_var = measurement_var * (1.0f + sample.range_m / _ekf2_obs_var_p);
	for (uint8_t i =0; i < 3; i++){
		observe_var(i) = measurement_var(i) * range_scale(i);
	}

	// Zero out components we don't want to update
	if (!(vel_update_mask & 0x1)) innov(0) = 0.f; // X
	if (!(vel_update_mask & 0x2)) innov(1) = 0.f; // Y
	if (!(vel_update_mask & 0x4)) innov(2) = 0.f; // Z

	 const auto state_vector = ekf._state.vector();
	 sym::ComputeBodyVelInnovVarH(state_vector, ekf.P, observe_var, &innov_var, &H[0], &H[1], &H[2]);

	 // Get innovation gate parameter
	 float innovation_gate = _ekf2_of_gate;

	 // Update aid source status with new measurement
	 ekf.updateAidSourceStatus(aid_src,
		       sample.time_us,        // sample timestamp
		       vel_body,              // observation
		       observe_var,           // observation variance
		       innov,                 // innovation
		       innov_var,             // innovation variance
		       innovation_gate);      // innovation gate

	 // Define conditions for using this measurement
	 const uint8_t quality_threshold = getMinQualityThreshold();
	 const bool continuing_conditions = ekf.control_status_flags().tilt_align
				&& sample.flow_quality > quality_threshold
				&& PX4_ISFINITE(sample.range_m);

	 const bool starting_conditions = continuing_conditions
			      && (sample.flow_quality > 100);

	 // State machine to manage the optical flow fusion
	 switch (_state) {
	 case State::stopped:
	 /* FALLTHROUGH */
	 case State::starting:
	     if (starting_conditions) {
		 _state = State::starting;

		 if ((_test_ratio_filtered > 0.f) && (_test_ratio_filtered < 0.5f)) {
		     // Conditions look good for starting fusion
		     bool fused = true;
		     bool reset = false;
		     if (fused || reset) {
			 ekf.enableControlStatusOpticalFlowVelocity(kFlowInstance);
			 _state = State::active;
		     }
		 }
	     }
	     break;

	 case State::active:
	     if (continuing_conditions) {
		 if (!aid_src.innovation_rejected) {
		     // Fuse each axis that's enabled by our mask
		     for (uint8_t index = 0; index <= 2; index++) {
			 // Skip axes we don't want to update
			 if (!(vel_update_mask & (1 << index))) {
			     continue;
			 }

			 if (index == 1) {
			     sym::ComputeBodyVelYInnovVar(state_vector, ekf.P, measurement_var(index), &aid_src.innovation_variance[index]);
			 } else if (index == 2) {
			     sym::ComputeBodyVelZInnovVar(state_vector, ekf.P, measurement_var(index), &aid_src.innovation_variance[index]);
			 }

			 aid_src.innovation[index] = Vector3f(ekf._R_to_earth.transpose().row(index)) * ekf._state.vel - measurement(index);

			 Ekf::VectorState Kfusion = ekf.P * H[index] / aid_src.innovation_variance[index];
			 ekf.measurementUpdate(Kfusion, H[index], aid_src.observation_variance[index], aid_src.innovation[index]);
		     }

		     aid_src.fused = true;
		     aid_src.time_last_fuse = imu_delayed.time_us;

		     // Better Notion of if this state is correct, need to sure that partial is timestamped
		     ekf._time_last_hor_vel_fuse = imu_delayed.time_us;
		     // Accounts for when tthe sensor is pointing in the Downward or Upward Directions
		     if(!(vel_update_mask & 0x4)){
		     	ekf._time_last_ver_vel_fuse = imu_delayed.time_us;
		     }
		 }

		 if (isTimedOut(aid_src.time_last_fuse, imu_delayed.time_us, ekf._params.no_aid_timeout_max)) {
		     if (ekf.isOnlyActiveSourceOfHorizontalPositionAiding(ekf.control_status_flags().optical_flow_velocity)) {
			 // TODO: Handle reset if this is the only source of horizontal aiding
		     } else {
			 ekf.disableControlStatusOpticalFlowVelocity(kFlowInstance);
			 _state = State::stopped;
		     }
		 }
	     } else {
		 ekf.disableControlStatusOpticalFlowVelocity(kFlowInstance);
		 _state = State::stopped;
	     }
	     break;

	 default:
	     break;
	 }

 #if defined(MODULE_NAME)
	 aid_src.device_id = _ekf2_ds_id;
	// Publish aid source data
	aid_src.timestamp = hrt_absolute_time();
	_estimator_aid_src_optical_flow_velocity_pub.publish(aid_src);

	// Publish optical flow velocity
	{
		vehicle_optical_flow_vel_s flow_vel{};
		flow_vel.timestamp_sample = sample.time_us;

		// Copy vectors to uORB message

		vel_sensor.copyTo(flow_vel.vel_sensor);

		vel_body_raw.copyTo(flow_vel.vel_body_raw);
		vel_body.copyTo(flow_vel.vel_body);

		const matrix::Vector3f vel_ned{ekf._R_to_earth * vel_body};
		vel_ned.copyTo(flow_vel.vel_ne);

		// Set filtered values
		_flow_sensor_vel_lpf.getState().copyTo(flow_vel.vel_sensor_filtered);
		_flow_body_vel_lpf.getState().copyTo(flow_vel.vel_body_filtered);

		// Set statistics
		_flow_mean.mean().copyTo(flow_vel.flow_mean);
		_flow_mean.variance().copyTo(flow_vel.flow_mean_var);
		_flow_sensor_vel_mean.mean().copyTo(flow_vel.vel_sensor_mean);
		_flow_sensor_vel_mean.variance().copyTo(flow_vel.vel_sensor_mean_var);

		// Set raw flow rates
		sample.flow_xy_rad.copyTo(flow_vel.flow_rate_uncompensated);
		flow_compensated_xy_rad.copyTo(flow_vel.flow_rate_compensated);

		// Set gyro rates
		const Vector3f gyro_rate = sample.gyro_integral / sample.flow_dt;
		gyro_rate.copyTo(flow_vel.gyro_rate);
		ref_body_rate.copyTo(flow_vel.ref_gyro);
		flow_vel.timestamp = hrt_absolute_time();
		_estimator_optical_flow_velocity_vel_pub.publish(flow_vel);
	}

	// Update test ratios
	_vel_ne_innovation = innov.xy();
	_vel_ne_test_ratio(0) = aid_src.test_ratio[0];
	_vel_ne_test_ratio(1) = aid_src.test_ratio[1];

	_test_ratio_filtered = math::max(fabsf(aid_src.test_ratio_filtered[0]), fabsf(aid_src.test_ratio_filtered[1]));
 #endif // MODULE_NAME

     } else if ((_state != State::stopped) && isTimedOut(_time_last_buffer_push, imu_delayed.time_us, (uint64_t)5e6)) {
	 ekf.disableControlStatusOpticalFlowVelocity(kFlowInstance);
	 _state = State::stopped;
	 ECL_WARN("Optical flow data stopped");
     }
 }

 #endif // CONFIG_EKF2_OPTICAL_FLOW_VELOCITY
