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

 bool OpticalFlowVelocity::fuseScalarVelocity(
    Ekf &ekf,
    const Vector3f &h_body,      // Measurement direction in body frame
    float measurement,
    float variance,
    uint8_t quality)
{
    // Transform h from body to NED frame for state vector
    const Vector3f h_ned = ekf._R_to_earth * h_body;

    // Build H vector (maps state to measurement)
    Ekf::VectorState H;
    H.setZero();

	static constexpr uint8_t VEL_NED_IDX = 3;
	H(VEL_NED_IDX + 0) = h_ned(0);  // d(measurement)/d(vel_N)
	H(VEL_NED_IDX + 1) = h_ned(1);  // d(measurement)/d(vel_E)
	H(VEL_NED_IDX + 2) = h_ned(2);  // d(measurement)/d(vel_D)

	const Vector3f vel_body_est = ekf._R_to_earth.transpose() * ekf._state.vel;

	const float predicted = h_body.dot(vel_body_est);

	// Innovation (measurement residual)
	const float innovation = predicted - measurement;
	// const float innovation = measurement - predicted;

	// Innovation variance: H * P * H^T + R
	// Using matrix operations as PX4 does elsewhere
	const Ekf::VectorState PH = ekf.P * H;  // P * H^T (H is column vector)
	float innov_var = H.dot(PH) + variance;  // H^T * P * H + R

    if (innov_var < variance) {
        // Numerical protection
        innov_var = variance;
    }

	if (!PX4_ISFINITE(innov_var) || innov_var < 1e-6f) {
		return false;
	}

    // Innovation gate check
    const float test_ratio = (innovation * innovation) / innov_var;
    if (test_ratio > _ekf2_of_gate * _ekf2_of_gate) {
        return false;  // Reject outlier
    }

    // Kalman gain
    Ekf::VectorState K = ekf.P * H / innov_var;

    // State update
    ekf.measurementUpdate(K, H, variance, innovation);

	return true;
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

	if (!PX4_ISFINITE(range_m)) {
		return;
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

		const float vel_from_flow_x = sample.range_m * flow_compensated_xy_rad(0) / sample.flow_dt;
        const float vel_from_flow_y = sample.range_m * flow_compensated_xy_rad(1) / sample.flow_dt;

		// Compute observation variance
		const float obs_var = computeObservationVariance(sample.range_m, sample.flow_quality);

		// Get current body velocity estimate for debugging/logging
		const Vector3f vel_body_est = ekf._R_to_earth.transpose() * ekf._state.vel;

	 	// Define conditions for using this measurement
	 	const uint8_t quality_threshold = getMinQualityThreshold();
	 	const bool continuing_conditions = ekf.control_status_flags().tilt_align
					&& sample.flow_quality > quality_threshold
					&& PX4_ISFINITE(sample.range_m);

	 	const bool starting_conditions = continuing_conditions
				      && (sample.flow_quality > 50);

		_fused_flow_x = false;
		_fused_flow_y = false;

		// State machine to manage the optical flow fusion
		switch (_state) {
		case State::stopped:
		/* FALLTHROUGH */
		case State::starting:
	    	if (starting_conditions) {
			_state = State::starting;

			_fused_flow_x = fuseScalarVelocity(ekf, _h_flow_x, vel_from_flow_x, obs_var, imu_delayed.time_us);
			_fused_flow_y = fuseScalarVelocity(ekf, _h_flow_y, vel_from_flow_y, obs_var, imu_delayed.time_us);
			// printf("Fusion Status: Instance %d Flow_x %s flow_y %s\n", kFlowInstance, _fused_flow_x   ? "true":"false", _fused_flow_y   ? "true":"false");

			if (_fused_flow_x || _fused_flow_y) {
				ekf.enableControlStatusOpticalFlowVelocity(kFlowInstance);
				_state = State::active;
			}
	    }
	    break;

	case State::active:
    	if (continuing_conditions) {
    	    _fused_flow_x = fuseScalarVelocity(ekf, _h_flow_x, vel_from_flow_x, obs_var, sample.flow_quality);
    	    _fused_flow_y = fuseScalarVelocity(ekf, _h_flow_y, vel_from_flow_y, obs_var, sample.flow_quality);

    	    if (_fused_flow_x || _fused_flow_y) {
    	        ekf._time_last_hor_vel_fuse = imu_delayed.time_us;

    	        // Check if this sensor contributes to vertical velocity
    	        if (fabsf(_h_flow_x(2)) > 0.3f || fabsf(_h_flow_y(2)) > 0.3f) {
    	            ekf._time_last_ver_vel_fuse = imu_delayed.time_us;
    	        }
    	    }
    	    // Stay in active state - DO NOT disable here!

    	} else {
    	    // Conditions no longer met - now we can disable
    	    ekf.disableControlStatusOpticalFlowVelocity(kFlowInstance);
    	    _state = State::stopped;
    	}
    	break;

	default:
	    break;
	}

 #if defined(MODULE_NAME)

	// Store innovations and test ratios
	const float pred_flow_x = _h_flow_x.dot(vel_body_est);
	const float pred_flow_y = _h_flow_y.dot(vel_body_est);

	aid_src.timestamp_sample = sample.time_us;
	aid_src.observation[0] = vel_from_flow_x;
	aid_src.observation[1] = vel_from_flow_y;
	aid_src.observation[2] = 0.f;  // Not used for scalar fusion

	aid_src.observation_variance[0] = obs_var;
	aid_src.observation_variance[1] = obs_var;
	aid_src.observation_variance[2] = 0.f;

	aid_src.innovation[0] = pred_flow_x - vel_from_flow_x;
	aid_src.innovation[1] = pred_flow_y - vel_from_flow_y;
	aid_src.innovation[2] = 0.f;

	aid_src.fused = _fused_flow_x || _fused_flow_y;
	aid_src.time_last_fuse = imu_delayed.time_us;

	aid_src.device_id = _ekf2_ds_id;
	aid_src.timestamp = hrt_absolute_time();
	_estimator_aid_src_optical_flow_velocity_pub.publish(aid_src);

	// Publish optical flow velocity
	{
		vehicle_optical_flow_vel_s flow_vel{};
		flow_vel.timestamp_sample = sample.time_us;

		// Sensor frame velocity (raw from flow + range)
		const Vector3f vel_sensor(vel_from_flow_y, vel_from_flow_x, 0.f);
		vel_sensor.copyTo(flow_vel.vel_sensor);

		// Body frame velocity (transformed)
		const Vector3f vel_body_raw = _R_sensor_to_body * vel_sensor;
		vel_body_raw.copyTo(flow_vel.vel_body_raw);

		// Account for lever arm (sensor offset from IMU)
		const Vector3f flow_pos_body(_ekf2_of_pos_x, _ekf2_of_pos_y, _ekf2_of_pos_z);
		const Vector3f angular_velocity = imu_delayed.delta_ang / imu_delayed.delta_ang_dt - ekf._state.gyro_bias;
		const Vector3f position_offset = flow_pos_body - ekf._params.imu_pos_body;
		const Vector3f velocity_offset = angular_velocity % position_offset;
		const Vector3f vel_body = vel_body_raw - velocity_offset;
		vel_body.copyTo(flow_vel.vel_body);

		// NED frame velocity
		const Vector3f vel_ned = ekf._R_to_earth * vel_body;
		vel_ned.copyTo(flow_vel.vel_ne);

		// Update filters
		_flow_sensor_vel_lpf.update(Vector2f(vel_body_raw(0), vel_body_raw(1)));
		_flow_body_vel_lpf.update(Vector2f(vel_body(0), vel_body(1)));
		_flow_mean.update(sample.flow_xy_rad);
		_flow_sensor_vel_mean.update(Vector2f(vel_body_raw(0), vel_body_raw(1)));

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
		const Vector3f ref_body_rate = -(imu_delayed.delta_ang / imu_delayed.delta_ang_dt - ekf.getGyroBias());
		gyro_rate.copyTo(flow_vel.gyro_rate);
		ref_body_rate.copyTo(flow_vel.ref_gyro);
		flow_vel.timestamp = hrt_absolute_time();
		_estimator_optical_flow_velocity_vel_pub.publish(flow_vel);
	}

	// Update test ratios for external monitoring
	_vel_ne_innovation(0) = pred_flow_x - vel_from_flow_x;
	_vel_ne_innovation(1) = pred_flow_y - vel_from_flow_y;
	_test_ratio_filtered = 0.9f * _test_ratio_filtered +
			       0.1f * math::max(fabsf(_vel_ne_innovation(0)), fabsf(_vel_ne_innovation(1))) / sqrtf(obs_var);

 #endif // MODULE_NAME

    } else if ((_state != State::stopped) && isTimedOut(_time_last_buffer_push, imu_delayed.time_us, (uint64_t)5e6)) {
		ekf.disableControlStatusOpticalFlowVelocity(kFlowInstance);
		_state = State::stopped;
		ECL_WARN("Optical flow velocity instance %d data stopped", kFlowInstance);
    }
}

 #endif // CONFIG_EKF2_OPTICAL_FLOW_VELOCITY
