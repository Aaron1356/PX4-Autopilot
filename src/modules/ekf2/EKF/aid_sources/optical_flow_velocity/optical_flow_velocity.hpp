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

 #ifndef EKF_OPTICAL_FLOW_VELOCITY_HPP
 #define EKF_OPTICAL_FLOW_VELOCITY_HPP

 #include "../../common.h"
 #include "../../RingBuffer.h"

 #include <lib/mathlib/math/WelfordMeanVector.hpp>

 #if defined(CONFIG_EKF2_OPTICAL_FLOW_VELOCITY) && defined(MODULE_NAME)

 #if defined(MODULE_NAME)
 # include <px4_platform_common/module_params.h>
 # include <uORB/PublicationMulti.hpp>
 # include <uORB/Subscription.hpp>
 # include <uORB/SubscriptionMultiArray.hpp>
 # include <uORB/topics/distance_sensor.h>
 # include <uORB/topics/estimator_aid_source3d.h>
 # include <uORB/topics/sensor_optical_flow.h>
 # include <uORB/topics/vehicle_optical_flow_vel.h>
 # include <lib/drivers/device/Device.hpp>
 #endif // MODULE_NAME

 class Ekf;

 class OpticalFlowVelocity : public ModuleParams
 {
 public:
	// Define sensor mounting type
	enum class MountingType {
		 FORWARD = 0,  // Facing forward (default)
		 SIDEWAYS = 1, // Facing sideways
		 UPWARD = 2,   // Facing upward
		 DOWNWARD = 3, // Facing downward
		 CUSTOM = 4    // Custom orientation defined by parameters
	};

	OpticalFlowVelocity(int flowInstance = 0,MountingType type = MountingType::CUSTOM) :
		ModuleParams(nullptr),kFlowInstance(flowInstance),
		_mounting_type(type)
	{
		// Initialize parameter handles dynamically
		char param_name[17];

		snprintf(param_name, sizeof(param_name), "EKF2_OFV%d_CTRL", flowInstance);
		_param_ekf2_of_ctrl = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "EKF2_OFV%d_DELAY", flowInstance);
		_param_ekf2_of_delay = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "EKF2_OFV%d_NOISE", flowInstance);
		_param_ekf2_of_noise = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "EKF2_OFV%d_GATE", flowInstance);
		_param_ekf2_of_gate = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "EKF2_OFV%d_ROLL", flowInstance);
		_param_ekf2_of_roll = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "EKF2_OFV%d_PITCH", flowInstance);
		_param_ekf2_of_pitch = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "EKF2_OFV%d_YAW", flowInstance);
		_param_ekf2_of_yaw = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "EKF2_OFV%d_POS_X", flowInstance);
		_param_ekf2_of_pos_x = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "EKF2_OFV%d_POS_Y", flowInstance);
		_param_ekf2_of_pos_y = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "EKF2_OFV%d_POS_Z", flowInstance);
		_param_ekf2_of_pos_z = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "EKF2_OFV%d_MODE", flowInstance);
		_param_ekf2_of_mode = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "EKF2_OFV%d_VAR_P", flowInstance);
		_param_ekf2_obs_var_p = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "EKF2_OFV%d_OF_ID", flowInstance);
		_param_ekf2_of_id = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "EKF2_OFV%d_DS_ID", flowInstance);
		_param_ekf2_ds_id = param_find(param_name);

		_estimator_aid_src_optical_flow_velocity_pub.advertise();
		_estimator_optical_flow_velocity_vel_pub.advertise();

		updateParameters();
	}

	 ~OpticalFlowVelocity() = default;

	static OpticalFlowVelocity* create_instance(int instance_num){
		param_t control_bit = PARAM_INVALID;
		char param_name[17];
		int32_t tmp_int;
		snprintf(param_name, sizeof(param_name), "EKF2_OFV%d_CTRL", instance_num);
		control_bit = param_find(param_name);
		param_get(control_bit, &tmp_int);
		if(tmp_int){
			return new OpticalFlowVelocity(instance_num);
		}
		return nullptr;
	}

	void update(Ekf &ekf, const estimator::imuSample &imu_delayed);

	void updateParameters()
	{
		// Get parameter values using handles
		int32_t tmp_int;
		float tmp_float;

		if (param_get(_param_ekf2_of_ctrl, &tmp_int) == PX4_OK) {
			_ekf2_of_ctrl = (tmp_int != 0);
		}

		if (param_get(_param_ekf2_of_delay, &tmp_float) == PX4_OK) {
			_ekf2_of_delay = tmp_float;
		}

		if (param_get(_param_ekf2_of_noise, &tmp_float) == PX4_OK) {
			_ekf2_of_noise = tmp_float;
		}

		if (param_get(_param_ekf2_of_gate, &tmp_float) == PX4_OK) {
			_ekf2_of_gate = tmp_float;
		}

		if (param_get(_param_ekf2_of_roll, &tmp_float) == PX4_OK) {
			_ekf2_of_roll = tmp_float;
		}

		if (param_get(_param_ekf2_of_pitch, &tmp_float) == PX4_OK) {
			_ekf2_of_pitch = tmp_float;
		}

		if (param_get(_param_ekf2_of_yaw, &tmp_float) == PX4_OK) {
			_ekf2_of_yaw = tmp_float;
		}

		if (param_get(_param_ekf2_of_pos_x, &tmp_float) == PX4_OK) {
			_ekf2_of_pos_x = tmp_float;
		}

		if (param_get(_param_ekf2_of_pos_y, &tmp_float) == PX4_OK) {
			_ekf2_of_pos_y = tmp_float;
		}

		if (param_get(_param_ekf2_of_pos_z, &tmp_float) == PX4_OK) {
			_ekf2_of_pos_z = tmp_float;
		}

		if (param_get(_param_ekf2_of_mode, &tmp_int) == PX4_OK) {
			_ekf2_of_mode = tmp_int;
		}

		if (param_get(_param_ekf2_obs_var_p, &tmp_float) == PX4_OK) {
			_ekf2_obs_var_p = tmp_float;
		}

		if (param_get(_param_ekf2_ds_id, &tmp_int) == PX4_OK) {
			getDistanceSensorInstance(tmp_int);
		}

		if (param_get(_param_ekf2_of_id, &tmp_int) == PX4_OK) {
			getOpticalFlowInstance(tmp_int);
		}

		computeSensorRotation();
		updateParams();
	}

	void getDistanceSensorInstance(int32_t tmp_int){
		distance_sensor_s topic;
		_ekf2_ds_id = tmp_int;
		uORB::SubscriptionMultiArray<distance_sensor_s> distance_sensor_subs{ORB_ID::distance_sensor};
		if(subDistanceInstanceSet){
			return;
		}
		for(int i = 0; i < distance_sensor_subs.size(); i++){
			if(distance_sensor_subs[i].copy(&topic)){
				if((int)((topic.device_id >> 8) & 0xFF) == (int)_ekf2_ds_id)
				{
					// printf("Distance Sensor Looking at: %d\n", (int)((topic.device_id >> 8) & 0xFF));
					_distance_sensor_sub.ChangeInstance(i);
					subDistanceInstanceSet = true;
					break;
				}
			}

		}
	}

	void getOpticalFlowInstance(int32_t tmp_int){
		sensor_optical_flow_s topic;
		_ekf2_of_id = tmp_int;
		uORB::SubscriptionMultiArray<sensor_optical_flow_s> optical_flow_subs{ORB_ID::sensor_optical_flow};
		if(subOpticalInstanceSet){
			return;
		}
		for(int i = 0; i < optical_flow_subs.size(); i++){
			if(optical_flow_subs[i].copy(&topic)){
				if((int)((topic.device_id >> 8) & 0xFF) == (int)_ekf2_of_id)
				{
					// printf("Optical Flow Sensor Looking at: %d\n", (int)((topic.device_id >> 8) & 0xFF));
					_sensor_optical_flow_sub.ChangeInstance(i);
					subOpticalInstanceSet = true;
					break;
				}
			}
		}
	}

	/**
	 * @brief Pre-compute sensor to body rotation matrix and measurement directions
	 */
	void computeSensorRotation()
	{
		const float roll_rad = math::radians(_ekf2_of_roll);
		const float pitch_rad = math::radians(_ekf2_of_pitch);
		const float yaw_rad = math::radians(_ekf2_of_yaw);

		// Sensor to body rotation matrix
		_R_sensor_to_body = matrix::Dcmf(matrix::Eulerf(roll_rad, pitch_rad, yaw_rad));

		// Extract sensor axes in body frame
		_sensor_x_in_body = _R_sensor_to_body.col(0);  // Sensor X axis in body frame
		_sensor_y_in_body = _R_sensor_to_body.col(1);  // Sensor Y axis in body frame
		_sensor_z_in_body = _R_sensor_to_body.col(2);  // Sensor Z axis (viewing direction) in body frame

		_h_flow_x = _sensor_y_in_body.normalized();
		_h_flow_y = (-_sensor_x_in_body).normalized();
	}

	uint8_t getMinQualityThreshold() const
	{
		MountingType mounting_type = static_cast<MountingType>(_ekf2_of_mode);

		switch (mounting_type) {
			case MountingType::DOWNWARD:
				return 50; // Most reliable - lower threshold

			case MountingType::FORWARD:
			case MountingType::SIDEWAYS:
				return 80; // Less reliable for height - higher threshold

			case MountingType::UPWARD:
				return 100; // Least reliable - highest threshold

			default:
				// Scale based on how much we rely on range measurement
				float down_component = fabsf(_sensor_z_in_body(2));
				return (uint8_t)(50 + 50 * (1.0f - down_component));
		}
	}

	/**
	 * @brief Compute observation variance with range scaling
	 */
	float computeObservationVariance(float range_m, uint8_t quality) const
	{
		const float base_var = _ekf2_of_noise * _ekf2_of_noise;

		// Quality scaling (lower quality = higher variance)
		const float quality_normalized = math::constrain((float)quality / 255.0f, 0.1f, 1.0f);
		const float quality_scale = 1.0f / (quality_normalized * quality_normalized);

		// Range scaling (larger range = higher variance)
		const float range_scale = 1.0f + (range_m * range_m) / (_ekf2_obs_var_p * _ekf2_obs_var_p);

		return base_var * quality_scale * range_scale;
	}

	bool isHealthy() const { return _healthy; }
    float getTestRatioFiltered() const { return _test_ratio_lpf.getState(); }
    float getFusionRate() const { return _fusion_rate_lpf.getState(); }

	const matrix::Vector2f &test_ratio() const { return _vel_ne_test_ratio; }
	const matrix::Vector2f &innovation() const { return _vel_ne_innovation; }

	float test_ratio_filtered() const { return _test_ratio_filtered; }

	void setMountingType(MountingType type) { _mounting_type = type; }

 private:

	bool _ekf2_of_ctrl;
	float _ekf2_of_delay;
	float _ekf2_of_noise;
	float _ekf2_of_gate;
	float _ekf2_of_roll;
	float _ekf2_of_pitch;
	float _ekf2_of_yaw;
	float _ekf2_of_pos_x;
	float _ekf2_of_pos_y;
	float _ekf2_of_pos_z;
	int _ekf2_of_mode;
	float _ekf2_obs_var_p;
	int _ekf2_of_id;
	int _ekf2_ds_id;

	// Parameter handles for dynamic parameters
	param_t _param_ekf2_of_ctrl;
	param_t _param_ekf2_of_delay;
	param_t _param_ekf2_of_noise;
	param_t _param_ekf2_of_gate;
	param_t _param_ekf2_of_roll;
	param_t _param_ekf2_of_pitch;
	param_t _param_ekf2_of_yaw;
	param_t _param_ekf2_of_pos_x;
	param_t _param_ekf2_of_pos_y;
	param_t _param_ekf2_of_pos_z;
	param_t _param_ekf2_of_mode;
	param_t _param_ekf2_obs_var_p;
	param_t _param_ekf2_of_id;
	param_t _param_ekf2_ds_id;

	matrix::Dcmf _R_sensor_to_body{};
	Vector3f _sensor_x_in_body{};
	Vector3f _sensor_y_in_body{};
	Vector3f _sensor_z_in_body{};
	Vector3f _h_flow_x{};
	Vector3f _h_flow_y{};

	bool isTimedOut(uint64_t last_sensor_timestamp, uint64_t time_delayed_us, uint64_t timeout_period) const
	{
		return (last_sensor_timestamp == 0) || (last_sensor_timestamp + timeout_period < time_delayed_us);
	}

	bool fuseScalarVelocity(Ekf &ekf, const Vector3f &h_body, float measurement, float variance, uint8_t quality);

	struct OpticalFlowSample {
		uint64_t    time_us{};   ///< timestamp of the integration period midpoint (uSec)
		float       flow_dt{};
		Vector2f    flow_xy_rad{}; ///< measured angular rate of the image about the X and Y body axes (rad/s), RH rotation is positive
		Vector3f    gyro_integral{}; ///< measured angular rate of the inertial frame about the body axes obtained from rate gyro measurements (rad/s), RH rotation is positive
		float       range_m{};
		uint8_t     flow_quality{};   ///< quality indicator between 0 and 255
	};

	estimator_aid_source3d_s _aid_src_optical_flow_velocity{};
	RingBuffer<OpticalFlowSample> _ringbuffer{20}; // TODO: size with _obs_buffer_length and actual publication rate
	uint64_t _time_last_buffer_push{0};

	enum class State {
		stopped,
		starting,
		active,
	};
	int kFlowInstance{0};

	State _state{State::stopped};
	MountingType _mounting_type{MountingType::CUSTOM};

	float _test_ratio_filtered{INFINITY};

	matrix::Vector2f _vel_ne_innovation{};
	matrix::Vector2f _vel_ne_test_ratio{};

	static constexpr float _kSensorLpfTimeConstant = 0.09f;
	AlphaFilter<Vector2f> _flow_sensor_vel_lpf{0.01, _kSensorLpfTimeConstant}; ///< filtered velocity from corrected flow measurement (body frame)(m/s)
	AlphaFilter<Vector2f> _flow_body_vel_lpf{0.01, _kSensorLpfTimeConstant}; ///< filtered velocity from corrected flow measurement (body frame)(m/s)
	uint32_t _flow_counter{0};                      ///< number of flow samples read for initialization

	math::WelfordMeanVector<float, 2> _flow_mean{};
	math::WelfordMeanVector<float, 2> _flow_sensor_vel_mean{};

	bool subOpticalInstanceSet{false};
	bool subDistanceInstanceSet{false};

	bool _fused_flow_x{false};
	bool _fused_flow_y{false};

	static constexpr float kHealthTimeConstant = 1.0f;  // 1 second filter
    static constexpr float kMaxTestRatioThreshold = 1.0f;  // Innovation gate threshold
    static constexpr float kMinFusionRateThreshold = 0.5f;  // Minimum 50% fusion rate
    static constexpr uint64_t kHealthTimeoutUs = 2000000;   // 2 seconds without good fusion

    AlphaFilter<float> _test_ratio_lpf{0.01f, kHealthTimeConstant};
    AlphaFilter<float> _fusion_rate_lpf{0.01f, kHealthTimeConstant};

    uint64_t _time_last_good_fusion{0};
    uint32_t _fusion_attempt_count{0};
    uint32_t _fusion_success_count{0};

    bool _healthy{false};

#if defined(MODULE_NAME)
	struct reset_counters_s {
		uint8_t lat_lon{};
	};
	reset_counters_s _reset_counters{};

	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_optical_flow_velocity_pub{ORB_ID(estimator_aid_src_optical_flow_velocity)};
	uORB::PublicationMulti<vehicle_optical_flow_vel_s> _estimator_optical_flow_velocity_vel_pub{ORB_ID(estimator_optical_flow_velocity_vel)};

	uORB::Subscription _sensor_optical_flow_sub{ORB_ID(sensor_optical_flow)};
	uORB::Subscription _distance_sensor_sub{ORB_ID(distance_sensor)};

 #endif // MODULE_NAME
 };

 #endif // CONFIG_EKF2_OPTICAL_FLOW_VELOCITY

 #endif // !EKF_OPTICAL_FLOW_VELOCITY_HPP
