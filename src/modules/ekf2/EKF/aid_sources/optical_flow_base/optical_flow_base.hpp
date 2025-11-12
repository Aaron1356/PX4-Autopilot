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

 #ifndef EKF_OPTICAL_FLOW_BASE_HPP
 #define EKF_OPTICAL_FLOW_BASE_HPP

 #include "../../common.h"
 #include "../../RingBuffer.h"

 #include <lib/mathlib/math/WelfordMeanVector.hpp>

 #if defined(CONFIG_EKF2_OPTICAL_FLOW_BASE) && defined(MODULE_NAME)

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

 class OpticalFlowBase : public ModuleParams
 {
 public:
	 // Define sensor mounting type
	 enum class MountingType {
		 FORWARD,  // Facing forward (default)
		 SIDEWAYS, // Facing sideways (right)
		 UPWARD,   // Facing upward
		 DOWNWARD, // Facing downward
		 CUSTOM    // Custom orientation defined by parameters
	 };

	 OpticalFlowBase(int flowInstance =0,MountingType type = MountingType::CUSTOM) :
		ModuleParams(nullptr),
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

		_estimator_aid_src_optical_flow_base_pub.advertise();
		printf("Starting Optical Flow instance: %d\n", flowInstance);
		updateParameters();
	 }

	 ~OpticalFlowBase() = default;

	 static OpticalFlowBase* create_instance(int instance_num){
		param_t control_bit = PARAM_INVALID;
		char param_name[17];
		int32_t tmp_int;
		snprintf(param_name, sizeof(param_name), "EKF2_OFV%d_CTRL", instance_num);
		control_bit = param_find(param_name);
		param_get(control_bit, &tmp_int);
		if(tmp_int){
			return new OpticalFlowBase(instance_num);
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

		printf("Doing something here in Velocity Optical Flow\n");
		if (param_get(_param_ekf2_ds_id, &tmp_int) == PX4_OK) {
			getDistanceSensorInstance(tmp_int);
			// distance_sensor_s topic;
			// _ekf2_ds_id = tmp_int;
			// uORB::SubscriptionMultiArray<distance_sensor_s> distance_sensor_subs{ORB_ID::distance_sensor};
			// printf("Number of Sensors: %d\n", distance_sensor_subs.size());
			// for(int i = 0; i < distance_sensor_subs.size(); i++){
			// 	if(distance_sensor_subs[i].copy(&topic)){
			// 		printf("Distance Sensor Looking For: %d\n", _ekf2_ds_id);
			// 		if((int)((topic.device_id >> 8) & 0xFF) == (int)_ekf2_ds_id)
			// 		{
			// 			printf("Distance Sensor Looking at: %d\n", (int)((topic.device_id >> 8) & 0xFF));
			// 			_distance_sensor_sub.ChangeInstance(i);

			// 			break;
			// 		}
			// 	}
			// 	printf("Device id @ %d : %d\n", i, (int)((topic.device_id >> 8) & 0xFF));

			// }
		}

		if (param_get(_param_ekf2_of_id, &tmp_int) == PX4_OK) {
			getOpticalFlowInstance(tmp_int);
			// sensor_optical_flow_s topic;
			// _ekf2_of_id = tmp_int;
			// uORB::SubscriptionMultiArray<sensor_optical_flow_s> optical_flow_subs{ORB_ID::sensor_optical_flow};
			// for(int i = 0; i < optical_flow_subs.size(); i++){
			// 	if(optical_flow_subs[i].copy(&topic)){
			// 		printf("Optical Flow Sensor Looking for: %d\n", _ekf2_of_id);
			// 		if((int)((topic.device_id >> 8) & 0xFF) == (int)_ekf2_of_id)
			// 		{
			// 			printf("Optical Flow Sensor Looking at: %d\n", (int)((topic.device_id >> 8) & 0xFF));
			// 			_sensor_optical_flow_sub.ChangeInstance(i);
			// 			subInstanceSet = true;
			// 		}
			// 	}
			// }

		}
		updateParams();
	 }

	 void getDistanceSensorInstance(int32_t tmp_int){
		distance_sensor_s topic;
		_ekf2_ds_id = tmp_int;
		uORB::SubscriptionMultiArray<distance_sensor_s> distance_sensor_subs{ORB_ID::distance_sensor};
		printf("Number of Sensors: %d\n", distance_sensor_subs.size());
		for(int i = 0; i < distance_sensor_subs.size(); i++){
			if(distance_sensor_subs[i].copy(&topic)){
				// printf("Distance Sensor Looking For: %d\n", _ekf2_ds_id);
				if((int)((topic.device_id >> 8) & 0xFF) == (int)_ekf2_ds_id)
				{
					printf("Distance Sensor Looking at: %d\n", (int)((topic.device_id >> 8) & 0xFF));
					_distance_sensor_sub.ChangeInstance(i);
					subDistanceInstanceSet = true;
					break;
				}
			}
			printf("Device id @ %d : %d\n", i, (int)((topic.device_id >> 8) & 0xFF));

		}
	 }

	 void getOpticalFlowInstance(int32_t tmp_int){
		sensor_optical_flow_s topic;
		_ekf2_of_id = tmp_int;
		uORB::SubscriptionMultiArray<sensor_optical_flow_s> optical_flow_subs{ORB_ID::sensor_optical_flow};
		for(int i = 0; i < optical_flow_subs.size(); i++){
			if(optical_flow_subs[i].copy(&topic)){
				// printf("Optical Flow Sensor Looking for: %d\n", _ekf2_of_id);
				if((int)((topic.device_id >> 8) & 0xFF) == (int)_ekf2_of_id)
				{
					printf("Optical Flow Sensor Looking at: %d\n", (int)((topic.device_id >> 8) & 0xFF));
					_sensor_optical_flow_sub.ChangeInstance(i);
					subOpticalInstanceSet = true;
					break;
				}
			}
		}
	 }

	 const matrix::Vector2f &test_ratio() const { return _vel_ne_test_ratio; }
	 const matrix::Vector2f &innovation() const { return _vel_ne_innovation; }

	 float test_ratio_filtered() const { return _test_ratio_filtered; }

	 void setMountingType(MountingType type) { _mounting_type = type; }

	 static constexpr uint8_t getInstance() { return 0; }

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

	bool isTimedOut(uint64_t last_sensor_timestamp, uint64_t time_delayed_us, uint64_t timeout_period) const
	{
		return (last_sensor_timestamp == 0) || (last_sensor_timestamp + timeout_period < time_delayed_us);
	}

	/**
	 * @brief Get velocity update mask based on mounting type and angles
	 * This determines which velocity components (x,y,z) should be updated
	 * @return Bit mask with bits set for components that should be updated
	 */
	uint8_t getVelocityUpdateMask() const;

	/**
	 * @brief Transform optical flow measurements to body frame velocity
	 * @param flow_compensated_xy_rad Compensated flow measurements
	 * @param range_m Range measurement in meters
	 * @param flow_dt Flow integration time in seconds
	 * @return Velocity in body frame
	 */
	Vector3f flowToBodyVelocity(const Vector2f &flow_compensated_xy_rad, float range_m, float flow_dt) const;

	/**
	 * Calculate the rotation matrix from sensor to body frame
	 * Accounts for orthogonal and non-orthogonal mounting
	 */
	matrix::Dcmf calculateSensorToBodyRotation() const;

	struct OpticalFlowSample {
		uint64_t    time_us{};   ///< timestamp of the integration period midpoint (uSec)
		float       flow_dt{};
		Vector2f    flow_xy_rad{}; ///< measured angular rate of the image about the X and Y body axes (rad/s), RH rotation is positive
		Vector3f    gyro_integral{}; ///< measured angular rate of the inertial frame about the body axes obtained from rate gyro measurements (rad/s), RH rotation is positive
		float       range_m{};
		uint8_t     flow_quality{};   ///< quality indicator between 0 and 255
	};

	estimator_aid_source3d_s _aid_src_optical_flow_base{};
	RingBuffer<OpticalFlowSample> _ringbuffer{20}; // TODO: size with _obs_buffer_length and actual publication rate
	uint64_t _time_last_buffer_push{0};

	enum class State {
		stopped,
		starting,
		active,
	};

	State _state{State::stopped};
	MountingType _mounting_type{MountingType::CUSTOM};

	float _test_ratio_filtered{INFINITY};

	matrix::Vector3f _flow_gyro_bias{};
	matrix::Vector2f _vel_ne_innovation{};
	matrix::Vector2f _vel_ne_test_ratio{};

	Vector2f _flow_vel_body{};

	static constexpr float _kSensorLpfTimeConstant = 0.09f;
	AlphaFilter<Vector2f> _flow_sensor_vel_lpf{0.01, _kSensorLpfTimeConstant}; ///< filtered velocity from corrected flow measurement (body frame)(m/s)
	AlphaFilter<Vector2f> _flow_body_vel_lpf{0.01, _kSensorLpfTimeConstant}; ///< filtered velocity from corrected flow measurement (body frame)(m/s)
	uint32_t _flow_counter{0};                      ///< number of flow samples read for initialization

	math::WelfordMeanVector<float, 2> _flow_mean{};
	math::WelfordMeanVector<float, 2> _flow_sensor_vel_mean{};

	bool subOpticalInstanceSet = false;
	bool subDistanceInstanceSet = false;

#if defined(MODULE_NAME)
	struct reset_counters_s {
		uint8_t lat_lon{};
	};
	reset_counters_s _reset_counters{};

	uORB::PublicationMulti<estimator_aid_source3d_s> _estimator_aid_src_optical_flow_base_pub{ORB_ID(estimator_aid_src_optical_flow_base)};
	uORB::PublicationMulti<vehicle_optical_flow_vel_s> _estimator_optical_flow_base_vel_pub{ORB_ID(estimator_optical_flow_base_vel)};

	const uint8_t kFlowInstance = 0;

	uORB::Subscription _sensor_optical_flow_sub{ORB_ID(sensor_optical_flow)};
	uORB::Subscription _distance_sensor_sub{ORB_ID(distance_sensor)};

 #endif // MODULE_NAME
 };

 #endif // CONFIG_EKF2_OPTICAL_FLOW_BASE

 #endif // !EKF_OPTICAL_FLOW_BASE_HPP
