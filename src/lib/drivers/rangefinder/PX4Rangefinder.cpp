/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "PX4Rangefinder.hpp"

#include <lib/drivers/device/Device.hpp>

PX4Rangefinder::PX4Rangefinder(const uint32_t device_id, const uint8_t device_orientation, const uint8_t device_address)
{
	_dev_address = device_address;
	set_device_id(device_id);
	set_orientation(device_orientation);
	set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
	set_mode(distance_sensor_s::MODE_UNKNOWN);

	set_device_address();

}

PX4Rangefinder::~PX4Rangefinder()
{
	_distance_sensor_pub.unadvertise();
}

void PX4Rangefinder::set_device_type(uint8_t device_type)
{
	// current DeviceStructure
	union device::Device::DeviceId device_id;
	device_id.devid = _distance_sensor_pub.get().device_id;

	// update to new device type
	device_id.devid_s.devtype = device_type;

	// copy back to report
	_distance_sensor_pub.get().device_id = device_id.devid;
}

void PX4Rangefinder::rpy_to_quaternion(const float roll, const float pitch, const float yaw, float quaternion[4])
{
	// Convert Euler angles to quaternion
	float cy = cosf(yaw * 0.5f);
	float sy = sinf(yaw * 0.5f);
	float cp = cosf(pitch * 0.5f);
	float sp = sinf(pitch * 0.5f);
	float cr = cosf(roll * 0.5f);
	float sr = sinf(roll * 0.5f);

	quaternion[0] = cr * cp * cy + sr * sp * sy; // w
	quaternion[1] = sr * cp * cy - cr * sp * sy; // x
	quaternion[2] = cr * sp * cy + sr * cp * sy; // y
	quaternion[3] = cr * cp * sy - sr * sp * cy; // z
}

void PX4Rangefinder::set_orientation_rpy(const float roll, const float pitch, const float yaw)
{
	// Convert roll, pitch, yaw to quaternion and store in the distance_sensor message
	float q[4];
	rpy_to_quaternion(roll, pitch, yaw, q);

	distance_sensor_s &report = _distance_sensor_pub.get();
	report.q[0] = q[0]; // w
	report.q[1] = q[1]; // x
	report.q[2] = q[2]; // y
	report.q[3] = q[3]; // z

	// Also set the traditional orientation field for backward compatibility
	// This is a simplified mapping and might not be perfect for all orientations
	report.orientation = distance_sensor_s::ROTATION_CUSTOM;
}

void PX4Rangefinder::set_orientation(const uint8_t device_orientation)
{
	distance_sensor_s &report = _distance_sensor_pub.get();
	report.orientation = device_orientation;

	// Set default quaternion based on the orientation
	// This is a simplified approach - in a real implementation, you might want to map
	// the standard orientations to their corresponding quaternions more accurately
	float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

	switch (device_orientation) {
		case distance_sensor_s::ROTATION_DOWNWARD_FACING:
			pitch = M_PI; // 180 degrees pitch for downward facing
			break;
		case distance_sensor_s::ROTATION_UPWARD_FACING:
			// Default orientation (0,0,0)
			break;
		case distance_sensor_s::ROTATION_FORWARD_FACING:
			pitch = -M_PI_2; // -90 degrees pitch
			break;
		case distance_sensor_s::ROTATION_BACKWARD_FACING:
			pitch = M_PI_2; // 90 degrees pitch
			break;
		case distance_sensor_s::ROTATION_LEFT_FACING:
			yaw = M_PI_2; // 90 degrees yaw
			break;
		case distance_sensor_s::ROTATION_RIGHT_FACING:
			yaw = -M_PI_2; // -90 degrees yaw
			break;
		// Add more cases as needed
		default:
			// Keep default orientation for unknown/custom orientations
			break;
	}

	// Convert the roll, pitch, yaw to quaternion
	float q[4];
	rpy_to_quaternion(roll, pitch, yaw, q);

	report.q[0] = q[0]; // w
	report.q[1] = q[1]; // x
	report.q[2] = q[2]; // y
	report.q[3] = q[3]; // z

}

void PX4Rangefinder::update(const hrt_abstime &timestamp_sample, const float distance, const int8_t quality)
{
	distance_sensor_s &report = _distance_sensor_pub.get();
	report.timestamp = timestamp_sample;
	report.current_distance = distance;
	report.signal_quality = quality;

	// if quality is unavailable (-1) set to 0 if distance is outside bounds
	if (quality < 0) {
		if ((distance < report.min_distance) || (distance > report.max_distance)) {
			report.signal_quality = 0;
		}
	}

	_distance_sensor_pub.publish(report);

}
