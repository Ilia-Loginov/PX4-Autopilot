#include "failure_detector_HITL.hpp"

#include <px4_platform_common/log.h>


FailureDetectorHITL::FailureDetectorHITL(bool hil_enabled) {

	PX4_WARN("RUN FailureDetectorHITL %d", hil_enabled);
	if(!hil_enabled)
		_vehicle_command_sub.unsubscribe();
	else {
		_fake_baro_publisher.Start();
		_fake_gps_publisher.Start();
	}
}
bool FailureDetectorHITL::update() {

	//sensors::FakeStuckSensors<sensor_gps_s> _fake_gps_publisher(ORB_ID(vehicle_gps_position));
	//sensors::FakeStuckSensors<sensor_gps_s> _fake_gps_publisher(ORB_ID::vehicle_gps_position);

	vehicle_command_s vehicle_command;
	bool update = false;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		if (vehicle_command.command != vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE) {
			continue;
		}

		bool handled = false;
		bool supported = false;

		const int failure_unit = static_cast<int>(vehicle_command.param1 + 0.5f);
		const int failure_type = static_cast<int>(vehicle_command.param2 + 0.5f);

		PX4_INFO("Sensor failure detector caught new injection");
		if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_GPS) {
			handled = true;
			if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, gps ok");
				_gps = FailureStatus::OK;
				supported = true;

			}
			else if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, gps off");
				supported = true;
				_gps = FailureStatus::OFF;
			}
			else if (failure_type == vehicle_command_s::FAILURE_TYPE_STUCK) {
				PX4_WARN("CMD_INJECT_FAILURE, gps stuck");
				_gps = FailureStatus::STUCK;
				supported = true;
			}
		}
		else if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_BARO) {
			handled = true;
			if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, baro ok");
				_baro = FailureStatus::OK;
				supported = true;
			}
			else if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, baro off");
				_baro = FailureStatus::OFF;
				supported = true;
			}
			else if (failure_type == vehicle_command_s::FAILURE_TYPE_STUCK) {
				PX4_WARN("CMD_INJECT_FAILURE, baro stuck");
				_baro = FailureStatus::STUCK;
				supported = true;
			}
		}
		else if (failure_unit == vehicle_command_s::FAILURE_UNIT_SENSOR_MAG) {
			handled = true;
			if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, mag ok");
				_mag = FailureStatus::OK;
				supported = true;
			}
			else if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, mag off");
				_mag = FailureStatus::OFF;
				supported = true;
			}
		}

		if (handled) {
			vehicle_command_ack_s ack{};
			ack.command = vehicle_command.command;
			ack.from_external = false;
			ack.result = supported ?
				vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED :
				vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
			ack.timestamp = hrt_absolute_time();
			_command_ack_pub.publish(ack);
		}
		update |= supported;
	}
	return update;
}

bool FailureDetectorHITL::isGpsBlocked() const {
	return FailureStatus::OFF == _gps || FailureStatus::STUCK == _gps;
}

bool FailureDetectorHITL::isBaroBlocked() const {
	return FailureStatus::OFF == _baro || FailureStatus::STUCK == _baro;
}

bool FailureDetectorHITL::isMagBlocked() const {
	return FailureStatus::OFF == _mag || FailureStatus::STUCK == _mag;
}

void FailureDetectorHITL::updateFakeSensors() {
	_fake_gps_publisher.SetUsing(_gps == FailureStatus::STUCK);
	_fake_baro_publisher.SetUsing(_baro == FailureStatus::STUCK);
}
