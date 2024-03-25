#pragma once

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_air_data.h>

using namespace time_literals;

namespace sensors
{

template <typename sensorsData>
class FakeStuckSensors final: public ModuleParams, public px4::ScheduledWorkItem
{
public:
	FakeStuckSensors(ORB_ID id):
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
		_sensor_pub(id),
		_sensor_sub(id) {
		_sensor_pub.advertise();
	}

	~FakeStuckSensors() override {
		Stop();
		perf_free(_cycle_perf);
	}

	bool Start() {
		ScheduleNow();
		return true;
	}

	void Stop() {
		Deinit();
	}

	void SetUsing(bool use) {
		_use = use;
	}
private:
	void Run() override {
		perf_begin(_cycle_perf);

		while (_sensor_sub.update(&_first_init)){
			_first_init= true;
		}

		if (_use && _first_init) {
			_sensor_pub.publish(_last_output);
		}

		ScheduleDelayed(300_ms); // backup schedule

		perf_end(_cycle_perf);
	}

	uORB::Publication<sensorsData> _sensor_pub{};
	uORB::Subscription _sensor_sub{};

	bool _use{};
	bool _first_init{};
	sensorsData _last_output{};
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
};

}

/**
 * @brief
 *
 * This class tracks cmd vehicale command \ref VEHICLE_CMD_INJECT_FAILURE and provide information about current state.
 * It is expected to work only in HITL mode.
 *
 * @note
 * Supported message:
 * \ref FAILURE_UNIT_SENSOR_GPS
 * \ref FAILURE_UNIT_SENSOR_MAG
 * \ref FAILURE_UNIT_SENSOR_BARO
 *
 * Supported commands:
 * \ref FAILURE_TYPE_OK
 * \ref FAILURE_TYPE_FAIL
 * \ref FAILURE_TYPE_STUCK(only \ref FAILURE_UNIT_SENSOR_GPS and \ref FAILURE_UNIT_SENSOR_BARO)
 */
class FailureDetectorHITL final
{
public:
	FailureDetectorHITL(bool hil_enabled);
	bool update();
	void updateFakeSensors();

	bool isGpsBlocked() const;
	bool isBaroBlocked() const;
	bool isMagBlocked() const;

private:
	enum class FailureStatus {
		ok,
		off,
		stuck
	};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};

	FailureStatus _gps{FailureStatus::ok};
	FailureStatus _baro{FailureStatus::ok};
	FailureStatus _mag{FailureStatus::ok};

	sensors::FakeStuckSensors<vehicle_air_data_s> _fake_baro_publisher{ORB_ID::vehicle_air_data};
	sensors::FakeStuckSensors<sensor_gps_s> _fake_gps_publisher{ORB_ID::vehicle_gps_position};
};
