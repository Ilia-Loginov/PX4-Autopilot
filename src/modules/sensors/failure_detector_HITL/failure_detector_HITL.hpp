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

/**
 * @brief FailureDetectorHITL class
 *
 * This class tracks cmd vehicle command \ref VEHICLE_CMD_INJECT_FAILURE and provide information about current state.
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

#if defined(CONFIG_SENSORS_VEHICLE_GPS_POSITION)
	bool isGpsBlocked() const;
	bool isGpsStuck() const;
#endif // CONFIG_SENSORS_VEHICLE_GPS_POSITION

#if defined(CONFIG_SENSORS_VEHICLE_AIR_DATA)
	bool isBaroBlocked() const;
	bool isBaroStuck() const;
#endif // CONFIG_SENSORS_VEHICLE_AIR_DATA

#if defined(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)
	bool isMagBlocked() const;
#endif // CONFIG_SENSORS_VEHICLE_MAGNETOMETER

private:
	enum class FailureStatus {
		ok,
		off,
		stuck
	};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};

#if defined(CONFIG_SENSORS_VEHICLE_GPS_POSITION)
	FailureStatus _gps{FailureStatus::ok};
#endif // CONFIG_SENSORS_VEHICLE_GPS_POSITION

#if defined(CONFIG_SENSORS_VEHICLE_AIR_DATA)
	FailureStatus _baro{FailureStatus::ok};
#endif // CONFIG_SENSORS_VEHICLE_AIR_DATA

#if defined(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)
	FailureStatus _mag{FailureStatus::ok};
#endif // CONFIG_SENSORS_VEHICLE_MAGNETOMETER
};


namespace sensors
{

/**
 * @brief FakeStuckSensor class
 *
 * This class implements the logic for simulating a stuck sensor
 *
 * @tparam sensorsData Type of the sensor data to be published.
 */
template <typename sensorsData>
class FakeStuckSensor final: public ModuleParams, public px4::ScheduledWorkItem
{
public:
	/**
	 * @brief Constructor for FakeStuckSensor class.
	 * @param id ORB ID for the sensor data topic.
	 */
	FakeStuckSensor(ORB_ID id):
		ModuleParams(nullptr),
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
		_sensor_pub(id),
		_sensor_sub(id) {
		_sensor_pub.advertise();
	}

	~FakeStuckSensor() override {
		stop();
		perf_free(_cycle_perf);
	}

	bool start() {
		ScheduleNow();
		return true;
	}

	void stop() {
		Deinit();
	}

	void setEnabled(bool enable) {
		_enable = enable;
	}
private:
	void Run() override {
		perf_begin(_cycle_perf);

		while (_sensor_sub.update(&_last_output)){
			_first_init= true;
		}

		if (_enable && _first_init) {
			_sensor_pub.publish(_last_output);
		}

		ScheduleDelayed(300_ms); // backup schedule

		perf_end(_cycle_perf);
	}

	uORB::Publication<sensorsData> _sensor_pub{};
	uORB::Subscription _sensor_sub{};

	bool _enable{};
	bool _first_init{}; /**< Flag indicating whether the sensor has been initialized for the first time. */
	sensorsData _last_output{};
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
};

}

/**
 * @brief FakeSensors class
 *
 * This class represents a collection of fake sensors used for simulation purposes.
 */
class FakeSensors final {
public:
	FakeSensors(bool hil_enabled);

	/**
	 * @brief Updates states of the fake sensors with data from the failure detector.
	 */
	void update(const FailureDetectorHITL& detector);
private:

#if defined(CONFIG_SENSORS_VEHICLE_AIR_DATA)
	sensors::FakeStuckSensor<vehicle_air_data_s> _fake_baro_publisher{ORB_ID::vehicle_air_data};
#endif // CONFIG_SENSORS_VEHICLE_AIR_DATA

#if defined(CONFIG_SENSORS_VEHICLE_GPS_POSITION)
	sensors::FakeStuckSensor<sensor_gps_s> _fake_gps_publisher{ORB_ID::vehicle_gps_position};
#endif // CONFIG_SENSORS_VEHICLE_GPS_POSITION
};
