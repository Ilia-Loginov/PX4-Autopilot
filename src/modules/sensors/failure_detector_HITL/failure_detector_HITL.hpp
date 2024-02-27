#pragma once

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>


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
 */
class FailureDetectorHITL final
{
public:
	FailureDetectorHITL(bool hil_enabled);
	bool update();

	bool isGpsBlocked() const;
	bool isBaroBlocked() const;
	bool isMagBlocked() const;

private:
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};

	bool _gps_blocked{};
	bool _baro_blocked{};
	bool _mag_blocked{};
};
