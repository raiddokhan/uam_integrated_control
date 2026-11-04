#ifndef TORQUE_SETPOINT_DEBUG_HPP
#define TORQUE_SETPOINT_DEBUG_HPP

#include <uORB/topics/vehicle_torque_setpoint.h>

class MavlinkStreamTorqueSetpointDebug : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamTorqueSetpointDebug(mavlink); }
	static constexpr const char *get_name_static() { return "TORQUE_SETPOINT_DEBUG"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY; }
	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }
	unsigned get_size() override { return MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES; }

private:
	explicit MavlinkStreamTorqueSetpointDebug(Mavlink *mavlink) : MavlinkStream(mavlink) {}
	uORB::Subscription _sub{ORB_ID(vehicle_torque_setpoint)};

	bool send() override
	{
		vehicle_torque_setpoint_s msg;

		if (_sub.update(&msg)) {
			mavlink_debug_float_array_t out{};
			out.time_usec = msg.timestamp;
			out.array_id = 43;                              // darf nicht identisch mit 42 (SENDE-KANAL)
			strncpy(out.name, "torque_dbg", sizeof(out.name));

			out.data[0] = msg.xyz[1];             // Y-Wert von Vehicle_Torque_Setpoint

			mavlink_msg_debug_float_array_send_struct(_mavlink->get_channel(), &out);
			return true;
		}

	return false;
	}
};
#endif
