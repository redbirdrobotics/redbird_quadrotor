#ifndef RR_MAVROS_UTIL_HPP_
#define RR_MAVROS_UTIL_HPP_

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

#include <string>

namespace rr {
namespace mavros_util {

/**
 * @brief
 *    Preferred Mavros API for controlling quadrotor.
 *
 * @note
 *    Units in meters, m/s, radians, radians/s
 */
using target_position = mavros_msgs::PositionTarget;


constexpr const char* const
  kMavrosSetpointLocalRawId = "mavros/setpoint_raw/local";

static constexpr const char* const
  kMavrosArmingService_Id = "mavros/cmd/arming";

static constexpr const char* const
  kMavrosSetmodeService_Id = "mavros/set_mode";

static constexpr const char* const
  kMavrosStateTopic_Id = "mavros/state";


/**
 * @brief
 *    The bitmask for mavros_msgs::PositionTarget which directs mavros to
 *    ignore all fields.
 *
 * @sa
 *    mavros_msgs::PositionTarget::IGNORE_PX
 */
constexpr auto kIgnoreAll =
    target_position::IGNORE_PX
  | target_position::IGNORE_PY
  | target_position::IGNORE_PZ
  | target_position::IGNORE_VX
  | target_position::IGNORE_VY
  | target_position::IGNORE_VZ
  | target_position::IGNORE_AFX
  | target_position::IGNORE_AFY
  | target_position::IGNORE_AFZ
  | target_position::FORCE
  | target_position::IGNORE_YAW
  | target_position::IGNORE_YAW_RATE;

/**
 * @brief
 *    Create a bitmask which directs mavros to ignore all
 *    mavros_msgs::PositionTarget fields EXCEPT those in the provided mask.
 *
 * @sa
 *    mavros_msgs::PositionTarget::IGNORE_PX
 *    mavros_msgs::PositionTarget::type_mask
 *
 * @param mask
 *    A bitmask of those mavros parameters which should NOT be ignored
 *
 * @code
 *    // Tell mavros only to respond to position.x value:
 *    target_position.type_mask
 *      = ignore_all_but(mavros_msgs::PositionTarget::IGNORE_PX);
 */
constexpr auto
ignore_all_but(decltype(kIgnoreAll) mask = 0) {
  // clear the bits which are set in mask. (i.e. toggle the mask's set bits)
  return kIgnoreAll ^ mask;
}

/**
 * @brief
 *    Construct a target_position message with all ignore bitfields set. (i.e.,
 *    instructs mavros to ignore all setpoints.
 *
 * @sa
 *    ignore_all_but
 *    kIgnoreAll
 */
inline auto
fully_ignored_mavros_setpoint_position() {
  auto setpoint_position_msg = target_position{};
  setpoint_position_msg.type_mask = kIgnoreAll;
  return setpoint_position_msg;
}



// TODO: this class should know less. In particular, don't pass NodeHandle in
// c'tor.
class mavros_mode_adapter {
 public:
  enum class server_response {
   success,
   denied,
   fcu_not_connected,
   cannot_reach_server,
  };

 private:
  template <typename CallServer, typename CheckSuccess>
  server_response
  call_server(CallServer&& call_server, CheckSuccess&& success);

 public:
  mavros_mode_adapter(ros::NodeHandle& nh);

  const auto&
  state() const { return mavros_state_; }

  server_response
  arm() { return set_armed(true); }

  server_response
  disarm() { return set_armed(false); }

  server_response
  set_custom_mode(std::string custom_mode_id);

 private:
  ros::ServiceClient arming_client;
  mavros_msgs::CommandBool arm_cmd_;

  ros::ServiceClient set_mode_client;
  mavros_msgs::SetMode set_mode_cmd_;

  ros::Subscriber state_sub_;
  mavros_msgs::State mavros_state_{};

  server_response
  set_armed(bool arm);
};

} // namespace mavros_util
} // namespace rr

#endif // RR_MAVROS_UTIL_HPP_
