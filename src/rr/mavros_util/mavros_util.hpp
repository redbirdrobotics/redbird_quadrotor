#ifndef RR_MAVROS_UTIL_HPP_
#define RR_MAVROS_UTIL_HPP_

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <string>

namespace rr {
namespace mavros_util {

using target_position = mavros_msgs::PositionTarget;

extern const std::string kMavrosSetpointLocalRawId;

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

inline auto
fully_ignored_mavros_setpoint_position() {
  auto setpoint_position_msg = target_position{};
  setpoint_position_msg.type_mask = kIgnoreAll;
  return setpoint_position_msg;
}

} // namespace mavros_util
} // namespace rr

#endif // RR_MAVROS_UTIL_HPP_
