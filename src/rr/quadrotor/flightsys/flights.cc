#include "flights.hpp"

#include <mavros_msgs/PositionTarget.h>

#include <utility>


namespace rr {
namespace quadrotor {
namespace flightsys {

flight::~flight() {};

using target_position = mavros_msgs::PositionTarget;


struct takeoff_and_land_config {
  decltype(target_position::position.z) target_altitude;
};

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

/*
 * @breif
 *    Create a bitmask which directs mavros to ignore all
 *    mavros_msgs::PositionTarget fields EXCEPT those in the provided mask.
 */
constexpr auto
ignore_all_but(decltype(kIgnoreAll) mask) {
  // clear the bits which are set in mask. (i.e. toggle the mask's set bits)
  return kIgnoreAll ^ mask;
}

class takeoff_and_land final : public flight {
 private:
  takeoff_and_land_config config;

 public:
  template <typename Config>
  takeoff_and_land(Config&& c)
    : config(std::forward<Config>(c))
  {}

  void fly(canceller c) {
    while (c() == cancellation::none) {
      target_position target_position{};

      target_position.header.stamp = ros::Time::now();
      target_position.header.frame_id = "base_link";

      // ignore all but YAW_RATE and velocities x, y, and z
      target_position.type_mask =
          target_position::IGNORE_AFX
        | target_position::IGNORE_AFY
        | target_position::IGNORE_AFZ
        | target_position::IGNORE_PX
        | target_position::IGNORE_PY
        | target_position::IGNORE_VX
        | target_position::IGNORE_VY
        | target_position::IGNORE_VZ
        | target_position::FORCE
        | target_position::IGNORE_YAW;
      // This makes it so our velocities are sent in the UAV frame.
      // If you use FRAME_LOCAL_NED then it will be in map frame
      target_position.coordinate_frame = target_position::FRAME_BODY_NED;

      // Set the velocities we want (it is in m/s)
      // On the real drone send a minimum of 0.3 m/s in x/y (except when you send 0.0 m/s).
      // If the velocity is lower than 0.3 m/s then the UAV can move in any direction!
      target_position.velocity.x = 0.3;
      target_position.position.x = 0.;
      target_position.position.y = 0.;
      target_position.position.z = 2.;
      target_position.velocity.y = 0.0;
      target_position.velocity.z = 0.1;
      // Set the yaw rate (it is in rad/s)
      target_position.yaw_rate = 0.1;

      ros::Rate r(20); // 20 hz
      while (ros::ok()) {
        setpoint_raw_publisher.publish(target_position);
        r.sleep();
      }
    }
  }
}

} // namespace flightsys
} // namespace quadrotor
} // namespace rr

#endif // RR_QUADROTOR_FLIGHTSYS_FLIGHTS_
