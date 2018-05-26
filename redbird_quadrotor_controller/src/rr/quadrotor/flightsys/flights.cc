#include "flights.hpp"

#include <rr/mavros_util/mavros_util.hpp>

#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <utility>


namespace rr {
namespace quadrotor {
namespace flightsys {

flight::~flight() {};

namespace {
template <typename Duration>
constexpr auto to_hz(Duration&& d) {
  auto seconds = std::chrono::duration_cast<std::chrono::seconds>(
        std::forward<Duration>(d));
  return 1. / seconds.count();
}
} // namespace


struct takeoff_and_land_config {
  decltype(mavros_util::target_position::position.z) target_altitude;
  std::function<void(mavros_util::target_position)> publish_setpoint_position;
  std::chrono::milliseconds publish_period;
};

auto
takeoff_and_land(takeoff_and_land_config& config, canceller cancel_status) {
  ros::ServiceClient c;
  auto target_position = mavros_util::target_position{};
  target_position.header.stamp = ros::Time::now();
  target_position.header.frame_id = "base_link";
  target_position.position.z = 2.;
  target_position.type_mask
      = mavros_util::ignore_all_but(mavros_util::target_position::IGNORE_PZ);

  auto rate = ros::Rate(to_hz(config.publish_period));

  while (cancel_status() == cancellation::none) {
    config.publish_setpoint_position(target_position);
    rate.sleep();
  }
}

} // namespace flightsys
} // namespace quadrotor
} // namespace rr
