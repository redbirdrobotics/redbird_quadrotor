#include "flights.hpp"

#include <rr/mavros_util/mavros_util.hpp>

#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>

#include <chrono>
#include <functional>
#include <utility>


namespace rr {
namespace quadrotor {
namespace flightsys {

flight::~flight() {};

using target_position = mavros_msgs::PositionTarget;


struct takeoff_and_land_config {
  decltype(target_position::position.z) target_altitude;
  std::chrono::milliseconds publish_period;
};

class takeoff_and_land final : public flight {
 private:
  std::function<void(mavros_util::target_position)> target_position_publisher;
  takeoff_and_land_config config;

 public:
  template <typename Config>
  takeoff_and_land(Config&& c, ros::NodeHandle nh)
    : config(std::forward<Config>(c))
    , position_publisher(nh.advertise<target_position>(
                           kMavrosSetpointLocalRawId,
                           kMavrosSetpointLocalQueueSize
                         ))
  {}

  void fly(canceller c) {
    while (c() == cancellation::none) {
      target_position target_position{};

      target_position.header.stamp = ros::Time::now();
      target_position.header.frame_id = "base_link";

      // ignore all but YAW_RATE and velocities x, y, and z
      target_position.type_mask
          = ::rr::mavros_util::ignore_all_but(target_position::IGNORE_PZ);
      target_position.position.z = 2.;

//      ros::Rate r(20); // 20 hz
//      while (ros::ok()) {
//        setpoint_raw_publisher.publish(target_position);
//        r.sleep();
//      }
    }
  }
};

} // namespace flightsys
} // namespace quadrotor
} // namespace rr
