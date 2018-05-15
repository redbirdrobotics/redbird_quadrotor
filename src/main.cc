#include <rr/mavros_util/mavros_util.hpp>

#include <boost/functional.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <cstddef>
#include <string>
#include <type_traits>

auto
arbitrary_setpoint() {
  rr::mavros_util::target_position target_position{};

  target_position.header.stamp = ros::Time::now();
  target_position.header.frame_id = "base_link";

  target_position.type_mask = rr::mavros_util::ignore_all_but(
      mavros_msgs::PositionTarget::IGNORE_YAW_RATE
    | mavros_msgs::PositionTarget::IGNORE_PX
    | mavros_msgs::PositionTarget::IGNORE_PY
    | mavros_msgs::PositionTarget::IGNORE_PZ
  );

  target_position.position.x = -3.;
  target_position.position.y = -3.;
  target_position.position.z = 3.;
  target_position.yaw_rate = 2.f;

  return target_position;
}

int main(int argc, char **argv) {
  using namespace rr::mavros_util;

  ros::init(argc, argv, "quadrotor_controller");
  ros::NodeHandle nh;

  auto mavros = std::make_shared<mavros_mode_adapter>(nh);

  ros::Publisher setpoint_publisher = nh.advertise<rr::mavros_util::target_position>
    (rr::mavros_util::kMavrosSetpointLocalRawId, 10);

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  ROS_INFO("Awaiting FCU connection");
  while(ros::ok() && !mavros->state().connected) {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("FCU connected");

  // Ensure offboard enabled.
  const auto* offboard_id = "OFFBOARD";
  if (mavros->state().mode != offboard_id) {
    auto success = mavros->set_custom_mode(offboard_id)
      == mavros_mode_adapter::server_response::success;
    ROS_INFO_COND(success, "Offboard enabled");
  }

  // Ensure vehicle is "armed".
  if (!mavros->state().armed) {
    auto success = mavros->arm()
      == mavros_mode_adapter::server_response::success;
    ROS_INFO_COND(success, "Vehicle armed");
  }

  while(ros::ok()) {
    setpoint_publisher.publish(arbitrary_setpoint());
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
