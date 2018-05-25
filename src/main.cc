#include <rr/mavros_util.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <string>
#include <thread>
#include <type_traits>


auto
arbitrary_setpoints() {
  rr::mavros_util::target_position target_position{};
  target_position.header.frame_id = "base_link";
  // We will ignore everything except for YAW_RATE and velocities in X, Y, and Z
  target_position.type_mask = rr::mavros_util::ignore_all_but(
      mavros_msgs::PositionTarget::IGNORE_YAW_RATE
    | mavros_msgs::PositionTarget::IGNORE_PX
    | mavros_msgs::PositionTarget::IGNORE_PY
    | mavros_msgs::PositionTarget::IGNORE_PZ
  );
  target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

  // Set the velocities we want (it is in m/s)
  // On the real drone send a minimum of 0.3 m/s in x/y (except when you send 0.0 m/s).
  // If the velocity is lower than 0.3 m/s then the UAV can move in any direction!
  target_position.position.x = 3.;
  target_position.position.y = 3.;
  target_position.position.z = 3.;

  // Set the yaw rate (it is in rad/s)
  target_position.yaw_rate = 2.f;

  target_position.header.stamp = ros::Time::now();
  return target_position;
}

int main(int argc, char **argv) {
  using namespace ::rr;
  using namespace ::rr::mavros_util;
  std::this_thread::sleep_for(std::chrono::seconds{5});

  ros::init(argc, argv, "quadrotor_controller");
  ros::NodeHandle nh;
  const std::uint32_t num_cores = std::thread::hardware_concurrency();
  auto spinner = ros::AsyncSpinner{num_cores};
  spinner.start();

  auto mavros = std::make_shared<mavros_adapter>(nh);
  mavros->arm();
  mavros->set_mode(px4::operating_mode::OFFBOARD);
  mavros->set_setpoints(arbitrary_setpoints());

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  auto setpoints = arbitrary_setpoints();

  while(ros::ok()) {
//    if (mavros->mode() != px4::operating_mode::OFFBOARD) {
//      auto success = mavros->enable_current_mode()
//        == mavros_adapter::server_response::success;
//      ROS_INFO_COND(success, "Offboard enabled");
//    } else {
//      if (!mavros->state().armed) {
//        auto success = mavros->enable_current_arming_command()
//          == mavros_adapter::server_response::success;
//        ROS_INFO_COND(success, "Vehicle armed");
//      }
//    }
//    if (mavros->mode() != px4::operating_mode::OFFBOARD) {
//      auto success = mavros->set_mode(px4::operating_mode::OFFBOARD)
//        == mavros_adapter::server_response::success;
//      ROS_INFO_COND(success, "Offboard enabled");
//    } else {
//      if (!mavros->state().armed) {
//        auto success = mavros->arm()
//          == mavros_adapter::server_response::success;
//        ROS_INFO_COND(success, "Vehicle armed");
//      }
//    }

    setpoints.header.stamp = ros::Time::now();
    //local_pos_pub.publish(setpoints);
    rate.sleep();
    //ros::spinOnce();
  }

  ros::waitForShutdown();

  return 0;
}
