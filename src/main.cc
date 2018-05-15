// This code below taken directly from PX4 example documentation

#include <rr/mavros_util/mavros_util.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <functional>

int main(int argc, char **argv) {
  ros::init(argc, argv, "quadrotor_controller");
  ros::NodeHandle nh;

  mavros_msgs::State mavros_state;
  auto set_state_callback = [&](const mavros_msgs::State::ConstPtr& msg) {
    mavros_state = *msg;
  };

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    ("mavros/state", 10, set_state_callback);
  ros::Publisher local_pos_pub = nh.advertise<rr::mavros_util::target_position>
    (rr::mavros_util::kMavrosSetpointLocalRawId, 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    ("mavros/set_mode");

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  // wait for FCU connection
  while(ros::ok() && !mavros_state.connected) {
    ros::spinOnce();
    rate.sleep();
  }
  rr::mavros_util::target_position target_position{};
  target_position.header.stamp = ros::Time::now();
  target_position.header.frame_id = "base_link";
  // We will ignore everything except for YAW_RATE and velocities in X, Y, and Z
  target_position.type_mask = rr::mavros_util::ignore_all_but(
      mavros_msgs::PositionTarget::IGNORE_YAW_RATE
    | mavros_msgs::PositionTarget::IGNORE_VX
    | mavros_msgs::PositionTarget::IGNORE_VY
    | mavros_msgs::PositionTarget::IGNORE_VZ
  );
  // This makes it so our velocities are sent in the UAV frame.
  // If you use FRAME_LOCAL_NED then it will be in map frame
  target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

  // Set the velocities we want (it is in m/s)
  // On the real drone send a minimum of 0.3 m/s in x/y (except when you send 0.0 m/s).
  // If the velocity is lower than 0.3 m/s then the UAV can move in any direction!
  target_position.velocity.x = 0.;
  target_position.velocity.y = 0.;
  target_position.velocity.z = 1.;

  // Set the yaw rate (it is in rad/s)
  target_position.yaw_rate = 2.f;

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  while(ros::ok()) {
    if (mavros_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
      if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent )
        ROS_INFO("Offboard enabled");

      last_request = ros::Time::now();
    } else {
      if (!mavros_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
          ROS_INFO("Vehicle armed");

        last_request = ros::Time::now();
      }
    }

    local_pos_pub.publish(target_position);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
