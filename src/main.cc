// This code below taken directly from PX4 example documentation

#include <rr/mavros_util/mavros_util.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <cstddef>
#include <functional>
#include <string>
#include <type_traits>


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
  ros::ServiceClient arming_client;

  ros::ServiceClient set_mode_client;

  static constexpr const char* const
    kMavrosArmingService_Id = "mavros/cmd/arming";

  static constexpr const char* const
    kMavrosSetmodeService_Id = "mavros/set_mode";

  static constexpr const char* const
    kMavrosStateTopic_Id = "mavros/state";

  mavros_msgs::State mavros_state_;
  mavros_msgs::CommandBool arm_cmd_;
  mavros_msgs::SetMode operating_mode_cmd_;

  template <typename CallServer, typename CheckSuccess>
  server_response
  call_server(CallServer&& call_server, CheckSuccess&& success) {
    using r = server_response;

    if (!mavros_state_.connected)
      return r::fcu_not_connected;

    auto able_to_reach_server = std::forward<CallServer>(call_server)();
    if (!able_to_reach_server)
      return r::cannot_reach_server;

    return std::forward<CheckSuccess>(success)() ? r::success : r::denied;
  }

  auto
  set_armed(bool arm) {
    arm_cmd_.request.value = arm;
    return call_server(
      [&]{ return arming_client.call(arm_cmd_); },
      [&]{ return arm_cmd_.response.success; }
    );
  }

 public:
  mavros_mode_adapter(ros::NodeHandle& nh) {
    arming_client
        = nh.serviceClient<mavros_msgs::CommandBool>(kMavrosArmingService_Id);

    set_mode_client
        = nh.serviceClient<mavros_msgs::SetMode>(kMavrosSetmodeService_Id);

    auto&& update_state_callback = [&](const mavros_msgs::State::ConstPtr& msg) {
      mavros_state_ = *msg;
    };
    const uint32_t mavros_state_queue_size = 10u;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
      kMavrosStateTopic_Id,
      mavros_state_queue_size,
      update_state_callback
    );
  }

  const auto& state() const { return mavros_state_; }
  auto arm() { return set_armed(true); }
  auto disarm() { return set_armed(false); }

  template <typename String> auto
  set_custom_mode(String&& custom_mode_id) {
    operating_mode_cmd_.request.custom_mode = std::forward<String>(custom_mode_id);
    return call_server(
      [&]{ return set_mode_client.call(operating_mode_cmd_); },
      [&]{ return operating_mode_cmd_.response.mode_sent; }
    );
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "quadrotor_controller");
  ros::NodeHandle nh;

  auto mavros = std::make_shared<mavros_mode_adapter>(nh);

  ros::Publisher local_pos_pub = nh.advertise<rr::mavros_util::target_position>
    (rr::mavros_util::kMavrosSetpointLocalRawId, 10);

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  // wait for FCU connection
  while(ros::ok() && !mavros->state().connected) {
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
    | mavros_msgs::PositionTarget::IGNORE_PX
    | mavros_msgs::PositionTarget::IGNORE_PY
    | mavros_msgs::PositionTarget::IGNORE_PZ
  );
  // This makes it so our velocities are sent in the UAV frame.
  // If you use FRAME_LOCAL_NED then it will be in map frame
  target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED;

  // Set the velocities we want (it is in m/s)
  // On the real drone send a minimum of 0.3 m/s in x/y (except when you send 0.0 m/s).
  // If the velocity is lower than 0.3 m/s then the UAV can move in any direction!
  target_position.position.x = 10.;
  target_position.position.y = 10.;
  target_position.position.z = 10.;

  // Set the yaw rate (it is in rad/s)
  target_position.yaw_rate = 2.f;

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  while(ros::ok()) {
    auto five_sec_since_last_request = (ros::Time::now() - last_request > ros::Duration(5.0));
    if (mavros->state().mode != "OFFBOARD" && five_sec_since_last_request) {
      auto success = mavros->set_custom_mode("OFFBOARD")
        == mavros_mode_adapter::server_response::success;
      ROS_INFO_COND(success, "Offboard enabled");

      last_request = ros::Time::now();
    } else {
      if (!mavros->state().armed && five_sec_since_last_request) {
        auto success = mavros->arm()
          == mavros_mode_adapter::server_response::success;
        ROS_INFO_COND(success, "Vehicle armed");
        last_request = ros::Time::now();
      }
    }

    local_pos_pub.publish(target_position);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
