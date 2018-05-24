//#include <rr/mavros_util/mavros_util.hpp>

//#include <boost/functional.hpp>
//#include <ros/ros.h>
//#include <geometry_msgs/PoseStamped.h>

//#include <cstddef>
//#include <string>
//#include <type_traits>

//auto
//arbitrary_setpoint() {
//  rr::mavros_util::target_position target_position{};

//  target_position.header.stamp = ros::Time::now();
//  target_position.header.frame_id = "base_link";

//  target_position.type_mask =
//      mavros_msgs::PositionTarget::IGNORE_VX
//    | mavros_msgs::PositionTarget::IGNORE_VY
//    | mavros_msgs::PositionTarget::IGNORE_VZ
//    | mavros_msgs::PositionTarget::IGNORE_YAW
//    | mavros_msgs::PositionTarget::IGNORE_AFX
//    | mavros_msgs::PositionTarget::IGNORE_AFY
//    | mavros_msgs::PositionTarget::IGNORE_AFZ;

//  target_position.position.x = -3.;
//  target_position.position.y = -3.;
//  target_position.position.z = 3.;
//  target_position.yaw_rate = 2.f;

//  return target_position;
//}
//#include <chrono>
//#include <thread>
//int main(int argc, char **argv) {
//  using namespace rr::mavros_util;
//  std::this_thread::sleep_for(std::chrono::seconds{5});
//  (void)(5 + 5);

//  ros::init(argc, argv, "quadrotor_controller");
//  ros::NodeHandle nh;

//  auto mavros = std::make_shared<mavros_mode_adapter>(nh);

//  ros::Publisher setpoint_publisher = nh.advertise<rr::mavros_util::target_position>
//    (rr::mavros_util::kMavrosSetpointLocalRawId, 10);

//  //the setpoint publishing rate MUST be faster than 2Hz
//  ros::Rate rate(20.0);

//  ROS_INFO("Awaiting FCU connection");
//  while(ros::ok() && !mavros->state().connected) {
//    ros::spinOnce();
//    rate.sleep();
//  }
//  ROS_INFO("FCU connected");

//  // Ensure offboard enabled.
//  const auto* offboard_id = "OFFBOARD";
//  if (mavros->state().mode != offboard_id) {
//    auto success = mavros->set_custom_mode(offboard_id)
//      == mavros_mode_adapter::server_response::success;
//    ROS_INFO_COND(success, "Offboard enabled");
//  }

//  // Ensure vehicle is "armed".
//  if (!mavros->state().armed) {
//    auto success = mavros->arm()
//      == mavros_mode_adapter::server_response::success;
//    ROS_INFO_COND(success, "Vehicle armed");
//  }

//  auto setpoint = arbitrary_setpoint();

//  ROS_INFO("starting");
//  while(ros::ok()) {
//    ROS_INFO("publishing setpoint");
//    setpoint_publisher.publish(setpoint);
//    ROS_INFO("published");
//    ros::spinOnce();
//    ROS_INFO("spun");
//    rate.sleep();
//    ROS_INFO("slept");
//  }

//  ROS_INFO("quadrotor_controller shutdown signalled; terminating.");
//  return 0;
//}


// This code below taken directly from PX4 example documentation

//#include <rr/mavros_util/mavros_util.hpp>

//#include <ros/ros.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <mavros_msgs/CommandBool.h>
//#include <mavros_msgs/SetMode.h>
//#include <mavros_msgs/State.h>

//#include <atomic>
//#include <thread>
//#include <chrono>
//#include <functional>
//#include <mutex>

//// TODO: this class should know less. In particular, don't pass NodeHandle in
//// c'tor.
//class mavros_mode_adapter {
//  ros::ServiceClient arming_client;

//  ros::ServiceClient set_mode_client;

//  static constexpr const char* const
//    kMavrosArmingService_Id = "mavros/cmd/arming";

//  static constexpr const char* const
//    kMavrosSetmodeService_Id = "mavros/set_mode";

//  static constexpr const char* const
//    kMavrosStateTopic_Id = "mavros/state";

//  mavros_msgs::State mavros_state;

//  mavros_msgs::CommandBool arm_cmd;

//  bool set_armed_mode(bool arm) {
//    arm_cmd.request.value = arm;
//    ROS_INFO_STREAM("Attempting to " << (arm ? "arm" : "disarm") << " vehicle");

//    auto call_successful = arming_client.call(arm_cmd);
//    ROS_ERROR_COND(!call_successful, "Could not open connection to mavros arming server");

//    ROS_INFO_STREAM("Vehicle " << (arm_cmd.response.result ? "armed" : "disarmed"));
//    return arm_cmd.response.success;
//  }

// public:
//  mavros_mode_adapter(ros::NodeHandle& nh) {
//    arming_client
//        = nh.serviceClient<mavros_msgs::CommandBool>(kMavrosArmingService_Id);

//    set_mode_client
//        = nh.serviceClient<mavros_msgs::SetMode>(kMavrosSetmodeService_Id);

//    const uint32_t mavros_state_queue_size = 10u;
//    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
//      kMavrosStateTopic_Id,
//      mavros_state_queue_size,
//      [&](const mavros_msgs::State::ConstPtr& msg) {
//        mavros_state = *msg;
//      }
//    );
//  }

//  bool arm() { return set_armed_mode(true); }
//  bool disarm() { return set_armed_mode(false); }
//};

//void arm_mavros(ros::ServiceClient& arming_client, mavros_msgs::CommandBool& arm_cmd) {
//  if (arming_client.call(arm_cmd) && arm_cmd.response.success)
//    ROS_INFO("Vehicle armed");
//}

//auto
//enable_armed_offboard(
//    std::unique_ptr<const mavros_msgs::State> mavros_state,
//    std::unique_ptr<std::mutex> mavros_state_mutex,
//    mavros_msgs::CommandBool arm_cmd,
//    mavros_msgs::SetMode offb_set_mode,
//    ros::ServiceClient set_mode_client,
//    ros::ServiceClient arming_client,
//    ) {
//  if (mavros_state->mode != "OFFBOARD") {
//    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
//      ROS_INFO("Offboard enabled");
//  } else {
//    if (!mavros_state->armed) {
//      arm_mavros(arming_client, arm_cmd);
//    }
//  }
//}

///**
// * @sa
// *    http://wiki.ros.org/mavros/CustomModes
// */
//enum class mavros_util::px4::operating_mode : const char* {
//  MANUAL,
//  ACRO,
//  ALTCTL,
//  POSCTL,
//  OFFBOARD,
//  STABILIZED,
//  RATTITUDE,
//  AUTO_MISSION,
//  AUTO_LOITER,
//  AUTO_RTL,
//  AUTO_LAND,
//  AUTO_RTGS,
//  AUTO_READY,
//  AUTO_TAKEOFF
//};

//mavros_util::px4::operating_mode
//mavros_util::px4::operating_mode_from_string(const std::string& mode) {
//  if (mode == "OFFBOARD") return mavros_util::px4::operating_mode::OFFBOARD;

//  // we don't need any others at this time; throw:
//  throw std::runtime_error("other modes not implemented");
//}

//class mavros_state {
//  using lock_guard = std::lock_guard<std::mutex>;

// protected:
//  mavros_msgs::State state{};
//  mutable std::mutex state_mutex{};
//  mavros_msgs::SetMode offb_set_mode{};
//  mavros_msgs::CommandBool arm_cmd{};

// public:
//  mavros_state() {
//    offb_set_mode.request.custom_mode = "OFFBOARD";
//    arm_cmd.request.value = true;
//  }

//  void set_state() {

//  }

//  mavros_util::px4::operating_mode mode() const {
//    lock_guard lock{ state_mutex };
//    auto mode = mavros_util::px4::operating_mode_from_string(state.mode);
//    return mode;
//  }

//  bool armed() const {
//    lock_guard lock{ state_mutex };
//    bool armed = static_cast<bool>(state.armed);
//    return armed;
//  }

//  virtual ~mavros_state() {}
//};

//int main(int argc, char **argv) {
//  ros::init(argc, argv, "quadrotor_controller");
//  ros::NodeHandle nh;

//  auto mavros_state = std::make_unique<mavros_msgs::State>;
//  auto set_state_callback = [&](const mavros_msgs::State::ConstPtr& msg) {
//    *mavros_state = *msg;
//  };

//  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
//    ("mavros/state", 10, set_state_callback);
//  ros::Publisher local_pos_pub = nh.advertise<rr::mavros_util::target_position>
//    (rr::mavros_util::kMavrosSetpointLocalRawId, 10);
//  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
//    ("mavros/cmd/arming");
//  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
//    ("mavros/set_mode");

//  //the setpoint publishing rate MUST be faster than 2Hz
//  ros::Rate rate(20.0);

//  // wait for FCU connection
////  while(ros::ok() && !mavros_state->connected) {
////    ros::spinOnce();
////    rate.sleep();
////  }
//  rr::mavros_util::target_position target_position{};
//  target_position.header.stamp = ros::Time::now();
//  target_position.header.frame_id = "base_link";
//  // We will ignore everything except for YAW_RATE and velocities in X, Y, and Z
//  target_position.type_mask = rr::mavros_util::ignore_all_but(
//      mavros_msgs::PositionTarget::IGNORE_YAW_RATE
//    | mavros_msgs::PositionTarget::IGNORE_PX
//    | mavros_msgs::PositionTarget::IGNORE_PY
//    | mavros_msgs::PositionTarget::IGNORE_PZ
//  );
//  // This makes it so our velocities are sent in the UAV frame.
//  // If you use FRAME_LOCAL_NED then it will be in map frame
//  target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED;

//  // Set the velocities we want (it is in m/s)
//  // On the real drone send a minimum of 0.3 m/s in x/y (except when you send 0.0 m/s).
//  // If the velocity is lower than 0.3 m/s then the UAV can move in any direction!
//  target_position.position.x = 10.;
//  target_position.position.y = 10.;
//  target_position.position.z = 10.;

//  // Set the yaw rate (it is in rad/s)
//  target_position.yaw_rate = 2.f;



//  std::thread(enable_armed_offboard,
//  while(ros::ok()) {
//    local_pos_pub.publish(target_position);
//    ros::spinOnce();
//    rate.sleep();
//  }

//  return 0;
//}







#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <string>
#include <type_traits>

#include <rr/mavros_util.hpp>

auto
arbitrary_setpoints() {
  rr::mavros_util::target_position target_position{};
  target_position.header.stamp = ros::Time::now();
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

  return target_position;
}

int main(int argc, char **argv) {
  using namespace ::rr;
  using namespace ::rr::mavros_util;

  ros::init(argc, argv, "quadrotor_controller");
  ros::NodeHandle nh;

  auto mavros = std::make_shared<mavros_adapter>(nh);
  ros::Publisher local_pos_pub = nh.advertise<target_position>(
    mavros_util::api_ids::topics::subscribed::kSetpointRawLocal, 10);

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  // await the FCU connection
  ROS_INFO("Awaiting FCU connection");
  while(ros::ok() && !mavros->state().connected) {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("FCU connected");


  auto setpoints = arbitrary_setpoints();

  while(ros::ok()) {
    if (mavros->mode() != px4::operating_mode::OFFBOARD) {
      auto success = mavros->set_mode(px4::operating_mode::OFFBOARD)
        == mavros_adapter::server_response::success;
      ROS_INFO_COND(success, "Offboard enabled");
    } else {
      if (!mavros->state().armed) {
        auto success = mavros->arm()
          == mavros_adapter::server_response::success;
        ROS_INFO_COND(success, "Vehicle armed");
      }
    }

    setpoints.header.stamp = ros::Time::now();
    local_pos_pub.publish(setpoints);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
