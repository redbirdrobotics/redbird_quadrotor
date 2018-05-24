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
//enum class mavros_px4_mode : const char* {
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

//mavros_px4_mode
//mavros_px4_mode_from_string(const std::string& mode) {
//  if (mode == "OFFBOARD") return mavros_px4_mode::OFFBOARD;

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

//  mavros_px4_mode mode() const {
//    lock_guard lock{ state_mutex };
//    auto mode = mavros_px4_mode_from_string(state.mode);
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






// This code below taken directly from PX4 example documentation

#include <rr/mavros_util/mavros_util.hpp>

#include <boost/functional.hpp>
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

/**
 * @sa
 *    http://wiki.ros.org/mavros/CustomModes
 */
enum class mavros_px4_mode {
  MANUAL,
  ACRO,
  ALTCTL,
  POSCTL,
  OFFBOARD,
  STABILIZED,
  RATTITUDE,
  AUTO_MISSION,
  AUTO_LOITER,
  AUTO_RTL,
  AUTO_LAND,
  AUTO_RTGS,
  AUTO_READY,
  AUTO_TAKEOFF,

  UNSUPPORTED
};
using mavros_state_mode_string = decltype(mavros_msgs::State{}.mode);
const std::map<mavros_px4_mode, mavros_state_mode_string>
    mavros_px4_mode_strings {
  { mavros_px4_mode::MANUAL, "MANUAL" },
  { mavros_px4_mode::ACRO, "ACRO" },
  { mavros_px4_mode::ALTCTL, "ALTCTL" },
  { mavros_px4_mode::POSCTL, "POSCTL" },
  { mavros_px4_mode::OFFBOARD, "OFFBOARD" },
  { mavros_px4_mode::STABILIZED, "STABILIZED" },
  { mavros_px4_mode::RATTITUDE, "RATTITUDE" },
  { mavros_px4_mode::AUTO_MISSION, "AUTO.MISSION" },
  { mavros_px4_mode::AUTO_LOITER, "AUTO.LOITER" },
  { mavros_px4_mode::AUTO_RTL, "AUTO.RTL" },
  { mavros_px4_mode::AUTO_LAND, "AUTO.LAND" },
  { mavros_px4_mode::AUTO_RTGS, "AUTO.RTGS" },
  { mavros_px4_mode::AUTO_READY, "AUTO.READY" },
  { mavros_px4_mode::AUTO_TAKEOFF, "AUTO.TAKEOFF" }
};


namespace detail {

std::unordered_map<mavros_state_mode_string, mavros_px4_mode>
    init_mavros_px4_mode_map() {
  std::unordered_map<mavros_state_mode_string, mavros_px4_mode> map{};
  map.reserve(mavros_px4_mode_strings.size());

  for (const auto& mode_and_string : mavros_px4_mode_strings)
    map.emplace(mode_and_string.second, mode_and_string.first);

  return map;
}

} // namespace detail


std::unordered_map<mavros_state_mode_string, mavros_px4_mode>
  mavros_px4_modes_by_string = detail::init_mavros_px4_mode_map();


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

  ros::Subscriber state_sub_;
  mavros_msgs::State mavros_state_{};

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
    const uint32_t mavros_state_queue_size = 10u;
    auto&& update_state_callback = [&](const mavros_msgs::State::ConstPtr& msg){
      mavros_state_ = *msg;
    };
    state_sub_ = nh.subscribe<mavros_msgs::State>(
      kMavrosStateTopic_Id,
      mavros_state_queue_size,
      update_state_callback
    );

    arming_client
        = nh.serviceClient<mavros_msgs::CommandBool>(kMavrosArmingService_Id);

    set_mode_client
        = nh.serviceClient<mavros_msgs::SetMode>(kMavrosSetmodeService_Id);
  }

  mavros_px4_mode mode() const {
    auto itr = mavros_px4_modes_by_string.find(mavros_state_.mode);
    auto end = mavros_px4_modes_by_string.cend();
    if (itr == end)
      return mavros_px4_mode::UNSUPPORTED;

    auto mode = itr->second;
    return mode;
  }
  mavros_msgs::State state() const { return mavros_state_; }
  auto arm() { return set_armed(true); }
  auto disarm() { return set_armed(false); }

  auto
  set_mode(mavros_px4_mode mode) {
    operating_mode_cmd_.request.custom_mode = mavros_px4_mode_strings.at(mode);
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
  ROS_INFO("Awaiting FCU connection");
  while(ros::ok() && !mavros->state().connected) {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("FCU connected");

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
  // This makes it so our velocities are sent in the UAV frame.
  // If you use FRAME_LOCAL_NED then it will be in map frame
  target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED;

  // Set the velocities we want (it is in m/s)
  // On the real drone send a minimum of 0.3 m/s in x/y (except when you send 0.0 m/s).
  // If the velocity is lower than 0.3 m/s then the UAV can move in any direction!
  target_position.position.x = 3.;
  target_position.position.y = 3.;
  target_position.position.z = 3.;

  // Set the yaw rate (it is in rad/s)
  target_position.yaw_rate = 2.f;


  while(ros::ok()) {
    if (mavros->mode() != mavros_px4_mode::OFFBOARD) {
      auto success = mavros->set_mode(mavros_px4_mode::OFFBOARD)
        == mavros_mode_adapter::server_response::success;
      ROS_INFO_COND(success, "Offboard enabled");
    } else {
      if (!mavros->state().armed) {
        auto success = mavros->arm()
          == mavros_mode_adapter::server_response::success;
        ROS_INFO_COND(success, "Vehicle armed");
      }
    }

    local_pos_pub.publish(target_position);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
