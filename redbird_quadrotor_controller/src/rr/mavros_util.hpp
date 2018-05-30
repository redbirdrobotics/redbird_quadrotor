#ifndef RR_MAVROS_UTIL_HPP_
#define RR_MAVROS_UTIL_HPP_

#include <rr/synchronized.hpp>
#include <rr/conditional_sleep.hpp>
#include <rr/optional.hpp>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <units.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <shared_mutex>
#include <thread>

namespace rr {
namespace mavros_util {

/**
 * @brief
 *    Preferred Mavros API for controlling quadrotor.
 *
 * @note
 *    Units in meters, m/s, radians, radians/s
 *
 * @sa
 *    quadrotor::setpoints
 */
using target_position = mavros_msgs::PositionTarget;


/**
 * @brief
 *    The bitmask for mavros_msgs::PositionTarget which directs mavros to
 *    ignore all fields.
 *
 * @sa
 *    target_position
 */
constexpr target_position::_type_mask_type kIgnoreAll =
    target_position::IGNORE_PX
  | target_position::IGNORE_PY
  | target_position::IGNORE_PZ
  | target_position::IGNORE_VX
  | target_position::IGNORE_VY
  | target_position::IGNORE_VZ
  | target_position::IGNORE_AFX
  | target_position::IGNORE_AFY
  | target_position::IGNORE_AFZ
  | target_position::IGNORE_YAW
  | target_position::IGNORE_YAW_RATE;


/**
 * @brief
 *    API identification strings for mavros
 *
 * @sa
 *    http://wiki.ros.org/mavros
 */
struct api_ids {
  /**
   * @brief
   *    ros::ServiceClient apis
   */
  struct services {
    static constexpr const char* kArming  = "mavros/cmd/arming";
    static constexpr const char* kSetmode = "mavros/set_mode";
  };

  /**
   * @brief
   *    Topics published or subscribed-to by mavros
   */
  struct topics {
    /**
     * @brief
     *    Topics published by mavros
     */
    struct published {
      static constexpr const char* kState = "mavros/state";
    };

    /**
     * @brief
     *    Topics subscribed-to by mavros
     */
    struct subscribed {
      static constexpr const char* kSetpointRawLocal
        = "mavros/setpoint_raw/local";
    };
  };
};


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
constexpr target_position::_type_mask_type
ignore_all_but(decltype(kIgnoreAll) mask = 0) {
  // clear the bits which are set in mask. (i.e. toggle the mask's set bits)
  return kIgnoreAll ^ mask;
}


/**
 * @brief
 *    Construct a target_position message with all ignore bitfields set. (i.e.,
 *    instructs mavros to ignore all setpoints.
 *
 * @sa
 *    ignore_all_but
 *    kIgnoreAll
 */
inline target_position
fully_ignored_mavros_setpoint_position() {
  auto setpoint_position_msg = target_position{};
  setpoint_position_msg.type_mask = kIgnoreAll;
  return setpoint_position_msg;
}


namespace px4 {

/**
 * @brief
 *    The custom modes available for PX4, sent via mavros.
 *
 * @sa
 *    http://wiki.ros.org/mavros/CustomModes
 */
enum class operating_mode {
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

  MODE_UNSUPPORTED // to handle unknown states
};

operating_mode
string_to_operating_mode(const std::string& mode_string);

std::string
to_string(operating_mode mode);

} // namespace px4


template <typename MavrosResolver,
          typename AsyncArmingClient,
          typename AsyncSetpointPublisher>
class _mavros_adapter {
  typename MavrosResolver::mavros_arming_client_ arming_client_{};
  typename MavrosResolver::mavros_mode_client_ mode_client_{};
  typename MavrosResolver::setpoint_publisher_ setpoint_publisher{};
  void arm();
  void disarm();
  bool is_armed();

  void set_target_mode(px4::operating_mode mode);
  px4::operating_mode target_mode();
  void current_reported_mode();

};


/**
 * @brief
 *    Asynchronously
 */
class mavros_adapter {
  // TODO: this class should know less. In particular, don't pass NodeHandle in
  // c'tor.

 public:
  enum class server_response {
    success,
    denied,
    fcu_not_connected,
    cannot_reach_server,
  };

  mavros_adapter(ros::NodeHandle& nh);

  px4::operating_mode mode() const;
  void set_mode(px4::operating_mode mode);

  void arm()    { return set_armed(true); }
  void disarm() { return set_armed(false); }


  template <typename T> void
  set_setpoints(T&& target) { setpoints_.set(std::forward<T>(target)); }
  target_position setpoints() const { return setpoints_.get(); }

  template <typename T> void
  set_arm_cmd(T&& cmd) { arm_cmd_.set(std::forward<T>(cmd)); }
  mavros_msgs::CommandBool arm_cmd() const { return arm_cmd_.get(); }

  /**
   * @brief
   *    Indicates that mavros is:
   *      1. connected to FCU
   *      2. armed
   *      3. in the currently-set operating mode
   */
  bool ready();

  virtual ~mavros_adapter();

 private:
  using lock = std::lock_guard<std::mutex>;

  server_response
  enable_current_arming_command();

  server_response
  enable_current_mode();

  mavros_msgs::State mavros_state() const {
    return mavros_state_.get();
  }

  bool publish_target_mode() {
    bool server_called = false;
    bool target_mode_achieved = (mode() == current_target_mode_);
    if (!target_mode_achieved) {
      auto success = enable_current_mode()
        == mavros_adapter::server_response::success;
      ROS_INFO_COND(success, "Offboard enabled");
      server_called = true;
    }

    return server_called;
  }

  bool publish_target_arm_cmd() {
    bool server_called = false;
    if (!mavros_state().armed) {
      auto success = enable_current_arming_command()
        == mavros_adapter::server_response::success;
      ROS_INFO_COND(success, "Vehicle armed");
      server_called = true;
    }

    return server_called;
  }

  void publish_setpoints() {
    setpoint_publisher_.publish(setpoints_.get());
  }

  template <typename CallServer, typename CheckSuccess>
  server_response
  call_server(CallServer&& call_server_func, CheckSuccess&& check_success) const;

  void
  set_armed(bool arm);

  ros::Subscriber state_sub_;
  synchronized<mavros_msgs::State> mavros_state_{};

  ros::ServiceClient arming_client_;
  synchronized<mavros_msgs::CommandBool> arm_cmd_{};

  ros::ServiceClient set_mode_client_;
  synchronized<mavros_msgs::SetMode> operating_mode_cmd_{};
  // TODO static assert is_lock_free?
  std::atomic<px4::operating_mode> current_target_mode_{};

  ros::Publisher setpoint_publisher_;
  synchronized<target_position> setpoints_
    { mavros_util::fully_ignored_mavros_setpoint_position() };

  std::atomic_bool destructor_called_{ false };
  bool destructing() { return destructor_called_.load(); }

  using ms = std::chrono::milliseconds;
  using s = std::chrono::seconds;
  using sleeper_t = synchronized<conditional_sleep>;
  sleeper_t check_armed_delay_   { ms{20}, [&]{ return destructing(); } };
  sleeper_t arm_delay_           { s{3},   [&]{ return destructing(); } };
  sleeper_t check_mode_delay_    { ms{20}, [&]{ return destructing(); } };
  sleeper_t change_mode_delay_   { s{3},   [&]{ return destructing(); } };
  sleeper_t send_setpoints_delay_{ ms{20}, [&]{ return destructing(); } };

  std::vector<std::thread> persistent_tasks;
};

} // namespace mavros_util
} // namespace rr

#endif // RR_MAVROS_UTIL_HPP_
