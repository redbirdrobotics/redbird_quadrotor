#ifndef RR_MAVROS_UTIL_HPP_
#define RR_MAVROS_UTIL_HPP_

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

namespace rr {
namespace mavros_util {

/**
 * @brief
 *    Preferred Mavros API for controlling quadrotor.
 *
 * @note
 *    Units in meters, m/s, radians, radians/s
 */
using target_position = mavros_msgs::PositionTarget;


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
 *    The bitmask for mavros_msgs::PositionTarget which directs mavros to
 *    ignore all fields.
 *
 * @sa
 *    mavros_msgs::PositionTarget::IGNORE_PX
 */
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
constexpr auto
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
inline auto
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


using mavros_state_mode_string_t = decltype(mavros_msgs::State{}.mode);

operating_mode
string_to_operating_mode(const mavros_state_mode_string_t& mode_string);

mavros_state_mode_string_t
to_string(operating_mode mode);

} // namespace px4

class i_mavros_adapter {
 public:
  virtual ~i_mavros_adapter();
};

class synchronized_delay {
 private:
  using lock = std::lock_guard<std::mutex>;

  std::chrono::microseconds duration_;
  mutable std::mutex duration_mutex_;

  void sleep_for(std::chrono::microseconds&& us) const {
    auto start = std::chrono::high_resolution_clock::now();
    auto end = start + us;
    do {
      std::this_thread::yield();
    } while (std::chrono::high_resolution_clock::now() < end);
  }

 public:
  explicit synchronized_delay(std::chrono::microseconds duration
                        = std::chrono::microseconds{})
    : duration_(duration)
  {}

  template <typename Duration>
  void set_duration(Duration&& d) {
    lock l(duration_mutex_);
    duration_ = std::forward<Duration>(d);
  }

  std::chrono::microseconds duration() const {
    lock l(duration_mutex_);
    return duration_;
  }

  void sleep() const {
    sleep_for(duration());
  }
};

template <typename T>
class sychronized final {
 private:
  using Type = std::remove_reference_t<T>;
  using lock = std::lock_guard<std::mutex>;
  Type value_;
  mutable std::mutex mutex_{};

 public:
  template <typename U>
  sychronized(U&& value)
    : value_(std::forward<U>(value))
  {}

  template <typename U> auto
  set(U&& new_value) {
    lock l(mutex_);
    value_ = std::forward<U>(new_value);
  }

  Type get() const {
    lock l(mutex_);
    return value_;
  }

  template <typename Mutator>
  void alter(Mutator&& func) {
    lock l(mutex_);
    std::forward<Mutator>(func)(value_);
  }
};

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
  std::vector<std::thread> persistent_tasks{};
  std::atomic_bool destructor_called_{ false };

  using ms = std::chrono::milliseconds;
  using s = std::chrono::seconds;
  synchronized_delay check_armed_delay_     { ms{20} };
  synchronized_delay arm_delay_             { s{3} };
  synchronized_delay check_mode_delay_      { ms{20} };
  synchronized_delay change_mode_delay_     { s{3} };
  synchronized_delay send_setpoints_delay_  { ms{20} };

  server_response
  enable_current_arming_command();

  server_response
  enable_current_mode();

  mavros_msgs::State mavros_state() const {
    lock l(mavros_state_mutex_);
    return mavros_state_;
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
    setpoint_publisher_.publish(setpoints_);
  }

  ros::Subscriber state_sub_;
  mavros_msgs::State mavros_state_{};
  mutable std::mutex mavros_state_mutex_{};

  ros::ServiceClient arming_client_;

  mavros_msgs::CommandBool arm_cmd_{};
  mutable std::mutex arm_cmd_mutex_{};
  mavros_msgs::CommandBool arm_cmd() const {
    lock l(arm_cmd_mutex_);
    return arm_cmd_;
  }
  template <typename T>
  mavros_msgs::CommandBool set_arm_cmd(T&& cmd) {
    lock l(arm_cmd_mutex_);
    arm_cmd_ = std::forward<T>(cmd);
  }

  std::atomic<px4::operating_mode> current_target_mode_{}; // TODO static assert lock free?

  ros::ServiceClient set_mode_client_;
  mavros_msgs::SetMode operating_mode_cmd_{};

  ros::Publisher setpoint_publisher_;
  synchronized<target_position> setpoints_
    = mavros_util::fully_ignored_mavros_setpoint_position();
  mutable std::mutex setpoints_mutex_;
  target_position setpoints() const {
    lock l(setpoints_mutex_);
    return setpoints_;
  }
  template <typename T>
  target_position set_setpoints(T&& target) {
    lock l(setpoints_mutex_);
    setpoints_ = std::forward<T>(target);
  }

  template <typename CallServer, typename CheckSuccess>
  server_response
  call_server(CallServer&& call_server_func, CheckSuccess&& check_success) const;

  void
  set_armed(bool arm);
};

} // namespace mavros_util
} // namespace rr

#endif // RR_MAVROS_UTIL_HPP_
