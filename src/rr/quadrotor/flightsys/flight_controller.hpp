#ifndef RR_QUADROTOR_FLIGHTSYS_FLIGHTCONTROLLER_HPP_
#define RR_QUADROTOR_FLIGHTSYS_FLIGHTCONTROLLER_HPP_

#include <rr/mavros_util/mavros_util.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

namespace rr {
namespace quadrotor {
namespace flightsys {

/**
 * @brief
 *    The interface through which flight_controller objects will perform its
 *    behaviours.
 *
 * @note
 *    The functions provided here may be called from other threads.
 */
class i_quadrotor_interfaces {
 public:
  virtual void
  publish_setpoint_position(const mavros_util::target_position& target);

  virtual geometry_msgs::PoseStamped
  get_current_position() const = 0;

  virtual ~i_quadrotor_interfaces();
};


struct flight_controller_config final {
  std::function<void(const mavros_util::target_position&)> publish_setpoint_position;

  std::function<void(const geometry_msgs::PoseStamped&)> get_current_position;

  std::function<void()> wait_between_publish;

  flight_controller_config(
      decltype(publish_setpoint_position) _publish_setpoint_positionition,
      decltype(wait_between_publish) _wait_between_publish)
    : publish_setpoint_position(_publish_setpoint_positionition)
    , wait_between_publish(_wait_between_publish)
  {}
};


/**
 * @brief
 *    The interface through which to set the target position of the quadrotor
 *    and check current reported position. This interface is designed to be
 *    implicity asynchronous (i.e. non-blocking).
 */
class i_flight_controller {
 public:
  /**
   * @return
   *    The currently set target position for the quadrotor.
   */
  virtual mavros_util::target_position
  target_position() const = 0;

  /**
   * @brief
   *    Overwrite the current target position for the quadrotor. This operation
   *    is intended to be asynchronous (i.e. non-blocking).
   *
   * @param new_target
   *    The new target position with which to overwrite the current target
   *    position.
   */
  virtual void
  set_target_position(mavros_util::target_position new_target) = 0;

  /**
   * @brief
   *    Get the currently reported position of the quadrotor. At flight-time,
   *    this will ultimately reach out to hardware to sense position.
   *
   * @return
   *    The currently reported position of the quadrotor.
   */
  virtual geometry_msgs::PoseStamped
  get_current_position() const = 0;


  virtual ~i_flight_controller();
};



//class flight_controller : public i_flight_controller {
// private:
//  using lock_guard = std::lock_guard<std::mutex>;

//  std::unique_ptr<flight_controller_config> quadrotor_interfaces;

//  mavros_util::target_position target_position_;
//  mutable std::mutex target_position_mutex_;
//  geometry_msgs::PoseStamped current_position_;
//  mutable std::mutex current_position_mutex_;

//  std::atomic_bool destructor_called_{ false };

//  void update_positions();

//  std::thread publish_until_destruct_thread;

// public:
//  flight_controller(
//      decltype(config_) config,
//      const mavros_util::target_position& target_position
//        = mavros_util::fully_ignored_mavros_setpoint_position())
//    : config_(std::move(config))
//    , target_position_(target_position)
//  {
//    publish_until_destruct_thread = std::thread(
//      std::bind(&flight_controller::update_positions, this)
//    );
//  }

//  mavros_util::target_position
//  target_position() const {
//    lock_guard lock{ target_position_mutex_ };
//    return target_position_;
//  }

//  void
//  set_target_position(mavros_util::target_position new_target) {
//    lock_guard lock{ target_position_mutex_ };
//    target_position_ = new_target;
//  }

//  virtual geometry_msgs::PoseStamped
//  get_current_position() const {
//    lock_guard lock{ current_position_mutex_ };
//    return current_position_;
//  }

//  virtual ~flight_controller();
//};



} // namespace flightsys
} // namespace quadrotor
} // namespace rr

#endif // RR_QUADROTOR_FLIGHTSYS_FLIGHTCONTROLLER_HPP_
