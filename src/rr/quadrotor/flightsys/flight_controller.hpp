#ifndef RR_QUADROTOR_FLIGHTSYS_FLIGHTCONTROLLER_HPP_
#define RR_QUADROTOR_FLIGHTSYS_FLIGHTCONTROLLER_HPP_

#include <rr/mavros_util/mavros_util.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

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
class i_quadrotor_adapter {
 public:
  virtual void
  publish_setpoint_position(const mavros_util::target_position& target);

  virtual geometry_msgs::PoseStamped
  get_current_position() const = 0;

  virtual geometry_msgs::TwistStamped
  get_current_velocity() const = 0;

  virtual ~i_quadrotor_adapter();
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
  virtual const geometry_msgs::PoseStamped&
  get_current_position() const = 0;


  virtual const geometry_msgs::TwistStamped&
  get_current_velocity() const = 0;

  virtual ~i_flight_controller();
};



class flight_controller : public i_flight_controller {
 private:
  using lock_guard = std::lock_guard<std::mutex>;

  std::unique_ptr<i_quadrotor_adapter> quadrotor_interfaces_;

  mutable geometry_msgs::PoseStamped current_position_;
  mutable geometry_msgs::TwistStamped current_velocity_;

  mavros_util::target_position target_position_;
  mutable std::mutex target_position_mutex_;
  std::function<void()> delay_between_publish_;

  void publish_setpoint();

  std::thread publish_until_destruct_thread;

  std::atomic_bool destructor_called_{ false };

  static constexpr auto default_publish_delay_ms = 50;
 public:
  flight_controller(
      decltype(quadrotor_interfaces_) quadrotor_interfaces,
      std::function<void()> delay_between_publish
        = [] {
          std::this_thread::sleep_for(
            std::chrono::milliseconds{ default_publish_delay_ms }
          );
        },
      const mavros_util::target_position& starting_target_position
        = mavros_util::fully_ignored_mavros_setpoint_position())
    : quadrotor_interfaces_(std::move(quadrotor_interfaces))
    , target_position_(starting_target_position)
    , delay_between_publish_(delay_between_publish)
  {
    publish_until_destruct_thread = std::thread(
      std::bind(&flight_controller::publish_setpoint, this)
    );
  }

  mavros_util::target_position
  target_position() const {
    lock_guard lock{ target_position_mutex_ };
    return target_position_;
  }

  void
  set_target_position(mavros_util::target_position new_target) {
    lock_guard lock{ target_position_mutex_ };
    target_position_ = new_target;
  }

  virtual const geometry_msgs::PoseStamped&
  get_current_position() const {
    current_position_ = quadrotor_interfaces_->get_current_position();
    return current_position_;
  }

  virtual const geometry_msgs::TwistStamped&
  get_current_velocity() const {
    current_velocity_ = quadrotor_interfaces_->get_current_velocity();
    return current_velocity_;
  }

  virtual ~flight_controller();
};



} // namespace flightsys
} // namespace quadrotor
} // namespace rr

#endif // RR_QUADROTOR_FLIGHTSYS_FLIGHTCONTROLLER_HPP_
