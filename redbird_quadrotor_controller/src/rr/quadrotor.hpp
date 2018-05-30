#ifndef RR_QUADROTOR_HPP_
#define RR_QUADROTOR_HPP_

#include <rr/mavros_util.hpp>
#include <rr/optional.hpp>

#include <units.h>

#include <utility>


namespace rr {
namespace quadrotor {


/**
 * @brief
 *    A simplified interface for quadrotor setpoint values.
 *
 * @sa
 *    void operator <<(target_position& t, const quadrotor_setpoints& sp);
 */
struct quadrotor_setpoints {
 private:
  using second_t = units::time::second_t;

  template <typename T>
  using optional = rr::optional<T>;

 public:
  using radian_t = units::angle::radian_t;
  using radian_per_s_t = std::decay_t<decltype(radian_t{} / second_t{})>;
  using meter_t = units::length::meter_t;

  optional<meter_t> x{};
  optional<meter_t> y{};
  optional<meter_t> z{};
  optional<radian_t> yaw{};
  optional<radian_per_s_t> yaw_rate{};
};


/**
 * @brief
 *    Put quadrotor_setpoints to the target_position, masking the typemask
 *  accordingly.
 *
 * @sa
 *    quadrotor_setpoints
 *
 * @code
 *   using units::literals;
 *   auto sp = quadrotor_setpoints{};
 *   sp.z = 3._m;
 *   sp.yaw = 180_deg;
 *
 *   mavros_msgs::PositionTarget target{};
 *   target << sp;
 *   send(target);
 */
void operator <<(mavros_util::target_position& t,
                 const quadrotor_setpoints& sp) {
  using tp = mavros_util::target_position;
  auto type_mask = mavros_util::kIgnoreAll;

  if (sp.x.is_set()) {
    t.position.x = sp.x.get().value();
    type_mask ^= tp::IGNORE_PX;
  }
  if (sp.y.is_set()) {
    t.position.y = sp.y.get().value();
    type_mask ^= tp::IGNORE_PY;
  }
  if (sp.z.is_set()) {
    t.position.z = sp.z.get().value();
    type_mask ^= tp::IGNORE_PZ;
  }
  if (sp.yaw.is_set()) {
    t.yaw = static_cast<decltype(t.yaw)>(sp.yaw.get().value());
    type_mask ^= tp::IGNORE_YAW;
  }
  if (sp.yaw_rate.is_set()) {
    t.yaw_rate = static_cast<decltype(t.yaw_rate)>(sp.yaw_rate.get().value());
    type_mask ^= tp::IGNORE_YAW_RATE;
  }

  t.type_mask = std::move(type_mask);
}


} // namespace quadrotor
} // namespace rr

#endif // RR_QUADROTOR_HPP_
