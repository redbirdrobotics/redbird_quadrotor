#ifndef RR_QUADROTOR_FLIGHTSEQUENCE_CIRCLE_HPP_
#define RR_QUADROTOR_FLIGHTSEQUENCE_CIRCLE_HPP_

#include <rr/optional.hpp>
#include <rr/mavros_util.hpp>

#include <ros/ros.h>
#include <units.h>

#include <bitset>
#include <functional>
#include <memory>
#include <tuple>
#include <type_traits>
#include <utility>

namespace rr {
namespace quadrotor {
namespace flight_sequence {


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

class i_quadrotor_controller {
 public:
  virtual ~i_quadrotor_controller();
  virtual void operator <<(const quadrotor_setpoints& setpoints) = 0;
};
i_quadrotor_controller::~i_quadrotor_controller() {}


enum class sequence {
  takeoff_and_land,
  circle,
};

void operator <<(mavros_util::target_position& t,
                 const quadrotor_setpoints& sp) {
  using tp = mavros_util::target_position;
  auto type_mask = mavros_util::kIgnoreAll;
  using tmt = decltype(type_mask);
  if (sp.x.is_set()) {
    t.position.x = sp.x.get().value();
    type_mask &= (tmt)~tp::IGNORE_PX;
  }
  if (sp.y.is_set()) {
    t.position.y = sp.y.get().value();
    type_mask &= (tmt)~tp::IGNORE_PY;
  }
  if (sp.z.is_set()) {
    t.position.z = sp.z.get().value();
    type_mask &= (tmt)~tp::IGNORE_PZ;
  }
  if (sp.yaw.is_set()) {
    t.yaw = static_cast<decltype(t.yaw)>(sp.yaw.get().value());
    type_mask &= (tmt)~tp::IGNORE_YAW;
  }
  if (sp.yaw_rate.is_set()) {
    t.yaw_rate = static_cast<decltype(t.yaw_rate)>(sp.yaw_rate.get().value());
    type_mask &= (tmt)~tp::IGNORE_YAW_RATE;
  }

  t.type_mask = std::move(type_mask);
}

template <typename SendSetpoints, typename CancelExecution>
void
circle(SendSetpoints&& send_setpoints, CancelExecution&& should_terminate) {
  using namespace units;
  using namespace units::literals;
  using namespace units::angle;
  using namespace units::length;
  using namespace units::math;
  namespace um = units::math;

  const auto altitude = 2.75_m;
  const auto radius = 3._m;
  const auto revolution_period = 15._s;
  const radian_t _2pi = 360._deg;

  auto theta_at_t = [&](const time::millisecond_t& t) {
    time::millisecond_t remainder = um::fmod(t, revolution_period);
    radian_t theta = (remainder / revolution_period) * _2pi;
    return theta;
  };

  auto xy_at_theta = [&](const radian_t& theta) {
    meter_t x = radius * um::cos(theta);
    meter_t y = radius * um::sin(theta);
    return std::make_tuple(x, y);
  };

  auto setpoints = quadrotor_setpoints{};
  setpoints.z = altitude;

  using clock = std::chrono::high_resolution_clock;
  const auto start = clock::now();
  time::millisecond_t t;
  while (!should_terminate()) {
    t = clock::now() - start;
    radian_t theta = theta_at_t(t);
    std::tie(setpoints.x, setpoints.y) = xy_at_theta(theta);
    setpoints.yaw = theta + 180._deg;
    send_setpoints(setpoints);
  }
}

template <typename SendSetpoints, typename CancelExecution>
void
takeoff_and_land(SendSetpoints&& send_setpoints, CancelExecution&& should_terminate) {
  using namespace units;
  using namespace units::literals;
  using namespace units::angle;
  using namespace units::length;
  using namespace units::math;
  namespace um = units::math;

  const auto altitude = 2._m;

  auto setpoints = quadrotor_setpoints{};
  setpoints.x = 0._m;
  setpoints.y = 0._m;
  setpoints.z = altitude;

  using clock = std::chrono::high_resolution_clock;

  const auto start = clock::now();
  auto t = clock::now();
  const auto loiter_time = std::chrono::seconds{6};
  while (!should_terminate() && t < (start + loiter_time)) {
    t = clock::now();
    send_setpoints(setpoints);
  }
  
  setpoints.z = -1._m;

  const auto landing_delay = std::chrono::seconds{10};

  while (!should_terminate() && t < start + landing_delay) {
    t = clock::now();
    send_setpoints(setpoints);
  }
}

} // namespace rr
} // namespace quadrotor
} // namespace flight_sequence

#endif // RR_QUADROTOR_FLIGHTSEQUENCE_CIRCLE_HPP_
