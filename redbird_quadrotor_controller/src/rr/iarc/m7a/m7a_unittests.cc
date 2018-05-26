#include <gmock/gmock.h>

#include <rr/iarc/m7a/m7a.hpp>

#include <array>
#include <bitset>
#include <chrono>
#include <cmath>
#include <memory>
#include <type_traits>
#include <utility>
#include <functional>
#include <vector>

#define UNIT_LIB_DISABLE_IOSTREAM
#include <units.h>
#include <geometry_msgs/Point.h>

using namespace ::testing;
using namespace rr;
using namespace rr::iarc;
using namespace rr::iarc::m7;


/*
 *  Imagine this coordinate system:
 *
 *  { position (x,y,z) | x,y,z given in m }
 *  { velocity <x,y,z> | x,y,z given in m/s }
 *
 *        +y ^
 *           |
 *           |
 *                    GREEN
 *        (0,20)___________________(20,20)
 *            |                    |
 *            |                    |
 *            |                    |
 *            |                    |
 *            |                    |
 *            |                    |
 *            | _. v = <2,3,0>     |
 * drone      | /|                 |
 * origin --> |/___________________|
 *        (0,0)        RED       (20,0)  --> +x
 *          /
 *         /
 *       \/_ +z (away from Earth)
 *     (0,0,3)
 *
 */



template <typename T>
class vec3 final {
 private:
  std::array<T, 3> vector{};

 public:
  constexpr T x() const { return std::get<0>(vector); }
  constexpr T y() const { return std::get<1>(vector); }
  constexpr T z() const { return std::get<2>(vector); }

  template<typename Value>
  void set_x(Value&& x) { std::get<0>(vector) = std::forward<Value>(x); }
  template<typename Value>
  void set_y(Value&& y) { std::get<1>(vector) = std::forward<Value>(y); }
  template<typename Value>
  void set_z(Value&& z) { std::get<2>(vector) = std::forward<Value>(z); }

  template <typename Value> constexpr
  vec3(Value&& x, Value&& y, Value&& z) {
    set_x(std::forward(x));
    set_y(std::forward(y));
    set_z(std::forward(z));
  }

  constexpr
  vec3() {}

  using value_t = T;

  constexpr auto
  delta(const vec3<T> other) {
    value_t dx = x() - other.x(),
            dy = y() - other.y(),
            dz = z() - other.z();

    namespace um = units::math;
    auto delta = um::sqrt(um::pow<2>(dx) + um::pow<2>(dy) + um::pow<2>(dz));
    return delta;
  }
};

using velocity = vec3<units::velocity::meters_per_second>;
using point = vec3<units::length::meters>;



class agent {
 public:
  virtual point
  current_position() const = 0;

  virtual ~agent() {}
};

class roomba {
 public:
  virtual point
  position() const = 0;

  virtual velocity
  last_observed_velocity() const = 0;

  virtual bool
  is_obstacle() const = 0;

  virtual ~roomba() {}
};


using const_roombalist_ptr = std::shared_ptr<
  const std::vector<const std::unique_ptr<const roomba>>>;

class m7a_player {
 public:
  virtual point
  desired_position() const = 0;

  virtual void
  set_roombas(const_roombalist_ptr roombas) = 0;

  virtual ~m7a_player() {}
};

class m7a_player_base {
 protected:
  const_roombalist_ptr roombas_{};
  std::shared_ptr<agent> quadrotor_{};

 public:
  explicit m7a_player_base(std::shared_ptr<agent> quadrotor)
    : quadrotor_(quadrotor)
  {}

  virtual void
  set_roombas(const_roombalist_ptr roombas) {
    roombas_ = roombas;
  }

  virtual ~m7a_player_base() {}
};

enum class roomba_interaction {
  top_touch,
  obstruct,
};


// TODO: complete this
auto
expected_location_after(point position, velocity vel,
                        std::chrono::milliseconds time) {
  using namespace std;
  auto seconds = chrono::duration_cast<chrono::seconds>(time);
}

auto
expected_roomba_location_after(const roomba&) {

}


// TODO: if multithreaded, synchronize access to roomba_array
class point_scorer : public m7a_player_base {
 private:
  // assumes at least one roomba present, no synchronization
  const roomba& find_closest_roomba() const {
    if (roombas_->size() < 1)
      throw std::runtime_error("cannot find closest roombas when size == 0");

    auto me = point{};

    auto itr = roombas_->cbegin();
    auto end = roombas_->cend();

    const roomba* closest = &**(itr++);
    auto closest_distance = me.delta(closest->position());
    for (; itr != end; ++itr) {
      if (me.delta((*itr)->position(), me) < closest_distance) {
        closest = &**itr;
        closest_distance = me.delta(closest->position(), me);
      }
    }

    return *closest;
  }

  point top_touch_target_position(const roomba& r) {
    using namespace units::literals;
    point target_position{};
    target_position.set_x(r.position().x());
    target_position.set_y(r.position().y());
    target_position.set_z(0._m);
    return target_position;
  }

  // TODO: we want to land in front here
  point obstruct_target_position(const roomba& r) {
    using namespace units::literals;
    point target_position{};
    return target_position;
  }

  roomba_interaction interaction_mode_ = roomba_interaction::top_touch;

 public:
  virtual point
  desired_position() {
    if (roombas_->size() < 1)
      return point{}; // TODO: where do we go?

    const auto& closest_robot = find_closest_roomba();

    using i = roomba_interaction;
    switch (interaction_mode_) {
      case i::top_touch: {
        return top_touch_target_position(closest_robot);
      }
      case i::obstruct: {
        return obstruct_target_position(closest_robot); // TODO
      }
    }
  }

  virtual ~point_scorer() {}
};

enum class flights {
  takeoff_land,
  takeoff_translate_land,
};


