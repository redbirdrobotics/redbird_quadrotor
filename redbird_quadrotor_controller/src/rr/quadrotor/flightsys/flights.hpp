#ifndef RR_QUADROTOR_FLIGHTSYS_FLIGHTS_HPP_
#define RR_QUADROTOR_FLIGHTSYS_FLIGHTS_HPP_

#include <functional>
#include <utility>

namespace rr {
namespace quadrotor {
namespace flightsys {

enum class cancellation {
  none,
  as_convenient,
  immediately,
};

using canceller = std::function<cancellation ()>;


class flight {
 public:
  /*
   * @breif
   *    Fly the given flight. Cancel the flight whenever the given
   *    cancellation_function returns a non- cancellation::none command.
   */
  virtual void fly(canceller c) = 0;

  virtual ~flight();
};

} // namespace flightsys
} // namespace quadrotor
} // namespace rr

#endif // RR_QUADROTOR_FLIGHTSYS_FLIGHTS_HPP_
