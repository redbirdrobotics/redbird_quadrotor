#ifndef RR_CONDITIONALSLEEP_HPP_
#define RR_CONDITIONALSLEEP_HPP_

#include <chrono>
#include <functional>
#include <thread>
#include <type_traits>

namespace rr {

namespace detail {
using default_clock = std::chrono::high_resolution_clock;
} // namespace detail

template <typename Clock = detail::default_clock,
          typename Duration, typename ShouldStopFunction>
void sleep_for(Duration&& duration,
               ShouldStopFunction&& should_stop) {
  auto start = Clock::now();
  auto end = start + std::forward<Duration>(duration);
  do {
    std::this_thread::yield();
  } while (!should_stop() && Clock::now() < end);
}

template <typename Clock = detail::default_clock,
          typename Duration = std::chrono::microseconds>
class _conditional_sleep;


/**
 * @brief
 *    Sleeps this thread for the currently set duration _or_ until the
 *    should_stop_ function returns true.
 */
using conditional_sleep = _conditional_sleep<>;


template <typename Clock, typename Duration>
class _conditional_sleep {
 public:
  /**
   * @param duration
   *    Initial value for sleep duration
   *
   * @param stop_sleep_when_true
   *    This function preempts sleep as soon as it returns true (instead of
   *  continuing to block the local thread for the remainder of the sleep
   *  duration.)
   */
  template <typename ShouldStopFunc>
  explicit _conditional_sleep(
      Duration duration,
      ShouldStopFunc&& stop_sleep_when_true)
    : duration_(duration)
    , should_stop_(std::forward<ShouldStopFunc>(stop_sleep_when_true))
  {}

  template <typename _Duration>
  void set_duration(_Duration&& d) {
    duration_ = std::forward<_Duration>(d);
  }

  Duration duration() const {
    return duration_;
  }

  /**
   * @brief
   *    Sleeps this thread for the currently set duration _or_ until the
   *    should_stop_ function returns true.
   */
  void sleep() const {
    sleep_for<Clock>(duration_, should_stop_);
  }

 private:
  Duration duration_;
  std::function<bool()> should_stop_;
};


} // namespace rr

#endif // RR_CONDITIONALSLEEP_HPP_
