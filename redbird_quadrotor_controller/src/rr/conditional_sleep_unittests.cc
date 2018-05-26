#include <gtest/gtest.h>

#include "conditional_sleep.hpp"

#include <cstddef>

using namespace rr;

using faketime_t = std::uintmax_t;
using fakeduration_t = std::uintmax_t;

// NOTE: this is globally stateful and may fail if more tests are added.
// If more tests are added, please consider redesigning the template dependency
// injection used to make this interface possible, or use unique static
// functions for each test (macros recommended in that case).
struct fakeclock {
  static thread_local faketime_t time;
  static faketime_t now() noexcept { return ++time; }
};
faketime_t thread_local fakeclock::time = faketime_t{0};



using test_conditional_sleep = _conditional_sleep<fakeclock, fakeduration_t>;

TEST(ConditionalSleep, does_preempt) {
  const faketime_t preempt_time = 3;
  auto should_stop_sleeping = [&] {
    auto now = fakeclock::now();
    return (now > preempt_time);
  };
  auto sleep_duration = fakeduration_t{ 10 };
  test_conditional_sleep sleeper{
    sleep_duration,
    should_stop_sleeping
  };
  sleeper.sleep();

  auto now = fakeclock::now();
  EXPECT_LE(preempt_time + 1, now);
}
