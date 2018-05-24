#include <gtest/gtest.h>

#include "mavros_util.hpp"

#include <mavros_msgs/PositionTarget.h>

using namespace rr;
using namespace rr::mavros_util;

using target = mavros_msgs::PositionTarget;


TEST(MavrosUtil, ignore_all_but_py) {
  const auto all_but_py =
      target::IGNORE_PX
    | target::IGNORE_PZ
    | target::IGNORE_VX
    | target::IGNORE_VY
    | target::IGNORE_VZ
    | target::IGNORE_AFX
    | target::IGNORE_AFY
    | target::IGNORE_AFZ
    | target::FORCE
    | target::IGNORE_YAW
    | target::IGNORE_YAW_RATE;

  // sanity check:
  EXPECT_EQ(kIgnoreAll ^ target::IGNORE_PY, all_but_py);
  EXPECT_EQ(kIgnoreAll, all_but_py | target::IGNORE_PY);


  auto all_but_py_actual = ignore_all_but(target::IGNORE_PY);
  EXPECT_EQ(all_but_py, all_but_py_actual);
}

// If we "ignore all but `kIgnoreAll`", we should be ignoring nothing!
// Therefore, the typemask should be zero.
TEST(MavrosUtil, ignore_all_but_all) {
  auto all_but_all = ignore_all_but(kIgnoreAll);
  auto zero = decltype(all_but_all){ 0 };
  EXPECT_EQ(zero, all_but_all);
}

// Not the intended usage, but calling ignore_all_but() should ignore everything.
TEST(MavrosUtil, ignore_all_but_nothing) {
  auto all_but_nothing = ignore_all_but();
  EXPECT_EQ(kIgnoreAll, all_but_nothing);
}
