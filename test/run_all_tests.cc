/**
 * @file run_all_tests.cc
 *    Run all tests in this package.
 *    See files named *_unittests.cc for unittests.
 */

#include <gmock/gmock.h>


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
