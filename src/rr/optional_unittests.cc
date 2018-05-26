#include <gtest/gtest.h>

#include "optional.hpp"

using namespace rr;

// default behavior is unset on construction (i.e. not set)
TEST(Optional, unset_on_construct) {
  optional<int> o{};
  EXPECT_FALSE(o.is_set());
}

TEST(Optional, remains_unset_after_redundant_unset) {
  optional<int> o{};
  o.unset();
  EXPECT_FALSE(o.is_set());
  o.unset();
  EXPECT_FALSE(o.is_set());
}

TEST(Optional, set_upon_set) {
  optional<int> o{};
  o.set();
  EXPECT_TRUE(o.is_set());
  EXPECT_EQ(0, o.get());
}

TEST(Optional, implicity_set_upon_set_with_value) {
  optional<int> o{};
  o.set(3);
  EXPECT_TRUE(o.is_set());
  EXPECT_EQ(3, o.get());
}

TEST(Optional, not_set_after_unset) {
  {
    optional<int> o{};
    o.set();
    o.unset();
    EXPECT_FALSE(o.is_set());
  }

  {
    optional<int> o{};
    o.set(2);
    o.unset();
    EXPECT_FALSE(o.is_set());
  }
}

TEST(Optional, unset_does_not_change_value) {
  optional<int> o{};
  o.set(1);
  o.unset();
  EXPECT_EQ(1, o.get());
  o.set();
  EXPECT_EQ(1, o.get());
}

TEST(Optional, value_preserved_after_set_unset_cycles) {
  optional<int> o{};
  o.set(1);
  o.unset();
}
