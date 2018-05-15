#include <gmock/gmock.h>

#include <rr/mavros_util/mavros_util.hpp>
#include <rr/quadrotor/flightsys/flight_controller.hpp>

#include <geometry_msgs/PoseStamped.h>

#include <memory>
#include <utility>

using namespace rr;
using namespace rr::mavros_util;
using namespace rr::quadrotor;
using namespace rr::quadrotor::flightsys;

using namespace ::testing;

class MockQuadrotorAdapter : i_quadrotor_adapter {
 public:
  MOCK_METHOD1(publish_setpoint_position,
    void(const mavros_util::target_position& target));

  MOCK_CONST_METHOD0(get_current_position,
    geometry_msgs::PoseStamped());

  MOCK_CONST_METHOD0(get_current_velocity,
    geometry_msgs::TwistStamped());
};

struct vec3 {
  double x;
  double y;
  double z;
};
struct vec2 {
  double x;
  double y;
};

//class waypoint_evaluator {
// public:
//  template <typename Vec3>
//  waypoint_evaluator(
//     Vec&& target,
//     Vec&& allowed_margins)
//    : target_(std::forward<Waypoint>(target))
//    , allowed_margins_(
//        std::forward<AllowedError>(allowed_margins))
//  {}
//};

class MockWaypointEvaluator {
 public:
  MOCK_METHOD1(waypoint_achieved,
    bool(const vec3& current_reported_position));
};

//TEST(FlightController, config) {
//  auto adapter = std::make_unique<MockQuadrotorAdapter>();

//  EXPECT_CALL(*adapter, publish_setpoint_position)
//      .Times()

//  flight_controller fc{quadrotor};
//}







class MockRoomba {
 public:
  MOCK_CONST_METHOD0(get_xy, vec2());
};

TEST(RoombaInteractions, top_touch) {

  MockRoomba tracked_roomba;

  {
    InSequence s;

    for (int i = 1; i <= 100; ++i) {
      decltype(vec2::x) xy = i;
      EXPECT_CALL(tracked_roomba, get_xy())
        .WillOnce(Return(vec2{xy, xy}))
        .RetiresOnSaturation();
    }
  }

}

