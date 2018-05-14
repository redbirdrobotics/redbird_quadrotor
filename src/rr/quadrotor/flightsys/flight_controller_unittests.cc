#include <gmock/gmock.h>

#include <rr/quadrotor/flightsys/flight_controller.hpp>

using namespace rr;
using namespace rr::quadrotor;
using namespace rr::quadrotor::flightsys;

class MockQuadrotorInterfaces : public i_quadrotor_interfaces {
 public:
  MOCK_METHOD1(publish_setpoint_position,
    void(const mavros_util::target_position& target));

  MOCK_CONST_METHOD0(get_current_position,
    geometry_msgs::PoseStamped());
};

TEST(FlightController, config) {

}
