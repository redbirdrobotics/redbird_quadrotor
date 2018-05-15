#include "flight_controller.hpp"


namespace rr {
namespace quadrotor {
namespace flightsys {

i_flight_controller::~i_flight_controller() {};
i_quadrotor_interfaces::~i_quadrotor_interfaces() {};

void flight_controller::publish_setpoint() {
  while (!destructor_called_) {
    {
      lock_guard lock_target{ target_position_mutex_ };
      //config_->publish_setpoint_position(target_position_);
    } // why extra scope? --> to unlock mutex ASAP

    //config_->wait_between_publish();
  }
}

flight_controller::~flight_controller() {
  destructor_called_ = true;
  publish_until_destruct_thread.join();
}

} // namespace flightsys
} // namespace quadrotor
} // namespace rr
