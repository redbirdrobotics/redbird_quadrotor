#include "flight_controller.hpp"


namespace rr {
namespace quadrotor {
namespace flightsys {

i_flight_controller::~i_flight_controller() {};
i_quadrotor_interfaces::~i_flight_controller_config() {};

void flight_controller::update_positions() {
  while (!destructor_called_) {
    {
      lock_guard lock_target{ target_position_mutex_ };
      lock_guard lock_current{ current_position_mutex_ };
      config_->publish_setpoint_position(target_position_);
      config_->get_current_position(current_position_);
    } // why extra scope? --> to unlock mutexes ASAP

    config_->wait_between_publish();
  }
}

flight_controller::~flight_controller() {
  destructor_called_ = true;
  publish_until_destruct_thread.join();
}

} // namespace flightsys
} // namespace quadrotor
} // namespace rr
