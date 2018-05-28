#include <rr/mavros_util.hpp>
#include <rr/quadrotor/flight_sequence/circle.hpp>

#include <ros/ros.h>

#include <thread>


auto
get_ros_async_spinner() {
  const std::uint32_t num_cores = std::thread::hardware_concurrency();
  auto spinner = ros::AsyncSpinner{num_cores};
  return spinner;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "quadrotor_controller");
  ros::NodeHandle nh;
  auto ros_spinner = get_ros_async_spinner();
  ros_spinner.start();

  using namespace rr;
  using namespace rr::mavros_util;
  using namespace rr::quadrotor;
  using namespace rr::quadrotor::flight_sequence;

  auto mavros = std::make_shared<rr::mavros_util::mavros_adapter>(nh);
  mavros->arm();
  mavros->set_mode(rr::mavros_util::px4::operating_mode::OFFBOARD);

  auto setpoints = default_mavros_setpoints_message();
  auto update_setpoints = [&](const auto& quadrotor_setpoints) {
    setpoints << quadrotor_setpoints;
    ROS_INFO_STREAM(setpoints);
    mavros->set_setpoints(setpoints);
  };

  circle(update_setpoints);

  ros::waitForShutdown();
  return 0;
}
