#include <ros/ros.h>
#include <boost/functional.hpp>
#include <mavros_msgs/PositionTarget.h>

using target_position = mavros_msgs::PositionTarget;

const std::string kRosNodeName = "redbird_quadrotor_flight_controller";
const std::string kMavrosSetpointLocalRawId = "mavros/setpoint_raw/local";
const uint32_t kMavrosSetpointLocalQueueSize = 1;

class mavros_publishers {
 private:
  boost::function<void(const target_position&)> publish_position;

 public:
  mavros_publishers(
    boost::function<void(const target_position&)> publish_position
  ) {

  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, kRosNodeName);
  ros::NodeHandle nh{};
  auto setpoint_raw_publisher = nh.advertise<target_position>(
    kMavrosSetpointLocalRawId,
    kMavrosSetpointLocalQueueSize
  );

  target_position target_position{};

  target_position.header.stamp = ros::Time::now();
  target_position.header.frame_id = "base_link";

  // ignore all but YAW_RATE and velocities x, y, and z
  target_position.type_mask =
      target_position::IGNORE_PX
    | target_position::IGNORE_PY
    | target_position::IGNORE_PZ
    | target_position::IGNORE_AFX
    | target_position::IGNORE_AFY
    | target_position::IGNORE_AFZ
    | target_position::FORCE
    | target_position::IGNORE_YAW;
  // This makes it so our velocities are sent in the UAV frame.
  // If you use FRAME_LOCAL_NED then it will be in map frame
  target_position.coordinate_frame = target_position::FRAME_BODY_NED;

  // Set the velocities we want (it is in m/s)
  // On the real drone send a minimum of 0.3 m/s in x/y (except when you send 0.0 m/s).
  // If the velocity is lower than 0.3 m/s then the UAV can move in any direction!
  target_position.velocity.x = 0.3;
  target_position.velocity.y = 0.0;
  target_position.velocity.z = 0.1;
  // Set the yaw rate (it is in rad/s)
  target_position.yaw_rate = 0.1;

  ros::Rate r(20); // 20 hz
  while (ros::ok()) {
    setpoint_raw_publisher.publish(target_position);
    r.sleep();
  }
}


