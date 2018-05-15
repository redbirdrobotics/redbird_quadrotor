#include <rr/mavros_util/mavros_util.hpp>

namespace rr {
namespace mavros_util {

mavros_mode_adapter::mavros_mode_adapter(ros::NodeHandle& nh) {
  const uint32_t mavros_state_queue_size = 10u;
  auto&& update_state_callback = [&](const mavros_msgs::State::ConstPtr& msg){
    mavros_state_ = *msg;
  };
  state_sub_ = nh.subscribe<mavros_msgs::State>(
    kMavrosStateTopic_Id,
    mavros_state_queue_size,
    update_state_callback
  );

  arming_client
      = nh.serviceClient<mavros_msgs::CommandBool>(kMavrosArmingService_Id);

  set_mode_client
      = nh.serviceClient<mavros_msgs::SetMode>(kMavrosSetmodeService_Id);
}

template <typename CallServer, typename CheckSuccess>
mavros_mode_adapter::server_response
mavros_mode_adapter::call_server(CallServer&& call_server, CheckSuccess&& success) {
  using r = server_response;

  if (!mavros_state_.connected)
    return r::fcu_not_connected;

  auto able_to_reach_server = std::forward<CallServer>(call_server)();
  if (!able_to_reach_server)
    return r::cannot_reach_server;

  return std::forward<CheckSuccess>(success)() ? r::success : r::denied;
}

mavros_mode_adapter::server_response
mavros_mode_adapter::set_armed(bool arm) {
  arm_cmd_.request.value = arm;
  return call_server(
    [&]{ return arming_client.call(arm_cmd_); },
    [&]{ return arm_cmd_.response.success; }
  );
}

mavros_mode_adapter::server_response
mavros_mode_adapter::set_custom_mode(std::string custom_mode_id) {
  set_mode_cmd_.request.custom_mode = custom_mode_id;
  return call_server(
    [&]{ return set_mode_client.call(set_mode_cmd_); },
    [&]{ return set_mode_cmd_.response.mode_sent; }
  );
}

} // namespace mavros_util
} // namespace rr
