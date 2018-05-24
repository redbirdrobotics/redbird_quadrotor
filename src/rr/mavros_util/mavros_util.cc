#include <rr/mavros_util/mavros_util.hpp>

namespace rr {
namespace mavros_util {

namespace px4 {

const std::unordered_map<operating_mode, mavros_state_mode_string>
    operating_mode_strings {
  { operating_mode::MANUAL, "MANUAL" },
  { operating_mode::ACRO, "ACRO" },
  { operating_mode::ALTCTL, "ALTCTL" },
  { operating_mode::POSCTL, "POSCTL" },
  { operating_mode::OFFBOARD, "OFFBOARD" },
  { operating_mode::STABILIZED, "STABILIZED" },
  { operating_mode::RATTITUDE, "RATTITUDE" },
  { operating_mode::AUTO_MISSION, "AUTO.MISSION" },
  { operating_mode::AUTO_LOITER, "AUTO.LOITER" },
  { operating_mode::AUTO_RTL, "AUTO.RTL" },
  { operating_mode::AUTO_LAND, "AUTO.LAND" },
  { operating_mode::AUTO_RTGS, "AUTO.RTGS" },
  { operating_mode::AUTO_READY, "AUTO.READY" },
  { operating_mode::AUTO_TAKEOFF, "AUTO.TAKEOFF" }
};


namespace detail {

std::unordered_map<mavros_state_mode_string, operating_mode>
    init_operating_mode_map() {
  std::unordered_map<mavros_state_mode_string, operating_mode> map{};
  map.reserve(operating_mode_strings.size());

  for (const auto& mode_and_string : operating_mode_strings)
    map.emplace(mode_and_string.second, mode_and_string.first);

  return map;
}

} // namespace detail


const std::unordered_map<mavros_state_mode_string, operating_mode>
  operating_modes_by_string = detail::init_operating_mode_map();


} // namespace px4


using server_response = mavros_adapter::server_response;

mavros_adapter::~mavros_adapter() {}

template <typename CallServer, typename CheckSuccess>
server_response
mavros_adapter::call_server(CallServer&& call_server, CheckSuccess&& success) {
  using r = server_response;

  if (!mavros_state_.connected)
    return r::fcu_not_connected;

  auto able_to_reach_server = std::forward<CallServer>(call_server)();
  if (!able_to_reach_server)
    return r::cannot_reach_server;

  return std::forward<CheckSuccess>(success)() ? r::success : r::denied;
}

server_response
mavros_adapter::set_armed(bool arm) {
  arm_cmd_.request.value = arm;
  return call_server(
    [&]{ return arming_client.call(arm_cmd_); },
    [&]{ return arm_cmd_.response.success; }
  );
}

mavros_adapter::mavros_adapter(ros::NodeHandle& nh) {
  const uint32_t mavros_state_queue_size = 10u;
  state_sub_ = nh.subscribe<mavros_msgs::State>(
    api_ids::topics::published::kState,
    mavros_state_queue_size,
    [&](const mavros_msgs::State::ConstPtr& msg){ mavros_state_ = *msg; }
  );

  arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
    api_ids::services::kArming
  );

  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
    api_ids::services::kSetmode
  );
}

px4::operating_mode
mavros_adapter::mode() const {
  auto itr = px4::operating_modes_by_string.find(mavros_state_.mode);
  auto end = px4::operating_modes_by_string.cend();
  if (itr == end)
    return px4::operating_mode::MODE_UNSUPPORTED;

  auto mode = itr->second;
  return mode;
}

server_response
mavros_adapter::set_mode(px4::operating_mode mode) {
  operating_mode_cmd_.request.custom_mode = px4::operating_mode_strings.at(mode);
  return call_server(
    [&]{ return set_mode_client.call(operating_mode_cmd_); },
    [&]{ return operating_mode_cmd_.response.mode_sent; }
  );
}


} // namespace mavros_util
} // namespace rr
