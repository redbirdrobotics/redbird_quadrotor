#include "mavros_util.hpp"

#include <unordered_map>

#include <string>
namespace rr {
namespace mavros_util {

namespace px4 {

struct enum_hash {
    template <typename T>
    std::size_t operator()(T t) const { return static_cast<std::size_t>(t); }
};

const std::unordered_map<operating_mode, std::string, enum_hash>
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

std::unordered_map<std::string, operating_mode>
    init_operating_mode_map() {
  std::unordered_map<std::string, operating_mode> map{};

  auto itr = operating_mode_strings.cbegin(),
       end = operating_mode_strings.cend();
  for (; itr != end; ++itr) {
    map.emplace(itr->second, itr->first);
  }

  return map;
}

} // namespace detail


const std::unordered_map<std::string, operating_mode>
  operating_modes_by_string = detail::init_operating_mode_map();


operating_mode
string_to_operating_mode(const std::string& mode_string) {
  auto itr = operating_modes_by_string.find(mode_string),
       end = operating_modes_by_string.end();
  if (itr == end) return operating_mode::MODE_UNSUPPORTED;
  return itr->second;
}

std::string
to_string(operating_mode mode) {
  return operating_mode_strings.at(mode);
}


} // namespace px4



using server_response = mavros_adapter::server_response;

mavros_adapter::~mavros_adapter() {}

template <typename CallServer, typename CheckSuccess>
server_response
mavros_adapter::call_server(CallServer&& call_server_func,
                            CheckSuccess&& check_success) {
  using r = server_response;

  if (!mavros_state_.connected)
    return r::fcu_not_connected;

  auto able_to_reach_server = std::forward<CallServer>(call_server_func)();
  if (!able_to_reach_server)
    return r::cannot_reach_server;

  return std::forward<CheckSuccess>(check_success)() ? r::success : r::denied;
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
  auto mode = px4::string_to_operating_mode(mavros_state_.mode.c_str());
  return mode;
}

server_response
mavros_adapter::set_mode(px4::operating_mode mode) {
  operating_mode_cmd_.request.custom_mode = px4::to_string(mode);
  return call_server(
    [&]{ return set_mode_client.call(operating_mode_cmd_); },
    [&]{ return operating_mode_cmd_.response.mode_sent; }
  );
}


} // namespace mavros_util
} // namespace rr
