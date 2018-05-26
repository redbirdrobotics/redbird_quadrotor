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

template <typename CallServer, typename CheckSuccess>
server_response
mavros_adapter::call_server(CallServer&& call_server_func,
                            CheckSuccess&& check_success) const {
  using r = server_response;

  if (!mavros_state().connected)
    return r::fcu_not_connected;

  auto able_to_reach_server = std::forward<CallServer>(call_server_func)();
  if (!able_to_reach_server)
    return r::cannot_reach_server;

  return std::forward<CheckSuccess>(check_success)() ? r::success : r::denied;
}

server_response
mavros_adapter::enable_current_arming_command() {
  server_response response;
  using cmd_t = decltype(arm_cmd_)::value_t;
  arm_cmd_.alter([&](cmd_t& cmd) {
    response = call_server(
      [&] { return arming_client_.call(cmd); },
      [&] { return cmd.response.success; }
    );
  });
  return response;
}

server_response
mavros_adapter::enable_current_mode() {
  server_response result;
  using cmd_t = decltype(operating_mode_cmd_)::value_t;
  operating_mode_cmd_.alter([&](cmd_t& cmd) {
    result = call_server(
      [&]{ return set_mode_client_.call(cmd); },
      [&]{ return cmd.response.mode_sent; }
    );
  });
  return result;
}

void
mavros_adapter::set_armed(bool arm) {
  auto cmd = arm_cmd();
  cmd.request.value = arm;
  set_arm_cmd(cmd);
}

mavros_adapter::mavros_adapter(ros::NodeHandle& nh) {
  const uint32_t mavros_state_queue_size = 10u;
  state_sub_ = nh.subscribe<mavros_msgs::State>(
    api_ids::topics::published::kState,
    mavros_state_queue_size,
    [&](const mavros_msgs::State::ConstPtr& msg){
      mavros_state_.set(*msg);
    }
  );

  arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>(
    api_ids::services::kArming
  );

  set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>(
    api_ids::services::kSetmode
  );

  const uint32_t mavros_setpoint_queue_size = 10;
  setpoint_publisher_ = nh.advertise<target_position>(
    mavros_util::api_ids::topics::subscribed::kSetpointRawLocal,
    mavros_setpoint_queue_size
  );

  auto set_mode = [&] {
    while (!destructor_called_) {
      bool server_called = publish_target_mode();
      if (server_called) change_mode_delay_.sleep();
      else check_mode_delay_.sleep();
    }
  };
  auto set_armed_cmd = [&] {
    while (!destructor_called_) {
      bool server_called = publish_target_arm_cmd();
      if (server_called) arm_delay_.sleep();
      else check_armed_delay_.sleep();
    }
  };
  auto send_setpoints = [&] {
    while (!destructor_called_) {
      publish_setpoints();
      send_setpoints_delay_.sleep();
    }
  };
  persistent_tasks.reserve(3);
  persistent_tasks.emplace_back(std::move(set_mode));
  persistent_tasks.emplace_back(std::move(set_armed_cmd));
  persistent_tasks.emplace_back(std::move(send_setpoints));
}

mavros_adapter::~mavros_adapter() {
  destructor_called_ = true;
  for (auto& t : persistent_tasks)
    t.join();
}

px4::operating_mode
mavros_adapter::mode() const {
  auto mode = px4::string_to_operating_mode(mavros_state_.get().mode);
  ros::spinOnce();
  return mode;
}

void
mavros_adapter::set_mode(px4::operating_mode mode) {
  current_target_mode_ = mode;
  auto mode2=mavros_msgs::SetMode{};
  mode2.request.custom_mode = std::string{};
  operating_mode_cmd_.alter([&](auto& cmd) {
    cmd.request.custom_mode = px4::to_string(mode);
  });
}


} // namespace mavros_util
} // namespace rr
