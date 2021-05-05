#pragma once


#include "rclcpp/time.hpp"
#include <jsoncpp/json/json.h>
#include <iostream>
#include <memory>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>

// #define TROUBLESHOOT false

namespace mrccc_utils {
namespace feedback_parser {

Json::Value string_to_json_parser(std::string text);

rmf_fleet_msgs::msg::FleetState RobotStateUpdate(std::string Jstring);

} // feedback_parser
} // mrccc_utils