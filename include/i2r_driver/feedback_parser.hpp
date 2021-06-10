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
#include <tf2/LinearMath/Matrix3x3.h>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>
// #define TROUBLESHOOT false

namespace mrccc_utils {
namespace feedback_parser {

Json::Value string_to_json_parser(std::string text);

void RobotStateUpdate(const std::string& str, 
    rmf_fleet_msgs::msg::FleetState& fs_msg,
    int& path_completion_status);

} // feedback_parser
} // mrccc_utils