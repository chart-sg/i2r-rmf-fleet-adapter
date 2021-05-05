#pragma once

#include <jsoncpp/json/json.h>
#include <iostream>
#include <memory>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>
#include <rmf_fleet_msgs/msg/location.hpp>

#define TABLE_LEN 5


namespace mrccc_utils{
namespace mission_gen {

// std::string line_following(int task_id, std::vector<rmf_fleet_msgs::msg::Location> waypoint);
std::string line_following(int& task_id, std::vector<rmf_fleet_msgs::msg::Location> &waypoint);

std::string abort(int task_id);

std::string dock(int task_id);

std::string identifyMe();

std::string SysTest();

std::string initRobotPose();

} // namespace mission_gen
} // namespace mrccc_utils