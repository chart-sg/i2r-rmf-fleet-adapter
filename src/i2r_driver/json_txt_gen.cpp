#include <jsoncpp/json/json.h>
#include <iostream>
#include <memory>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <fstream>
// #include <i2r_driver.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>

/*
 * Example Usage:
 * $g++ json_txt_gen.cpp -ljsoncpp -std=c++11 -o test_json_gen
 * $./test_json_gen
 */
#define TROUBLESHOOT true


class Json_to_FleetState
{
private:
    rmf_fleet_msgs::msg::FleetState _fleet_state;
    rmf_fleet_msgs::msg::RobotState _robot_state;
    // rmf_fleet_msgs::msg::RobotMode  _robot_mode;
    rmf_fleet_msgs::msg::Location   _location; //FILL IN PATH REQUEST RECIEVED HERE!!

public:

    Json_to_FleetState()
    {

        std::cout<<"Json_to_Fleet Entity created"<<std::endl;

    }

    ~Json_to_FleetState()
    {
        std::cout<<"Json_to_Fleet Entity Destructed"<<std::endl;
    }

    rmf_fleet_msgs::msg::FleetState json_amclpose_to_fleetstate(Json::Value& obj)
    {

        #ifdef TROUBLESHOOT
        std::cout<<"ori_w"<<obj["payload"]["pose"]["pose"]["orientation"]["w"]<<std::endl;
        std::cout<<"ori_x"<<obj["payload"]["pose"]["pose"]["orientation"]["x"]<<std::endl;
        std::cout<<"ori_y"<<obj["payload"]["pose"]["pose"]["orientation"]["y"]<<std::endl;
        std::cout<<"ori_z"<<obj["payload"]["pose"]["pose"]["orientation"]["z"]<<std::endl;
        std::cout<<"pos_x"<<obj["payload"]["pose"]["pose"]["position"]["x"]<<std::endl;
        std::cout<<"pos_y"<<obj["payload"]["pose"]["pose"]["position"]["y"]<<std::endl;
        std::cout<<"pos_z"<<obj["payload"]["pose"]["pose"]["position"]["z"]<<std::endl;
        #endif

        _robot_state.name               = std::move(obj["header"]["clientname"].asString());
        _robot_state.model              = "empty"; // ??????
        _robot_state.task_id            = "empty"; // ??????
        _robot_state.seq                = 1; // Filler for now

        _robot_state.mode.mode          = rmf_fleet_msgs::msg::RobotMode::MODE_IDLE;
        _robot_state.battery_percent    =0;

        _robot_state.location.x         = std::move(obj["payload"]["pose"]["pose"]["position"]["x"].asFloat());
        _robot_state.location.y         = std::move(obj["payload"]["pose"]["pose"]["position"]["x"].asFloat());
        _robot_state.location.yaw       = std::move(obj["payload"]["pose"]["pose"]["position"]["x"].asFloat());
        _robot_state.location.level_name= std::move(obj["payload"]["pose"]["pose"]["position"]["x"].asFloat());
        _robot_state.location.index     = std::move(obj["payload"]["pose"]["pose"]["position"]["x"].asFloat());
        _robot_state.path.emplace_back(); // FILL IN PATH REQUEST RECIEVED HERE!!

        _fleet_state.name               = std::string("Magni");
        _fleet_state.robots.emplace_back(_robot_state);

        return _fleet_state;
    }

    rmf_fleet_msgs::msg::FleetState json_statuspub_to_fleetstate(Json::Value& obj)
    {

        #ifdef TROUBLESHOOT
        std::cout<<"ori_w"<<obj["payload"]["pose"]["pose"]["orientation"]["w"]<<std::endl;
        std::cout<<"ori_x"<<obj["payload"]["pose"]["pose"]["orientation"]["x"]<<std::endl;
        std::cout<<"ori_y"<<obj["payload"]["pose"]["pose"]["orientation"]["y"]<<std::endl;
        std::cout<<"ori_z"<<obj["payload"]["pose"]["pose"]["orientation"]["z"]<<std::endl;
        std::cout<<"pos_x"<<obj["payload"]["pose"]["pose"]["position"]["x"]<<std::endl;
        std::cout<<"pos_y"<<obj["payload"]["pose"]["pose"]["position"]["y"]<<std::endl;
        std::cout<<"pos_z"<<obj["payload"]["pose"]["pose"]["position"]["z"]<<std::endl;
        #endif

        _fleet_state.name = std::string("Magni");

        _robot_state.name               = std::move(obj["header"]["clientname"].asString());
        _robot_state.model              = "empty"; // ??????
        _robot_state.task_id            = "empty"; // ??????
        _robot_state.seq                = 1; // Filler for now

        _robot_state.mode.mode          = rmf_fleet_msgs::msg::RobotMode::MODE_IDLE;
        _robot_state.battery_percent    =0;

        _robot_state.location.x         = std::move(obj["payload"]["pose"]["pose"]["position"]["x"].asFloat());
        _robot_state.location.y         = std::move(obj["payload"]["pose"]["pose"]["position"]["y"].asFloat());
        _robot_state.location.yaw       = std::move(obj["payload"]["pose"]["pose"]["position"]["yaw"].asFloat());
        _robot_state.location.level_name= std::move(obj["payload"]["pose"]["pose"]["position"]["level_name"].asFloat());
        _robot_state.location.index     = std::move(obj["payload"]["pose"]["pose"]["position"]["index"].asFloat());
        _robot_state.path.emplace_back(); // FILL IN PATH REQUEST RECIEVED HERE!!

        _fleet_state.name               = std::string("Magni");
        _fleet_state.robots.emplace_back(_robot_state);

        return _fleet_state;
    }
};

// This portion will be removed later, for testing now
int main()
{
    rmf_fleet_msgs::msg::FleetState fs;
    Json_to_FleetState j_fs;

    // Simulate reading a json
    std::ifstream ifs("/home/jh/Dev/barkin_mad_ws/src/mrccc_utils/resource/pose.json");
    Json::Reader reader;
    Json::Value obj;
    reader.parse(ifs, obj); // reader can also read strings

    // Safety check: If there is nothing in the Json file, throw an error
    if (obj.getMemberNames().empty() ) {
        std::cout<<"Json obj is empty!"<<std::endl;
        return 0;
    }

    /// Status codes for what type of status is returned back from robot
    enum StatusType
    {
        /// Robot status info
        kStatusStatusPub = 0,

        /// move base footprint
        kStatusMoveBaseFootprint,

        /// current completed sub mission status
        kStatusCurrentCompletedSubMission,

        /// amcl pose message
        kStatusAMCLPose,

        /// unknown status type
        kStatusUnknown = 254
    } status_id;

    status_id = (StatusType)obj["header"]["status_id"].asInt();

    switch(status_id)
    {
    case kStatusStatusPub:
        j_fs.json_statuspub_to_fleetstate(obj);
        return 0;
        
    case kStatusMoveBaseFootprint:
        // empty for now
        return 0;

    case kStatusCurrentCompletedSubMission:
        // empty for now
        return 0;

    case kStatusAMCLPose:
        // if (obj.getMemberNames().empty() ) {
        // std::cout<<"Json obj is empty!"<<std::endl;
        // return 0;
        // }
        j_fs.json_amclpose_to_fleetstate(obj);
        return 0;

    case kStatusUnknown:
        // empty for now
        return 0;
    }
}