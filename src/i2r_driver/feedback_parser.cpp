#include <i2r_driver/feedback_parser.hpp>

namespace mrccc_utils {
namespace feedback_parser {

rmf_fleet_msgs::msg::FleetState _fleet_state;
rmf_fleet_msgs::msg::RobotState _robot_state;
// rmf_fleet_msgs::msg::RobotMode  _robot_mode;
rmf_fleet_msgs::msg::Location   _location; //FILL IN PATH REQUEST RECIEVED HERE!!

Json::Value string_to_json_parser(std::string text) {
    const auto text_len = static_cast<int>(text.length());
    std::string err;
    Json::Value root;
    Json::CharReaderBuilder builder;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(text.c_str(), text.c_str() + text_len, &root, &err)) {
        std::cout << "error" << std::endl;
    }
    return root;
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
    _robot_state.model              = "empty"; // Can be input later by the fleet adapter
    _robot_state.task_id            = "empty"; // Will be known by the fleet adapter 
    _robot_state.seq                = 1; // Filler for now

    _robot_state.mode.mode          = rmf_fleet_msgs::msg::RobotMode::MODE_IDLE; // Should be filled by statuspub
    _robot_state.battery_percent    =0; // Should be filled by statuspub
    _robot_state.location.x         = std::move(obj["payload"]["pose"]["pose"]["position"]["x"].asFloat());
    _robot_state.location.y         = std::move(obj["payload"]["pose"]["pose"]["position"]["y"].asFloat());
    _robot_state.location.yaw       = std::move(obj["payload"]["pose"]["pose"]["orientation"]["z"].asFloat());
    _robot_state.location.level_name= ""; //std::move(obj["payload"]["pose"]["pose"]["position"]["x"].asFloat());
    _robot_state.location.index     = 0; //std::move(obj["payload"]["pose"]["pose"]["position"]["x"].asFloat());
    _robot_state.path               = {}; // FILL IN PATH REQUEST RECIEVED HERE!!
    
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
    _robot_state.model              = "empty"; // Can be input later by the fleet adapter
    _robot_state.task_id            = "empty"; // Will be known by the fleet adapter
    _robot_state.seq                = 1; // Filler for now

    _robot_state.mode.mode          = rmf_fleet_msgs::msg::RobotMode::MODE_IDLE;
    _robot_state.battery_percent    =0;

    _robot_state.location.x         = std::move(obj["payload"]["pose"]["pose"]["position"]["x"].asFloat());
    _robot_state.location.y         = std::move(obj["payload"]["pose"]["pose"]["position"]["y"].asFloat());
    _robot_state.location.yaw       = std::move(obj["payload"]["pose"]["pose"]["position"]["yaw"].asFloat());
    _robot_state.location.level_name= ""; // std::move(obj["payload"]["pose"]["pose"]["position"]["level_name"].asString());
    _robot_state.location.index     = 0; // std::move(obj["payload"]["pose"]["pose"]["position"]["index"].asFloat());
    _robot_state.path.emplace_back(); // FILL IN PATH REQUEST RECIEVED HERE!!

    _fleet_state.name               = std::string("Magni");
    _fleet_state.robots.emplace_back(_robot_state);

    return _fleet_state;
}

rmf_fleet_msgs::msg::FleetState RobotStateUpdate(std::string Jstring)
{
    Json::Value obj;
    rmf_fleet_msgs::msg::FleetState fs;
    
    obj = string_to_json_parser(Jstring);
    status_id = (StatusType)obj["header"]["status_id"].asInt();

    switch(status_id)
    {
    case kStatusStatusPub:
        fs = json_statuspub_to_fleetstate(obj);
        return fs;
        
    case kStatusMoveBaseFootprint:
        // empty for now
        break;

    case kStatusCurrentCompletedSubMission:
        // empty for now
        break;

    case kStatusAMCLPose:
        // if (obj.getMemberNames().empty() ) {
        // std::cout<<"Json obj is empty!"<<std::endl;
        // return 0;
        // }
        fs = json_amclpose_to_fleetstate(obj);
        return fs;

    case kStatusUnknown:
        // empty for now
        break;
    }


}

} // feedback_parser
} // mrccc_utils