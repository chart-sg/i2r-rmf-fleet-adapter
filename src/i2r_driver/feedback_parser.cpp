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

rmf_fleet_msgs::msg::FleetState json_amclpose_to_fleetstate(Json::Value& obj, std::string str, 
    rmf_fleet_msgs::msg::FleetState &fs)

{
    std::cout<<str<<std::endl;

    #ifdef TROUBLESHOOT 
    std::cout<<"ori_w"<<obj["payload"]["pose"]["pose"]["orientation"]["w"].asFloat()<<std::endl;
    std::cout<<"ori_x"<<obj["payload"]["pose"]["pose"]["orientation"]["x"].asFloat()<<std::endl;
    std::cout<<"ori_y"<<obj["payload"]["pose"]["pose"]["orientation"]["y"].asFloat()<<std::endl;
    std::cout<<"ori_z"<<obj["payload"]["pose"]["pose"]["orientation"]["z"].asFloat()<<std::endl;
    std::cout<<"pos_x"<<obj["payload"]["pose"]["pose"]["position"]["x"].asFloat()<<std::endl;
    std::cout<<"pos_y"<<obj["payload"]["pose"]["pose"]["position"]["y"].asFloat()<<std::endl;
    std::cout<<"pos_z"<<obj["payload"]["pose"]["pose"]["position"]["z"].asFloat()<<std::endl;
    #endif

    _robot_state.name               = std::move(obj["header"]["clientname"].asString());
    _robot_state.model              = "empty"; // ??????
    _robot_state.task_id            = "empty"; // ??????
    _robot_state.seq                = obj["payload"]["mission_id"].asInt64();
    _robot_state.mode.mode          = rmf_fleet_msgs::msg::RobotMode::MODE_IDLE;
    _robot_state.battery_percent    = 0;
    _robot_state.location.x         = obj["payload"]["pose"]["pose"]["position"]["x"].asFloat();
    _robot_state.location.y         = obj["payload"]["pose"]["pose"]["position"]["y"].asFloat();

    // Need to convert this to Euler degrees. This is in quaternions for now
    _robot_state.location.yaw       = obj["payload"]["pose"]["pose"]["orientation"]["z"].asFloat(); 

    _robot_state.location.index     = obj["payload"]["mission_id"].asInt64();

    // _robot_state.path.emplace_back(); // FILL IN PATH REQUEST RECIEVED HERE!!

    fs.name               = std::string("Magni");
    fs.robots.emplace_back(_robot_state);

    return _fleet_state;
}

rmf_fleet_msgs::msg::FleetState json_statuspub_to_fleetstate(Json::Value& obj)
{
    return _fleet_state;
}

rmf_fleet_msgs::msg::FleetState json_movebasefoot_to_fleetstate(Json::Value& obj)
{
    return _fleet_state;
}

rmf_fleet_msgs::msg::FleetState RobotStateUpdate(std::string str)
{
    Json::Value obj;
    
    obj = string_to_json_parser(str);
    status_id = (StatusType)obj["header"]["status_id"].asInt();

    switch(status_id)
    {
    case kStatusStatusPub: // Does not have pose
    {
        rmf_fleet_msgs::msg::FleetState fs;
        // std::cout<<"kStatusStatusPub"<<std::endl;
        // fs = json_statuspub_to_fleetstate(obj); 
        return fs;
    }
    case kStatusMoveBaseFootprint: // Cant find message type, will have to see from msg->get_payload()
     {
        rmf_fleet_msgs::msg::FleetState fs;
        // std::cout<<"kStatusMoveBaseFootprint"<<std::endl;
        // fs = json_movebasefoot_to_fleetstate(obj);
        return fs;
     }
    case kStatusCurrentCompletedSubMission:
    {
        rmf_fleet_msgs::msg::FleetState fs;
        // std::cout<<"kStatusCurrentCompletedSubMission"<<std::endl;
        return fs;
    }
    case kStatusAMCLPose:
    {
        rmf_fleet_msgs::msg::FleetState fs;
        std::cout<<"kStatusAMCLPose"<<std::endl;
        fs = json_amclpose_to_fleetstate(obj, str, fs);
        std::cout<<"FleetState name "<<fs.name<<std::endl;
        std::cout<<"Mission id"<<fs.robots.at(0).task_id<<std::endl;
        
        std::cout<<"FleetState robot x = "<<fs.robots.at(0).location.x;
        std::cout<<" y = "<<fs.robots.at(0).location.y;
        std::cout<<" z = "<<fs.robots.at(0).location.yaw<<std::endl;
        return fs;
    }
    case kStatusUnknown:
    {
        rmf_fleet_msgs::msg::FleetState fs;
        std::cout<<"kStatusUnknown"<<std::endl;
        return fs;
    }
    }

}

} // feedback_parser
} // mrccc_utils