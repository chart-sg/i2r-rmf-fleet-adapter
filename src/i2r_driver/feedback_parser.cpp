#include "i2r_driver/feedback_parser.hpp"

namespace mrccc_utils {
namespace feedback_parser {

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

rmf_fleet_msgs::msg::FleetState json_amclpose_to_fleetstate(
    const Json::Value& obj, 
    rmf_fleet_msgs::msg::FleetState &fs)
{
    // Implement move constructor here
    rmf_fleet_msgs::msg::RobotState _robot_state;
    
    // std::cout<<"Before converstion"
    //     <<" Z: "<<obj["payload"]["pose"]["pose"]["orientation"]["z"].asFloat()
    //     <<" W: "<<obj["payload"]["pose"]["pose"]["orientation"]["w"].asFloat()
    //     <<std::endl;
    
    // Creating quaternion class
    tf2::Quaternion q( obj["payload"]["pose"]["pose"]["orientation"]["x"].asFloat(),
		obj["payload"]["pose"]["pose"]["orientation"]["y"].asFloat(),
		obj["payload"]["pose"]["pose"]["orientation"]["z"].asFloat(),
		obj["payload"]["pose"]["pose"]["orientation"]["w"].asFloat() 
	);
    q=q.normalize();
    tf2::Matrix3x3 m(q);

    _robot_state.name               = "magni";//std::move(obj["header"]["clientname"].asString());
    _robot_state.model              = "empty"; // ??????
    _robot_state.task_id            = "empty"; // ??????
    _robot_state.seq                = obj["payload"]["seq"].asUInt64(); // Increments for every new message
    _robot_state.mode.mode          = rmf_fleet_msgs::msg::RobotMode::MODE_IDLE;
    _robot_state.battery_percent    = 100;

    rclcpp::Time t (obj["payload"]["header"]["stamp"]["secs"].asFloat(),
        obj["payload"]["header"]["stamp"]["nsecs"].asFloat()
    );
    _robot_state.location.t         = t;
        
    _robot_state.location.x         = obj["payload"]["pose"]["pose"]["position"]["x"].asFloat();
    _robot_state.location.y         = obj["payload"]["pose"]["pose"]["position"]["y"].asFloat();
    
    double r,p,y;
    m.getRPY(r, p ,y);

    // std::cout<<"After converstion"
    //     <<" Yaw: "<<y
    //     <<std::endl;

    _robot_state.location.yaw       = y; 
    _robot_state.location.level_name= "B1"; 
    _robot_state.location.index     = 0;

    fs.name               = std::string("tinyRobot");
    fs.robots.assign(1, _robot_state);

    return fs;
}

void json_path_completion_status(const Json::Value& obj,
    int & path_compeletion_status)
{
    const Json::Value& state = obj["payload"]["submission_states"];
    std::cout<<"submission state is: "<<state[0].asInt()<<std::endl;;
    
    if (state[0].asInt() <4) return;
    else path_compeletion_status = state[0].asInt();
}

void RobotStateUpdate(
    const std::string& str, 
    rmf_fleet_msgs::msg::FleetState& fs_msg,
    int& path_compeletion_status)
{
    Json::Value obj;
    obj = string_to_json_parser(str);
    status_id = (StatusType)obj["header"]["status_id"].asInt();

    switch(status_id)
    {
    case kStatusStatusPub: // Does not have pose
    {
        // json_get_path_recieved_ack(obj);
        json_path_completion_status(obj, path_compeletion_status);
        break;
    }
    case kStatusMoveBaseFootprint: // Cant find message type, will have to see from msg->get_payload()
     {
        // std::cout<<"Feedback -> kStatusMoveBaseFootprint"<<std::endl;
        // fs = json_movebasefoot_to_fleetstate(obj);
        break;
     }
    case kStatusCurrentCompletedSubMission:
    {
        // std::cout<<"Request completed"<<std::endl;
        // std::cout<<str<<std::endl;
        rmf_fleet_adapter::agv::RobotCommandHandle::RequestCompleted();
        break;
    }
    case kStatusAMCLPose:
    {
        fs_msg = json_amclpose_to_fleetstate(
            obj, 
            fs_msg);
        break;
    }
    case kStatusUnknown:
    {
        // std::cout<<"Feedback -> kStatusUnknown"<<std::endl;
        break;
    }
    default:
    {
        // std::cout<<"Feedback -> Default"<<std::endl;
        break;
    }
    }
}

} // feedback_parser
} // mrccc_utils