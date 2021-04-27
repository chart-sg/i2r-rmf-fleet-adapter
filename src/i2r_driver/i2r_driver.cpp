#include "i2r_driver/i2r_driver.hpp"

namespace i2r_driver {
      

void send_i2r_line_following_mission(rclcpp::Node* node, I2RPathInfo& info)
{

    RCLCPP_INFO(
        node->get_logger(), "---------------> Task id (line following): [%s]", info.task_id.c_str());

}

void send_i2r_docking_mission(rclcpp::Node* node, std::string task_id)
{

    RCLCPP_INFO(
        node->get_logger(), "---------------> Task id (docking): [%s]", task_id.c_str());
    
}


void get_map_transfomation_param(rclcpp::Node* node, std::vector<double>& map_coordinate_transformation)
{
    node->get_parameter("map_coordinate_transformation", map_coordinate_transformation);
    
    /*
    RCLCPP_INFO(
    node->get_logger(), "map_coordinate_transformation: (%.3f, %.3f, %.3f, %.3f)",
        map_coordinate_transformation.at(0), map_coordinate_transformation.at(1),map_coordinate_transformation.at(2),map_coordinate_transformation.at(3));
    */
}

//transformation i2r-->rmf
  
void transform_i2r_to_rmf(
    rclcpp::Node* node,
    const rmf_fleet_msgs::msg::Location& _fleet_frame_location,
    rmf_fleet_msgs::msg::Location& _rmf_frame_location) 
{

    std::vector<double> map_coordinate_transformation;
    get_map_transfomation_param(node, map_coordinate_transformation);

    const Eigen::Vector2d translated =
        Eigen::Vector2d(_fleet_frame_location.x, _fleet_frame_location.y)
        - Eigen::Vector2d(
            map_coordinate_transformation.at(0), map_coordinate_transformation.at(1));

    const Eigen::Vector2d rotated =
        Eigen::Rotation2D<double>(-map_coordinate_transformation.at(2)) * translated;

    const Eigen::Vector2d scaled = 1.0 / map_coordinate_transformation.at(3) * rotated;
        
    _rmf_frame_location.x = scaled[0];
    _rmf_frame_location.y = scaled[1];
    _rmf_frame_location.yaw = 
        _fleet_frame_location.yaw - map_coordinate_transformation.at(2);

    _rmf_frame_location.t = _fleet_frame_location.t;
    _rmf_frame_location.level_name = _fleet_frame_location.level_name;
}

  
//transformation rmf-->i2r

void transform_rmf_to_i2r(
    rclcpp::Node* node,
    const rmf_fleet_msgs::msg::Location& _rmf_frame_location,
    rmf_fleet_msgs::msg::Location& _fleet_frame_location) 
{

    std::vector<double> map_coordinate_transformation;
    get_map_transfomation_param(node, map_coordinate_transformation);

    const Eigen::Vector2d scaled = 
        map_coordinate_transformation.at(3) * 
        Eigen::Vector2d(_rmf_frame_location.x, _rmf_frame_location.y);

    const Eigen::Vector2d rotated =
        Eigen::Rotation2D<double>(map_coordinate_transformation.at(2)) * scaled;

    const Eigen::Vector2d translated =
        rotated + 
        Eigen::Vector2d(
            map_coordinate_transformation.at(0), map_coordinate_transformation.at(1));

    _fleet_frame_location.x = translated[0];
    _fleet_frame_location.y = translated[1];
    _fleet_frame_location.yaw = 
        _rmf_frame_location.yaw + map_coordinate_transformation.at(2);

    _fleet_frame_location.t = _rmf_frame_location.t;
    _fleet_frame_location.level_name = _rmf_frame_location.level_name;
}


double get_yaw_from_quat(tf2::Quaternion _quat)
{
  tf2::Matrix3x3 tf2_mat(_quat);
  
  double yaw;
  double pitch;
  double roll;
  tf2_mat.getEulerYPR(yaw, pitch, roll);
  return yaw;

}


tf2::Quaternion get_quat_from_yaw(double _yaw)
{
  tf2::Quaternion quat_tf;
  quat_tf.setRPY(0.0, 0.0, _yaw);
  quat_tf.normalize();

  return quat_tf;

}


} //namespace i2r driver