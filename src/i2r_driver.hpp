
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <Eigen/Geometry>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include "rclcpp/rclcpp.hpp"



namespace i2r_driver {


struct I2RPathInfo
{
    std::string task_id;
    std::vector<rmf_fleet_msgs::msg::Location> to_i2r_waypoint; 
};

void send_i2r_line_following_mission(rclcpp::Node* node, I2RPathInfo& info);

void send_i2r_docking_mission(rclcpp::Node* node, std::string task_id);

void get_map_transfomation_param(rclcpp::Node* node, std::vector<double>& map_coordinate_transformation);

void transform_i2r_to_rmf(
      rclcpp::Node* node,
      const rmf_fleet_msgs::msg::Location& _fleet_frame_location, 
      rmf_fleet_msgs::msg::Location& _rmf_frame_location) ;

void transform_rmf_to_i2r(
      rclcpp::Node* node,
      const rmf_fleet_msgs::msg::Location& _rmf_frame_location, 
      rmf_fleet_msgs::msg::Location& _fleet_frame_location) ;
  

double get_yaw_from_quat(tf2::Quaternion quat);

tf2::Quaternion get_quat_from_yaw(double yaw);


} //namespace i2r driver