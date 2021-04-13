#include <rclcpp/rclcpp.hpp>
#include <rmf_fleet_adapter/agv/Adapter.hpp>

struct Connections : public std::enable_shared_from_this<Connections>
{};

std::shared_ptr<Connections> make_fleet(
    const rmf_fleet_adapter::agv::AdapterPtr& adapter)
{
    return 0;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const auto adapter = rmf_fleet_adapter::agv::Adapter::make("fleet_adapter");
  if (!adapter)
    return 1;

  const auto fleet_connections = make_fleet(adapter);
  if (!fleet_connections)
    return 1;

  RCLCPP_INFO(adapter->node()->get_logger(), "Starting Fleet Adapter");

  // Start running the adapter and wait until it gets stopped by SIGINT
  adapter->start().wait();

  RCLCPP_INFO(adapter->node()->get_logger(), "Closing Fleet Adapter");

  rclcpp::shutdown();
}