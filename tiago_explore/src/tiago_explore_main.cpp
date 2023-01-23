#include <memory>

#include "tiago_explore/See.hpp"
#include "tiago_explore/Think.hpp"
#include "tiago_explore/Act.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto see_node = std::make_shared<See>();
  auto think_node = std::make_shared<Think>();
  auto act_node = std::make_shared<Act>();
  
  rclcpp::executors::SingleThreadedExecutor executor; 
  executor.add_node(see_node); 
  executor.add_node(think_node); 
  executor.add_node(act_node);

  executor.spin(); 
  rclcpp::shutdown();

  return 0;
}