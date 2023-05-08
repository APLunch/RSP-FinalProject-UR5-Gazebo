#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <thread>

class GPTBridgeNode : public rclcpp::Node {
public:
  GPTBridgeNode()
      : Node("gpt_bridge") {
    // Create a subscription to the topic /commands_to_bridge
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/commands_to_bridge", 10000, std::bind(&GPTBridgeNode::command_callback, this, std::placeholders::_1));
  }

private:
  void command_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string command_string = msg->data;
    std::vector<std::string> commands = split_commands(command_string);
    for (const auto& command : commands) {
      execute_command(command);
    }
  }

  std::vector<std::string> split_commands(const std::string& command_string) {
    std::vector<std::string> commands;
    std::stringstream ss(command_string);
    std::string command;

    while (std::getline(ss, command)) {
      commands.push_back(command);
    }
    return commands;
  }

  void execute_command(const std::string& command) {
    if (command.find("adjust_target") != std::string::npos) {
      std::string target;
      std::vector<float> coordinates(3);
      sscanf(command.c_str(), "adjust_target:%[^;];[%f %f %f]", target.data(), &coordinates[0], &coordinates[1], &coordinates[2]);
      RCLCPP_INFO(this->get_logger(), "Adjusting target %s to coordinates [%f, %f, %f]", target.c_str(), coordinates[0], coordinates[1], coordinates[2]);
      
      // Add code to execute adjust_target command
      ///////////////////////////////////////////////////////////////////////////////////
      std::this_thread::sleep_for(std::chrono::seconds(3));
      ///////////////////////////////////////////////////////////////////////////////////

    } else if (command == "move_robot") {
      RCLCPP_INFO(this->get_logger(), "Moving robot");

      // Add code to execute move_robot command
      ///////////////////////////////////////////////////////////////////////////////////
      std::this_thread::sleep_for(std::chrono::seconds(3));
      ///////////////////////////////////////////////////////////////////////////////////

    } else if (command.find("move_gripper") != std::string::npos) {
      int open;
      sscanf(command.c_str(), "move_gripper:%d", &open);
      RCLCPP_INFO(this->get_logger(), "Moving gripper: %s", open ? "1" : "0");

      // Add code to execute move_gripper command
      ///////////////////////////////////////////////////////////////////////////////////
      std::this_thread::sleep_for(std::chrono::seconds(3));
      ///////////////////////////////////////////////////////////////////////////////////
      
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown command: %s", command.c_str());
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPTBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
