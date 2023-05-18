#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <thread>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
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
    if (command.find("reposition_robot") != std::string::npos) {
      RCLCPP_INFO(this->get_logger(), "Repositioning robot");
      
      // Add code to execute adjust_target command
      ///////////////////////////////////////////////////////////////////////////////////
      // Create a publisher for the specified topic and message type
    auto parameters_client = this->create_client<rcl_interfaces::srv::SetParameters>("/rtt/set_parameters");
    while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to become available...");
        }
        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        rcl_interfaces::msg::Parameter pos1;
        rcl_interfaces::msg::Parameter pos2;
        rcl_interfaces::msg::Parameter pos3;
        rcl_interfaces::msg::Parameter pos4;
        rcl_interfaces::msg::Parameter pos5;
        rcl_interfaces::msg::Parameter pos6;

        pos1.name = "Desired_joint_positions/shoulder_pan_joint";
        pos2.name = "Desired_joint_positions/shoulder_lift_joint";
        pos3.name = "Desired_joint_positions/elbow_joint";
        pos4.name = "Desired_joint_positions/wrist_1_joint";
        pos5.name = "Desired_joint_positions/wrist_2_joint";
        pos6.name = "Desired_joint_positions/wrist_3_joint";

        pos1.value.double_value = 0.25;
        pos2.value.double_value = -1.5;
        pos3.value.double_value = -1.7;
        pos4.value.double_value = -1.0;
        pos5.value.double_value = 1.5;
        pos6.value.double_value = 0.3;

        pos1.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        pos2.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        pos3.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        pos4.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        pos5.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        pos6.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

        request->parameters.push_back(pos1);
        request->parameters.push_back(pos2);
        request->parameters.push_back(pos3);
        request->parameters.push_back(pos4);
        request->parameters.push_back(pos5);
        request->parameters.push_back(pos6);
        auto result_future = parameters_client->async_send_request(request);
        std::this_thread::sleep_for(std::chrono::seconds(8));
        /*
    auto pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ur5_controller/commands", 10);
    // Create a message
    std_msgs::msg::Float64MultiArray msg;
    // Fill the message
    msg.layout.dim = {}; // Empty as per your example
    msg.layout.data_offset = 0;
    msg.data = {0.25, -1.5, -1.7, -1.0, 1.5, 0.3, -0.3, -0.3};
    // Publish the message
    pub->publish(msg);
    */

      ///////////////////////////////////////////////////////////////////////////////////
    } else if (command.find("move_robot") != std::string::npos) {
      std::string target;
      sscanf(command.c_str(), "move_robot: %s\n", target.data());
      auto parameters_client = this->create_client<rcl_interfaces::srv::SetParameters>("/rtt/set_parameters");
    while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to become available...");
        }
        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        rcl_interfaces::msg::Parameter pos1;
        rcl_interfaces::msg::Parameter pos2;
        rcl_interfaces::msg::Parameter pos3;
        rcl_interfaces::msg::Parameter pos4;
        rcl_interfaces::msg::Parameter pos5;
        rcl_interfaces::msg::Parameter pos6;

        pos1.name = "Desired_joint_positions/shoulder_pan_joint";
        pos2.name = "Desired_joint_positions/shoulder_lift_joint";
        pos3.name = "Desired_joint_positions/elbow_joint";
        pos4.name = "Desired_joint_positions/wrist_1_joint";
        pos5.name = "Desired_joint_positions/wrist_2_joint";
        pos6.name = "Desired_joint_positions/wrist_3_joint";
        std::string str1(target.c_str());
        std::string strg("green_box");
        std::string strb("blue_ball");
        std::string strr("red_ball");
          if(str1.compare(strg)==0){
            RCLCPP_INFO(this->get_logger(), "Begin to move the green box...");
        pos1.value.double_value = 0.65;
        pos2.value.double_value = -2.0;
        pos3.value.double_value = -1.8;
        pos4.value.double_value = -1.0;
        pos5.value.double_value = 1.5;
        pos6.value.double_value = 0.4;
          }
          else if(str1.compare(strb)==0){
            RCLCPP_INFO(this->get_logger(), "Begin to move the blue ball...");
            pos1.value.double_value = 0.2;
            pos2.value.double_value = -1.95;
        pos3.value.double_value = -1.9;
        pos4.value.double_value = -1.0;
        pos5.value.double_value = 1.5;
        pos6.value.double_value = 0.3;
          }
          else if(str1.compare(strr)==0){
            RCLCPP_INFO(this->get_logger(), "Begin to move the red ball...");
            pos1.value.double_value = -0.2;
        pos2.value.double_value = -1.95;
        pos3.value.double_value = -1.9;
        pos4.value.double_value = -0.8;
        pos5.value.double_value = 1.5;
        pos6.value.double_value = -0.3;
          }
        

        pos1.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        pos2.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        pos3.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        pos4.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        pos5.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        pos6.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

        request->parameters.push_back(pos1);
        request->parameters.push_back(pos2);
        request->parameters.push_back(pos3);
        request->parameters.push_back(pos4);
        request->parameters.push_back(pos5);
        request->parameters.push_back(pos6);

        auto result_future = parameters_client->async_send_request(request);
        std::this_thread::sleep_for(std::chrono::seconds(10));
    } else if (command.find("move_gripper") != std::string::npos) {
      int open;
      sscanf(command.c_str(), "move_gripper:%d", &open);
      RCLCPP_INFO(this->get_logger(), "Moving gripper: %s", open ? "1" : "0");
      auto parameters_client = this->create_client<rcl_interfaces::srv::SetParameters>("/rtt/set_parameters");
      while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to become available...");
        }
        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        rcl_interfaces::msg::Parameter left_g;
        rcl_interfaces::msg::Parameter right_g;
        left_g.name = "Control/left_gripper";
        right_g.name = "Control/right_gripper";
        if(open==1){
        left_g.value.double_value = 0.2;
        right_g.value.double_value = 0.2;
        }
        else if(open==0){
        left_g.value.double_value = -0.3;
        right_g.value.double_value = -0.3;
        }
        left_g.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        right_g.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        request->parameters.push_back(left_g);
        request->parameters.push_back(right_g);
        auto result_future = parameters_client->async_send_request(request);

        std::this_thread::sleep_for(std::chrono::seconds(8));
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
