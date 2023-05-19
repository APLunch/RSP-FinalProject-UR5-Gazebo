#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/interactive_marker_update.hpp>


namespace MoveRobot{
  class move_robot_node : public rclcpp::Node{
    private:
    
      //move_robot_fnf_subscriber (Fire-and-Forget) is a subscriber that listens for a end effector pose message and move the robot to that pose
      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr move_robot_fnf_subscriber;

      //subscriber to /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update for the end effector pose
      //This provides feasible end effector pose
      rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerUpdate>::SharedPtr robot_interaction_interactive_marker_topic_subscriber;

      //Empty publisher to /rviz/moveit/plan to trigger the planner
      rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr plan_publisher;
      //Empty publisher to /rviz/moveit/execute to trigger the execution
      rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr execute_publisher;

      //Variable to store the feasible end effector pose
      geometry_msgs::msg::PoseStamped feasible_end_effector_pose;


      void robot_interaction_interactive_marker_topic_callback(const visualization_msgs::msg::InteractiveMarkerUpdate::SharedPtr msg){
        //Upon receiving a message, store the feasible end effector pose
        feasible_end_effector_pose.pose = msg->poses[0].pose;
      }

      void move_robot_fnf_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Received end effector pose message");
        //Invoke /rviz/moveit/move_marker/goal_gripper_pick topic to move the planner to the desired pose
        //Publish the desired pose to /rviz/moveit/move_marker/goal_gripper_pick topic
        //Create a temporary publisher
        auto temp_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/rviz/moveit/move_marker/goal_gripper_pick", 10);
        //Publish the message once
        temp_publisher->publish(*msg);
        //Wait for 0.5 seconds
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        //Publishe an Empty message to /rviz/moveit/plan to invoke the planner
        RCLCPP_INFO(this->get_logger(), "Planning...");
        plan_publisher->publish(std_msgs::msg::Empty());
        //Wait for 0.5 seconds
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        //Publish an Empty message to /rviz/moveit/execute to invoke the execution
        RCLCPP_INFO(this->get_logger(), "Executing...");
        execute_publisher->publish(std_msgs::msg::Empty());
      }

    public:
      move_robot_node() : Node("move_robot_node"){
        //Setup the publishers and subscribers
        //Move robot fnf subscriber
        move_robot_fnf_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_robot/move_robot_fnf", 10, std::bind(&move_robot_node::move_robot_fnf_callback, this, std::placeholders::_1));
        //The publisher that gets feasible end effector pose
        robot_interaction_interactive_marker_topic_subscriber = this->create_subscription<visualization_msgs::msg::InteractiveMarkerUpdate>("rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update", 10, std::bind(&move_robot_node::robot_interaction_interactive_marker_topic_callback, this, std::placeholders::_1));
        //The publisher to trigger the planner
        plan_publisher = this->create_publisher<std_msgs::msg::Empty>("/rviz/moveit/plan", 10);
        //The publisher to trigger the execution
        execute_publisher = this->create_publisher<std_msgs::msg::Empty>("/rviz/moveit/execute", 10);
      }
  };
}

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveRobot::move_robot_node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
  
