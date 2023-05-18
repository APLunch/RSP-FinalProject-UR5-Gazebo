#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace ur_gazebo
{
    class JointMirror : public rclcpp::Node
    {
        //Declear a subscriber
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
        //Declear a publisher
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mirror_pub;
        public:
            JointMirror(const std::string& name) : Node(name)
            {
                joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
                    "/world/ur_simulation_world/model/ur5/joint_state", 10, std::bind(&JointMirror::joint_state_callback, this, std::placeholders::_1));
                mirror_pub = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
            }

            void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                //Upon receiving a message, remove the gripper joint states and re-publish

                //Create a new message
                auto new_msg = std::make_shared<sensor_msgs::msg::JointState>();
                //Copy the header
                new_msg->header = msg->header;
                //Modify the time stamp to current ROS time -- It is necessary for MoveIt to work with simulated robot
                new_msg->header.stamp = this->now();
                //Copy the joint names
                new_msg->name = msg->name;
                //Copy the joint positions
                new_msg->position = msg->position;
                //Copy the joint velocities
                new_msg->velocity = msg->velocity;
                //Copy the joint efforts
                new_msg->effort = msg->effort;
                //Remove the gripper joint states
                /*
                new_msg->name.erase(new_msg->name.begin()+6, new_msg->name.end());
                new_msg->position.erase(new_msg->position.begin()+6, new_msg->position.end());
                new_msg->velocity.erase(new_msg->velocity.begin()+6, new_msg->velocity.end());
                new_msg->effort.erase(new_msg->effort.begin()+6, new_msg->effort.end());*/
                //Publish the new message
                mirror_pub->publish(*new_msg);
            }
    };
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ur_gazebo::JointMirror>("joint_state_mirror");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
