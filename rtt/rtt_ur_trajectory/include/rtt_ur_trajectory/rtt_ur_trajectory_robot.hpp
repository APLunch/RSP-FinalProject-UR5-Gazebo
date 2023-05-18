#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#include <rtt_ros2_params/rtt_ros2_params.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"

#include <kdl/jntarray.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "rclcpp/parameter_event_handler.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
class rtt_robot : public RTT::TaskContext {

private:
/////////////////////////////////HW9///////////////////////////////
  double x, y, z, roll, pitch, yaw;

  std::vector<double> desired_joint_positions;

  double velocity_scaling;

  RTT::Service::shared_ptr param_service;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
///////////////////////////////////////////////////////////////////

  RTT::OutputPort<sensor_msgs::msg::JointState> port_msr_joints;
  RTT::OutputPort<geometry_msgs::msg::PoseStamped> port_msr_pose;
  RTT::OutputPort<std_msgs::msg::Float64MultiArray> port_ctr_joints;
  
  RTT::OperationCaller<void(void)> zero;
  RTT::OperationCaller<void(const KDL::Wrench&)> offset;

  //size_t N;
  ReflexxesAPI* rml;
  RMLPositionInputParameters* ip;
  RMLPositionOutputParameters* op;
  RMLPositionFlags flags;

  RTT::Service::shared_ptr global_ros;
  RTT::Service::shared_ptr global_params;
  RTT::OperationCaller<rclcpp::ParameterValue(std::string)> getparam_operation;
  
  KDL::Chain chain;
  KDL::ChainFkSolverPos_recursive* fk_solver;
  
public:

  rtt_robot( const std::string& name );
  ~rtt_robot();

  KDL::JntArray jointMsr();
  KDL::Frame frameMsr();
  void MoveJ( const KDL::JntArray& q );
  void MoveC( const KDL::Frame& f );
  bool isMoveC;
  bool isCartMove;
  virtual bool configureHook();
  virtual bool startHook();

  virtual void updateHook();

  virtual void stopHook();
  virtual void cleanupHook();
  rcl_interfaces::msg::ParameterDescriptor angle_range = rcl_interfaces::msg::ParameterDescriptor();
  rcl_interfaces::msg::ParameterDescriptor tool_range = rcl_interfaces::msg::ParameterDescriptor();
  rcl_interfaces::msg::ParameterDescriptor scale_range = rcl_interfaces::msg::ParameterDescriptor();
  rcl_interfaces::msg::ParameterDescriptor Cartesian_Mode = rcl_interfaces::msg::ParameterDescriptor();
  rcl_interfaces::msg::ParameterDescriptor gripper_range = rcl_interfaces::msg::ParameterDescriptor();
  
  rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);
  
};
