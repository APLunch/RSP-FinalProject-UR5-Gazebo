#include <rtt_ur_trajectory/rtt_ur_trajectory_robot.hpp>
#include <iomanip>

#include <rtt/internal/GlobalService.hpp>

#include <rtt_ros2/rtt_ros2.hpp>
#include <rtt_ros2_node/rtt_ros2_node.hpp>
#include <rtt_ros2_params/rtt_ros2_params.hpp>

#include <tf2_kdl/tf2_kdl.h>
#include <rtt/Component.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <cmath>
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include <rcl_interfaces/msg/floating_point_range.hpp>

rtt_robot::rtt_robot( const std::string& name ) :
  TaskContext(name),
  fk_solver(NULL){

  std::cout << "rtt_robot::rtt_robot" << std::endl;

  addPort("MsrJntPos", port_msr_joints ).doc("Measured joint positions");
  addPort("MsrCartPose", port_msr_pose ).doc("Measured Cartesian position");
  addPort("CtrJoint", port_ctr_joints ).doc("Control joint positions");

  addOperation("MoveJ", &rtt_robot::MoveJ, this, RTT::OwnThread );
  addOperation("MoveC", &rtt_robot::MoveC, this, RTT::OwnThread );
  addOperation("JointMsr", &rtt_robot::jointMsr, this, RTT::OwnThread );
  addOperation("FrameMsr", &rtt_robot::frameMsr, this, RTT::OwnThread );

  global_ros = RTT::internal::GlobalService::Instance()->getService("ros");
  RTT::OperationCaller<bool(const std::string&)> create_node =
    global_ros->getOperation("create_named_node");
  create_node.ready();
  //create_node(name);

  global_params = global_ros->getService("rosparam");
  getparam_operation = global_params->getOperation("getParameter");

std::string robot_description_value;
auto node = rtt_ros2_node::getNode(this);
auto parameters_client = node->create_client<rcl_interfaces::srv::GetParameters>("/robot_state_publisher/get_parameters");

while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
    return;
  }
  RCLCPP_INFO(node->get_logger(), "Waiting for service to appear...");
}
auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
request->names.push_back("robot_description");
auto future_result = parameters_client->async_send_request(request);
auto status = future_result.wait_for(std::chrono::seconds(1));

if (status == std::future_status::ready) {
  auto result = future_result.get();
  if (!result->values.empty()) {
    robot_description_value = result->values[0].string_value;
  } else {
    RCLCPP_ERROR(node->get_logger(), "The requested parameter was not found.");
  }
} else {
  RCLCPP_ERROR(node->get_logger(), "Service call failed.");
}

Cartesian_Mode.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;

angle_range.floating_point_range.push_back(rcl_interfaces::msg::FloatingPointRange());
angle_range.floating_point_range[0].from_value = -M_PI;
angle_range.floating_point_range[0].to_value = M_PI;

tool_range.floating_point_range.push_back(rcl_interfaces::msg::FloatingPointRange());
tool_range.floating_point_range[0].from_value = -1.0;
tool_range.floating_point_range[0].to_value = 1.0;

scale_range.floating_point_range.push_back(rcl_interfaces::msg::FloatingPointRange());
scale_range.floating_point_range[0].from_value = 0.0;
scale_range.floating_point_range[0].to_value = 100.0;

gripper_range.floating_point_range.push_back(rcl_interfaces::msg::FloatingPointRange());
gripper_range.floating_point_range[0].from_value = -1;
gripper_range.floating_point_range[0].to_value = 1;

  node->declare_parameter("robot_description", robot_description_value);

  node->declare_parameter("Pendant/Cartesian_Mode", false, Cartesian_Mode);

  node->declare_parameter("Desired_Cartesian_position/x", 0.81725, tool_range);
  node->declare_parameter("Desired_Cartesian_position/y", 0.19145, tool_range);
  node->declare_parameter("Desired_Cartesian_position/z", -0.005491, tool_range);
  node->declare_parameter("Desired_Cartesian_position/roll", M_PI/2, angle_range);
  node->declare_parameter("Desired_Cartesian_position/pitch", 0.0,angle_range);
  node->declare_parameter("Desired_Cartesian_position/yaw", M_PI, angle_range);

  node->declare_parameter("Desired_joint_positions/shoulder_pan_joint", 0.0, angle_range);
  node->declare_parameter("Desired_joint_positions/shoulder_lift_joint", 0.0, angle_range);
  node->declare_parameter("Desired_joint_positions/elbow_joint", 0.0, angle_range);
  node->declare_parameter("Desired_joint_positions/wrist_1_joint", 0.0, angle_range);
  node->declare_parameter("Desired_joint_positions/wrist_2_joint", 0.0, angle_range);
  node->declare_parameter("Desired_joint_positions/wrist_3_joint", 0.0, angle_range);
  
  node->declare_parameter("Control/left_gripper", -0.3, gripper_range);
  node->declare_parameter("Control/right_gripper", -0.3, gripper_range);

  node->declare_parameter("Velocity_scaling", 100.0, scale_range);
/*
  node->declare_parameter("Desired_Cartesian_position/x", 0.00165, tool_range);
  node->declare_parameter("Desired_Cartesian_position/y", -0.19333, tool_range);
  node->declare_parameter("Desired_Cartesian_position/z", 0.60195, tool_range);
  node->declare_parameter("Desired_Cartesian_position/roll", 0.0, angle_range);
  node->declare_parameter("Desired_Cartesian_position/pitch", -2.2259,angle_range);
  node->declare_parameter("Desired_Cartesian_position/yaw", 2.2244, angle_range);

  node->declare_parameter("Desired_joint_positions/shoulder_pan_joint", 0.0, angle_range);
  node->declare_parameter("Desired_joint_positions/shoulder_lift_joint", -1.57076, angle_range);
  node->declare_parameter("Desired_joint_positions/elbow_joint", 0.0, angle_range);
  node->declare_parameter("Desired_joint_positions/wrist_1_joint", -1.570736, angle_range);
  node->declare_parameter("Desired_joint_positions/wrist_2_joint", 0.0, angle_range);
  node->declare_parameter("Desired_joint_positions/wrist_3_joint", 0.0, angle_range);

  node->declare_parameter("Velocity_scaling", 30.0, scale_range);
*/

  callback_handle_ = node->add_on_set_parameters_callback(
    std::bind(&rtt_robot::parametersCallback, this, std::placeholders::_1));
////////////////////////////////////////////////////////////////////////////////////
}

rtt_robot::~rtt_robot(){
  if( rml ) delete rml;
  if( ip ) delete ip;
  if( op ) delete op;
  if( fk_solver ) delete fk_solver;
}

bool rtt_robot::configureHook(){

  std::cout << "rtt_robot::configureHook" << std::endl;
  
  rclcpp::ParameterValue new_string = getparam_operation.call("robot_description");
  
  std::string robot_description = new_string.get<std::string>();
  RTT::log().setLogLevel( RTT::Logger::Info );
			
  KDL::Tree tree;
  if( kdl_parser::treeFromString( robot_description, tree)){
    RTT::log(RTT::Info) << "Description parsed to a tree" << RTT::endlog();
    if( tree.getChain("world", "tool0", chain)){
      fk_solver = new KDL::ChainFkSolverPos_recursive(chain);
    }
    else{
      RTT::log(RTT::Error) << "Failed to parse chain between world and link"
			   << RTT::endlog();
    }
  }
  else{
    RTT::log(RTT::Error) << "Failed to parse tree" << RTT::endlog();
  }
  RTT::log(RTT::Info) << "Description parsed to a chain" << RTT::endlog();

  rml = new ReflexxesAPI( chain.getNrOfJoints(), getPeriod() );
  ip = new RMLPositionInputParameters( chain.getNrOfJoints() );
  op = new RMLPositionOutputParameters( chain.getNrOfJoints() );
  isMoveC=false;
  return true;

  //////////////////////////////////HW9/////////////////////////////////////////


}

bool rtt_robot::startHook(){
  std::cout << "rtt_robot::startHook" << std::endl;
  isMoveC=false;
  ip->CurrentPositionVector->VecData[0] = 0.0;
  ip->CurrentPositionVector->VecData[1] = 0.0;
  ip->CurrentPositionVector->VecData[2] = 0.0;
  ip->CurrentPositionVector->VecData[3] = 0.0;
  ip->CurrentPositionVector->VecData[4] = 0.0;
  ip->CurrentPositionVector->VecData[5] = 0.0;
  /*
  ip->CurrentPositionVector->VecData[0] = 0.0;
  ip->CurrentPositionVector->VecData[1] = -1.57076;
  ip->CurrentPositionVector->VecData[2] = 0.0;
  ip->CurrentPositionVector->VecData[3] = -1.570736;
  ip->CurrentPositionVector->VecData[4] = 0.0;
  ip->CurrentPositionVector->VecData[5] = 0.0;

  op->NewPositionVector->VecData[0] = 0.0;
  op->NewPositionVector->VecData[1] = -1.57076;
  op->NewPositionVector->VecData[2] = 0.0;
  op->NewPositionVector->VecData[3] = -1.570736;
  op->NewPositionVector->VecData[4] = 0.0;
  op->NewPositionVector->VecData[5] = 0.0;
  */
  for( size_t i=0; i<chain.getNrOfJoints(); i++ ){

    // current state of the robot
    // In reality you need to querry the real robot to get these
    //ip->CurrentPositionVector->VecData[i] = 0.0;
    ip->CurrentVelocityVector->VecData[i] = 0.0;
    ip->CurrentAccelerationVector->VecData[i] = 0.0;

    // robot limits
    // These should come from a config file
    ip->MaxVelocityVector->VecData[i] = 0.10*1.0;
    ip->MaxAccelerationVector->VecData[i] = 1.0;
    ip->MaxJerkVector->VecData[i] = 10.0;

    ip->SelectionVector->VecData[i] = true;

  }

  return true;
}

  void rtt_robot::updateHook(){

  //  std::cout << "rtt_robot::updateHook" << std::endl;
  // Interpolate the next trajectory point
  int result = rml->RMLPosition( *ip, op, flags );
  auto node = rtt_ros2_node::getNode(this);
  // Update the current state to be the state
  // This assume that the positions/velocities/accelerations
  // are sent to the real robot _and_ that the real robot
  // will move to that position. Otherwise, the current state
  // should be obtained from the real roboto
  double scale_=node->get_parameter("Velocity_scaling").as_double();
  double new_speed_limit=0.10 * scale_ / 100;
  for( size_t i=0; i<chain.getNrOfJoints(); i++ ){
  ip->MaxVelocityVector->VecData[i] = new_speed_limit;
  }
  
  *ip->CurrentPositionVector = *op->NewPositionVector;
  *ip->CurrentVelocityVector = *op->NewVelocityVector;
  *ip->CurrentAccelerationVector = *op->NewAccelerationVector;
  // Pack the position into KDL and write to port
  KDL::JntArray q(chain.getNrOfJoints()), qd(chain.getNrOfJoints());
  q(1)=-1.57076;
  q(3)=-1.57076;
  op->GetNewPositionVector( q.data.data(), sizeof(double)*chain.getNrOfJoints() );
  op->GetNewVelocityVector( qd.data.data(), sizeof(double)*chain.getNrOfJoints() );

  if(isMoveC==true){
  // Pack the position into KDL and write to port
  KDL::JntArray q0(chain.getNrOfJoints()), qd0(chain.getNrOfJoints());
  op->GetNewPositionVector( q0.data.data(), sizeof(double)*chain.getNrOfJoints() );
  op->GetNewVelocityVector( qd0.data.data(), sizeof(double)*chain.getNrOfJoints() );
  KDL::Vector position(q0(3), q0(4), q0(5));
  KDL::Rotation rotation = KDL::Rotation::RPY(q0(0), q0(1), q0(2));
  KDL::Frame frame(rotation, position);

  //////////////////////////////////////////Ik_solver////////////////////////////////////////////////////

  KDL::ChainIkSolverPos_LMA ik_solver = KDL::ChainIkSolverPos_LMA(chain, 1E-5, 500, 1E-15);
  int ret = ik_solver.CartToJnt(q0, frame, q);


  }
  


  //change KDL::JntArray to sensor_msgs::msg::JointState
  sensor_msgs::msg::JointState joint_state_msg;
  std_msgs::msg::Float64MultiArray control_msg;
  control_msg.data.resize(8);
  std::vector<std::string> joint_names = {
  "shoulder_pan_joint",
  "shoulder_lift_joint",
  "elbow_joint",
  "wrist_1_joint",
  "wrist_2_joint",
  "wrist_3_joint"
  };
  joint_state_msg.name = joint_names;
  joint_state_msg.position.resize(chain.getNrOfJoints());
  //joint_state_msg.velocity.resize(chain.getNrOfJoints());

  for (size_t i = 0; i < chain.getNrOfJoints(); ++i) {
    joint_state_msg.position[i] = q(i);
    //joint_state_msg.velocity[i] = qd(i);
  }
  rclcpp::Clock clock;
  joint_state_msg.header.stamp = clock.now();
  joint_state_msg.header.frame_id = "";
  port_msr_joints.write(joint_state_msg);

  control_msg.data[0]=q(0);
  control_msg.data[1]=q(1);
  control_msg.data[2]=q(2);
  control_msg.data[3]=q(3);
  control_msg.data[4]=q(4);
  control_msg.data[5]=q(5);
  control_msg.data[6]=node->get_parameter("Control/left_gripper").as_double();
  control_msg.data[7]=node->get_parameter("Control/right_gripper").as_double();

  port_ctr_joints.write(control_msg);

  /////////////////Update Parameter//////////////////////
    isCartMove=node->get_parameter("Pendant/Cartesian_Mode").as_bool();

    if(isCartMove==true){
      auto curj = jointMsr();

        rclcpp::Parameter updated_param0("Desired_joint_positions/shoulder_pan_joint", curj(0));
        rclcpp::Parameter updated_param1("Desired_joint_positions/shoulder_lift_joint", curj(1));
        rclcpp::Parameter updated_param2("Desired_joint_positions/elbow_joint", curj(2));
        rclcpp::Parameter updated_param3("Desired_joint_positions/wrist_1_joint", curj(3));
        rclcpp::Parameter updated_param4("Desired_joint_positions/wrist_2_joint", curj(4));
        rclcpp::Parameter updated_param5("Desired_joint_positions/wrist_3_joint", curj(5));
        node->set_parameter(updated_param0);
        node->set_parameter(updated_param1);
        node->set_parameter(updated_param2);
        node->set_parameter(updated_param3);
        node->set_parameter(updated_param4);
        node->set_parameter(updated_param5);
    }
    else{
      KDL::Frame framecur=frameMsr();
      double roll_0, pitch_0, yaw_0;
      framecur.M.GetRPY(roll_0, pitch_0, yaw_0);
        rclcpp::Parameter updated_param_0("Desired_Cartesian_position/x", framecur.p[0]);
        rclcpp::Parameter updated_param_1("Desired_Cartesian_position/y", framecur.p[1]);
        rclcpp::Parameter updated_param_2("Desired_Cartesian_position/z", framecur.p[2]);
        rclcpp::Parameter updated_param_3("Desired_Cartesian_position/roll", roll_0);
        rclcpp::Parameter updated_param_4("Desired_Cartesian_position/pitch", pitch_0);
        rclcpp::Parameter updated_param_5("Desired_Cartesian_position/yaw", yaw_0);
        node->set_parameter(updated_param_0);
        node->set_parameter(updated_param_1);
        node->set_parameter(updated_param_2);
        node->set_parameter(updated_param_3);
        node->set_parameter(updated_param_4);
        node->set_parameter(updated_param_5);
    }
    
}

void rtt_robot::stopHook(){
  std::cout << "rtt_robot::stopHook" << std::endl;
}

void rtt_robot::cleanupHook(){
  std::cout << "rtt_robot::cleanupHook" << std::endl;
}

void rtt_robot::MoveJ( const KDL::JntArray& q ){
  
  if(isMoveC==true){
  KDL::JntArray q0(chain.getNrOfJoints()), qd0(chain.getNrOfJoints()), q1(chain.getNrOfJoints());
  op->GetNewPositionVector( q0.data.data(), sizeof(double)*chain.getNrOfJoints() );
  op->GetNewVelocityVector( qd0.data.data(), sizeof(double)*chain.getNrOfJoints() );
  KDL::Vector position(q0(3), q0(4), q0(5));
  KDL::Rotation rotation = KDL::Rotation::RPY(q0(0), q0(1), q0(2));
  KDL::ChainIkSolverPos_LMA ik_solver = KDL::ChainIkSolverPos_LMA(chain, 1E-5, 500, 1E-15);
  KDL::Frame frame(rotation, position);
  int ret = ik_solver.CartToJnt(q0, frame, q1);
  for( size_t i=0; i<chain.getNrOfJoints(); i++ ){
    ip->CurrentPositionVector->VecData[i] = q1(i);
  }
  } 
  isMoveC=false;
  // These are goal position/velocities
  // This has no business here and should come from an operation/port
  for( size_t i=0; i<chain.getNrOfJoints(); i++ ){
  ip->TargetPositionVector->VecData[i] = q(i);
  //ip->TargetVelocityVector->VecData[0] = 0.0;
  }
  
}
void rtt_robot::MoveC( const KDL::Frame& f ){
  
  if(isMoveC==false){
  KDL::Frame frame0;
  KDL::JntArray q0 = jointMsr();
  KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
  int ret0 = fksolver.JntToCart(q0, frame0);
  double roll0, pitch0, yaw0;
  frame0.M.GetRPY(roll0, pitch0, yaw0);
  ip->CurrentPositionVector->VecData[0] = roll0;
  ip->CurrentPositionVector->VecData[1] = pitch0;
  ip->CurrentPositionVector->VecData[2] = yaw0;
  ip->CurrentPositionVector->VecData[3] = frame0.p[0];
  ip->CurrentPositionVector->VecData[4] = frame0.p[1];
  ip->CurrentPositionVector->VecData[5] = frame0.p[2];
  }
  isMoveC=true;
  double roll, pitch, yaw;
  f.M.GetRPY(roll, pitch, yaw);
  ip->TargetPositionVector->VecData[0] = roll;
  ip->TargetPositionVector->VecData[1] = pitch;
  ip->TargetPositionVector->VecData[2] = yaw;
  ip->TargetPositionVector->VecData[3] = f.p[0];
  ip->TargetPositionVector->VecData[4] = f.p[1];
  ip->TargetPositionVector->VecData[5] = f.p[2];

}

KDL::JntArray rtt_robot::jointMsr(){
  KDL::JntArray q(chain.getNrOfJoints());
  if(isMoveC==true){
  KDL::JntArray q0(chain.getNrOfJoints()), qd0(chain.getNrOfJoints());
  op->GetNewPositionVector( q0.data.data(), sizeof(double)*chain.getNrOfJoints() );
  op->GetNewVelocityVector( qd0.data.data(), sizeof(double)*chain.getNrOfJoints() );
  KDL::Vector position(q0(3), q0(4), q0(5));
  KDL::Rotation rotation = KDL::Rotation::RPY(q0(0), q0(1), q0(2));
  KDL::Frame frame(rotation, position);
  KDL::ChainIkSolverPos_LMA ik_solver = KDL::ChainIkSolverPos_LMA(chain, 1E-5, 500, 1E-15);
  int ret = ik_solver.CartToJnt(q0, frame, q);
  }
  else{
  for( size_t i=0; i<chain.getNrOfJoints(); i++ ){
  q(i) = ip->CurrentPositionVector->VecData[i];
    }
  }


  return q;
}

KDL::Frame rtt_robot::frameMsr(){
 // if(isMoveC==true){
 //   
 // }
 // else{ 
  KDL::Frame frame;
  KDL::JntArray q = jointMsr();
  KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
  int ret = fksolver.JntToCart(q, frame);
  if(ret >= 0){
    return frame;
  } 
  else{
    RTT::log(RTT::Error) << "Error in frameMsr: couldn't calculate forward kinematics." << RTT::endlog();
    return KDL::Frame();
  }
 // }
}

rcl_interfaces::msg::SetParametersResult rtt_robot::parametersCallback(const std::vector<rclcpp::Parameter> &parameters){
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &param: parameters){
        //std::cout << param.get_name()<<": "<< param.value_to_string() << std::endl;
        auto node = rtt_ros2_node::getNode(this);
        isCartMove=node->get_parameter("Pendant/Cartesian_Mode").as_bool();
        if((param.get_name()=="Desired_Cartesian_position/x" ||
           param.get_name()=="Desired_Cartesian_position/y" ||
           param.get_name()=="Desired_Cartesian_position/z" ||
           param.get_name()=="Desired_Cartesian_position/roll" ||
           param.get_name()=="Desired_Cartesian_position/pitch" ||
           param.get_name()=="Desired_Cartesian_position/yaw") && isCartMove==true){
              
              double des_x=node->get_parameter("Desired_Cartesian_position/x").as_double();
              double des_y=node->get_parameter("Desired_Cartesian_position/y").as_double();
              double des_z=node->get_parameter("Desired_Cartesian_position/z").as_double();
              double des_r=node->get_parameter("Desired_Cartesian_position/roll").as_double();
              double des_p=node->get_parameter("Desired_Cartesian_position/pitch").as_double();
              double des_ya=node->get_parameter("Desired_Cartesian_position/yaw").as_double();

              if(param.get_name()=="Desired_Cartesian_position/x"){
                des_x=param.as_double();
              }
              else if(param.get_name()=="Desired_Cartesian_position/y"){
                des_y=param.as_double();
              }
              else if(param.get_name()=="Desired_Cartesian_position/z"){
                des_z=param.as_double();
              }
              else if(param.get_name()=="Desired_Cartesian_position/roll"){
                des_r=param.as_double();
              }
              else if(param.get_name()=="Desired_Cartesian_position/pitch"){
                des_p=param.as_double();
              }
              else if(param.get_name()=="Desired_Cartesian_position/yaw"){
                des_ya=param.as_double();
              }
              KDL::Vector des_pos(des_x, des_y, des_z);
              KDL::Rotation des_rot = KDL::Rotation::RPY(des_r, des_p, des_ya);
              KDL::Frame des_frame(des_rot, des_pos);
              MoveC(des_frame);
              }
              
          if((param.get_name()=="Desired_joint_positions/shoulder_pan_joint" ||
           param.get_name()=="Desired_joint_positions/shoulder_lift_joint" ||
           param.get_name()=="Desired_joint_positions/elbow_joint" ||
           param.get_name()=="Desired_joint_positions/wrist_1_joint" ||
           param.get_name()=="Desired_joint_positions/wrist_2_joint" ||
           param.get_name()=="Desired_joint_positions/wrist_3_joint") && isCartMove==false){
              double des_1=node->get_parameter("Desired_joint_positions/shoulder_pan_joint").as_double();
              double des_2=node->get_parameter("Desired_joint_positions/shoulder_lift_joint").as_double();
              double des_3=node->get_parameter("Desired_joint_positions/elbow_joint").as_double();
              double des_4=node->get_parameter("Desired_joint_positions/wrist_1_joint").as_double();
              double des_5=node->get_parameter("Desired_joint_positions/wrist_2_joint").as_double();
              double des_6=node->get_parameter("Desired_joint_positions/wrist_3_joint").as_double();

              if(param.get_name()=="Desired_joint_positions/shoulder_pan_joint"){
                des_1=param.as_double();
              }
              else if(param.get_name()=="Desired_joint_positions/shoulder_lift_joint"){
                des_2=param.as_double();
              }
              else if(param.get_name()=="Desired_joint_positions/elbow_joint"){
                des_3=param.as_double();
              }
              else if(param.get_name()=="Desired_joint_positions/wrist_1_joint"){
                des_4=param.as_double();
              }
              else if(param.get_name()=="Desired_joint_positions/wrist_2_joint"){
                des_5=param.as_double();
              }
              else if(param.get_name()=="Desired_joint_positions/wrist_3_joint"){
                des_6=param.as_double();
              }
              KDL::JntArray q(chain.getNrOfJoints());
              q(0)=des_1;
              q(1)=des_2;
              q(2)=des_3;
              q(3)=des_4;
              q(4)=des_5;
              q(5)=des_6;
              MoveJ(q);
        }
        }
        return result;
    }
ORO_CREATE_COMPONENT(rtt_robot)
