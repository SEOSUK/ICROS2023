// /*

// /*******************************************************************************
// * D.A.S.O.M
// *
// * Department of Aerial Manipulator System for Object Manipulation
// *
// *     https://github.com/S-CHOI-S/D.A.S.O.M.git
// *
// * Mobile Robotics Lab. (MRL)
// * 	  @ Seoul National University of Science and Technology
// *
// *	  https://mrl.seoultech.ac.kr/index.do
// *
// *******************************************************************************/

// /* Authors: Sol Choi (Jennifer) */

// #include "two_link/torque_jacobian.h"

//   double time_loop = 0;
//   double time_f = 0;
//   double time_i = 0;

// TorqJ::TorqJ()
// : node_handle_(""),
//   priv_node_handle_("~")
// {
//   robot_name_ = node_handle_.param<std::string>("robot_name", "dasom");

//   X_P_gain = node_handle_.param<int>("X_P_gain",1);
//   X_I_gain = node_handle_.param<int>("X_I_gain",1);
//   X_D_gain = node_handle_.param<int>("X_D_gain",1);

//   Y_P_gain = node_handle_.param<int>("Y_P_gain",1);
//   Y_I_gain = node_handle_.param<int>("Y_I_gain",1);
//   Y_D_gain = node_handle_.param<int>("Y_D_gain",1);

//   VX_P_gain = node_handle_.param<int>("VX_P_gain",1);
//   VX_I_gain = node_handle_.param<int>("VX_I_gain",1);
//   VX_D_gain = node_handle_.param<int>("VX_D_gain",1);

//   VY_P_gain = node_handle_.param<int>("VY_P_gain",1);
//   VY_I_gain = node_handle_.param<int>("VY_I_gain",1);
//   VY_D_gain = node_handle_.param<int>("VY_D_gain",1);  

//   /************************************************************
//   ** Initialize ROS Subscribers and Clients
//   ************************************************************/
//   initPublisher();
//   initSubscriber();

//   Position_P_gain << X_P_gain, Y_P_gain;
//   Position_P_gain << X_I_gain, Y_I_gain;
//   Position_P_gain << X_D_gain, Y_D_gain;

//   Velocity_P_gain << VX_P_gain, VY_P_gain;
//   Velocity_I_gain << VX_I_gain, VY_I_gain;
//   Velocity_D_gain << VX_D_gain, VY_D_gain;

//   // std::cout<<V_gain<<std::endl<<"---------------------------------------"<<std::endl;

//   ROS_INFO("TorqJ node start");
// }

// TorqJ::~TorqJ()
// {
//   ROS_INFO("Bye!");
//   ros::shutdown();
// }

// void TorqJ::initPublisher()
// {
//   joint_command_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/goal_dynamixel_position", 10);
//   joint_measured_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/measured_dynamixel_position", 10);
//   // joint_command_pub = node_handle_.advertise<sensor_msgs::JointState>("/goal_dynamixel_position", 10);
// }

// void TorqJ::initSubscriber()
// {
//   EE_command_sub_ = node_handle_.subscribe("/goal_EE_position", 10, &TorqJ::commandCallback, this);
//   forwardkinematics_sub_ = node_handle_.subscribe("/EE_pose", 10, &TorqJ::poseCallback, this);
//   joint_states_sub_ = node_handle_.subscribe("/joint_states", 10, &TorqJ::jointCallback, this);
// }

// void TorqJ::commandCallback(const sensor_msgs::JointState::ConstPtr &msg)
// {
//   // X_command, Y_command 값 받아오기(O)
//   X_cmd[0] = msg->position.at(0);
//   X_cmd[1] = msg->position.at(1);

// }

// void TorqJ::poseCallback(const geometry_msgs::Twist::ConstPtr &msg)
// // 현재 EE position 값 받아오기
// {
//   X_measured[0] = Measured_EE_Position.linear.x;
//   X_measured[1] = Measured_EE_Position.linear.y;

// }

// void TorqJ::jointCallback(const sensor_msgs::JointState::ConstPtr &msg)
// {
//   angle_measured[0] = msg->position.at(0);
//   angle_measured[1] = msg->position.at(1);

//   // Jacobian 계산하기()
//   J = Jacobian(angle_measured[0], angle_measured[1]);
//   // Jacobian transpose()
//   JT = J.transpose();
//   ///////////////////////////////////////////////////////////////////////
//   theta_dot << msg->velocity.at(0), msg->velocity.at(1);

//   V_measured = J * theta_dot;
//   /////////////////////////////////////////////////////////////////////// 이 정도 계산은 어떤지?
// }


// void TorqJ::calc_des()
// {

//   //PID Control 실시 (위치 오차로 인한 전류값)
//     for(int i = 0; i<2; i++)
//     {
//       X_error_p[i] = X_cmd[i] - X_measured[i]; 
//       X_error_i[i] = X_error_i[i] + X_error_p[i] * time_loop;
//       if (X_error_p[i] - X_error_p_i[i] != 0) X_error_d[i] = (X_error_p[i] - X_error_p_i[i]) / time_loop;
//       X_error_p_i[i] = X_error_p[i];
//       X_PID[i] = Position_P_gain[i] * X_error_p[i] + Position_I_gain[i] * X_error_i[i] + Position_D_gain[i] * X_error_d[i];
//     }

//     for(int i = 0; i<2; i++)
//     {
//       V_error_p[i] = X_PID[i] - V_measured[i]; 
//       V_error_i[i] = V_error_i[i] + V_error_p[i] * time_loop;
//       if (V_error_p[i] - V_error_p_i[i] != 0) V_error_d[i] = (V_error_p[i] - V_error_p_i[i]) / time_loop;
//       V_error_p_i[i] = V_error_p[i];
//       V_PID[i] = Velocity_P_gain[i] * V_error_p[i] + Velocity_I_gain[i] * V_error_i[i] + Velocity_D_gain[i] * V_error_d[i];
//     }

//   JT.resize(2,2);
//   V_PID.resize(2,1);

//   tau_loop = JT * V_PID;


//   // // 중력 매트릭스 (OCM 있을 때)--------------------------------
// 	 tau_gravity << (mass1 * CoM1 * cos(angle_measured[0]) + mass2 * Link1 * cos(angle_measured[0]) + mass2 * CoM2 * cos(angle_measured[0] + angle_measured[1])) * 9.81 - offset_1,
// 	 				mass2 * CoM2 * cos(angle_measured[0] + angle_measured[1]) * 9.81 - offset_2;

// }

// void TorqJ::calc_taudes()
// {
//   tau_des = tau_loop + tau_gravity;


//   ROS_WARN("update!");
//   std::cout<<JT<<std::endl<<"---------------------------------------"<<std::endl;
//   std::cout<<V_PID<<std::endl<<"***************************************"<<std::endl;
//   std::cout<<tau_des<<std::endl<<"#########################################"<<std::endl;
// }

// void TorqJ::PublishCmdNMeasured()
// {
//   sensor_msgs::JointState joint_cmd;
//   sensor_msgs::JointState joint_measured;

//   // tau_des를 publish
//   joint_cmd.header.stamp = ros::Time::now();
//   joint_cmd.position.push_back(X_cmd[0]); // cartesian space
//   joint_cmd.position.push_back(X_cmd[1]); // cartesian space
//   joint_cmd.velocity.push_back(X_PID[0]); // cartesian space
//   joint_cmd.velocity.push_back(X_PID[1]); // cartesian space
//   joint_cmd.effort.push_back(tau_des[0]); // joint space
//   joint_cmd.effort.push_back(tau_des[1]); // joint space
//   joint_command_pub_.publish(joint_cmd);

//   joint_measured.header.stamp = ros::Time::now();
//   joint_measured.position.push_back(X_measured[0]);
//   joint_measured.position.push_back(X_measured[1]);
//   joint_measured.velocity.push_back(V_measured[0]);
//   joint_measured.velocity.push_back(V_measured[1]);
//   joint_measured_pub_.publish(joint_measured);
// }


// int main(int argc, char **argv)
// {
//   // Init ROS node
//   ros::init(argc, argv, "TorqJ");
//   TorqJ torqJ;

//   ros::Rate loop_rate(250); //250


// 	time_i = ros::Time::now().toSec();
//   while (ros::ok())
//   {
//     time_f = ros::Time::now().toSec();
// 		time_loop = time_f - time_i;
// 		time_i = ros::Time::now().toSec();

//     torqJ.calc_des();
//     torqJ.calc_taudes();
//     torqJ.PublishCmdNMeasured();
//     ros::spinOnce();
//     loop_rate.sleep();
//   }

//   return 0;
// }




























/*******************************************************************************
* D.A.S.O.M
*
* Department of Aerial Manipulator System for Object Manipulation
*
*     https://github.com/S-CHOI-S/D.A.S.O.M.git
*
* Mobile Robotics Lab. (MRL)
* 	  @ Seoul National University of Science and Technology
*
*	  https://mrl.seoultech.ac.kr/index.do
*
*******************************************************************************/

/* Authors: Sol Choi (Jennifer) */

#include "two_link/torque_jacobian.h"

  double time_loop = 0;
  double time_f = 0;
  double time_i = 0;
  double measured_effort = 0;
  int i = 0;


TorqJ::TorqJ()
: node_handle_(""),
  priv_node_handle_("~")
{
  robot_name_ = node_handle_.param<std::string>("robot_name", "dasom");

  X_P_gain = node_handle_.param<int>("X_P_gain",1);
  X_I_gain = node_handle_.param<int>("X_I_gain",1);
  X_D_gain = node_handle_.param<int>("X_D_gain",1);

  Y_P_gain = node_handle_.param<int>("Y_P_gain",1);
  Y_I_gain = node_handle_.param<int>("Y_I_gain",1);
  Y_D_gain = node_handle_.param<int>("Y_D_gain",1);

  VX_P_gain = node_handle_.param<int>("VX_P_gain",1);
  VX_I_gain = node_handle_.param<int>("VX_I_gain",1);
  VX_D_gain = node_handle_.param<int>("VX_D_gain",1);

  VY_P_gain = node_handle_.param<int>("VY_P_gain",1);
  VY_I_gain = node_handle_.param<int>("VY_I_gain",1);
  VY_D_gain = node_handle_.param<int>("VY_D_gain",1);  

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initPublisher();
  initSubscriber();

  Position_P_gain << X_P_gain, Y_P_gain;
  Position_I_gain << X_I_gain, Y_I_gain;
  Position_D_gain << X_D_gain, Y_D_gain;

  Velocity_P_gain << VX_P_gain, VY_P_gain;
  Velocity_I_gain << VX_I_gain, VY_I_gain;
  Velocity_D_gain << VX_D_gain, VY_D_gain;

  // std::cout<<V_gain<<std::endl<<"---------------------------------------"<<std::endl;

  ROS_INFO("TorqJ node start");



  // --------- butterworth 2nd order -----------//

  bw_2nd_output << 0, 0, 0;
  bw_2nd_input << 0, 0, 0;
  error_gain << 0, 0;

  cut_off_freq = 2;


}

TorqJ::~TorqJ()
{
  ROS_INFO("Bye!");
  ros::shutdown();
}

void TorqJ::initPublisher()
{
  joint_command_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/goal_dynamixel_position", 10);
  joint_measured_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/measured_dynamixel_position", 10);
  // joint_command_pub = node_handle_.advertise<sensor_msgs::JointState>("/goal_dynamixel_position", 10);
}

void TorqJ::initSubscriber()
{
  EE_command_sub_ = node_handle_.subscribe("/goal_EE_position", 10, &TorqJ::commandCallback, this);
  forwardkinematics_sub_ = node_handle_.subscribe("/EE_pose", 10, &TorqJ::poseCallback, this);
  joint_states_sub_ = node_handle_.subscribe("/joint_states", 10, &TorqJ::jointCallback, this);
}

void TorqJ::commandCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  // X_command, Y_command 값 받아오기(O)
//  X_cmd[0] = msg->position.at(0);
  X_cmd[1] = msg->position.at(1);

}

void TorqJ::poseCallback(const geometry_msgs::Twist::ConstPtr &msg)
// 현재 EE position 값 받아오기
{
  X_measured[0] = Measured_EE_Position.linear.x;
  X_measured[1] = Measured_EE_Position.linear.y;

}

void TorqJ::jointCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  angle_measured[0] = msg->position.at(0);
  angle_measured[1] = msg->position.at(1);

  measured_effort = msg->effort.at(0);
  // Jacobian 계산하기()
  J = Jacobian(angle_measured[0], angle_measured[1]);
  // Jacobian transpose()
  JT = J.transpose();
  ///////////////////////////////////////////////////////////////////////
  theta_dot << msg->velocity.at(0), msg->velocity.at(1);

  V_measured = J * theta_dot;
  /////////////////////////////////////////////////////////////////////// 이 정도 계산은 어떤지?
}


void TorqJ::calc_des()
{
  double I_limit = 0.1;

  //PID Control 실시 (위치 오차로 인한 전류값)
    for(int i = 0; i<2; i++)
    {
      X_error_p[i] = X_cmd[i] - angle_measured[i];
      X_error_i[i] = X_error_i[i] + X_error_p[i] * time_loop;
      X_error_i[i] = X_error_i[i] + X_error_p[i] * time_loop;


      if (X_error_p[i] - X_error_p_i[i] != 0) X_error_d[i] = (X_error_p[i] - X_error_p_i[i]) / time_loop;
      X_error_p_i[i] = X_error_p[i];
      X_PID[i] = Position_P_gain[i] * X_error_p[i] + Position_I_gain[i] * X_error_i[i] + Position_D_gain[i] * X_error_d[i];
    }
  std::cout<<X_error_i[0]<<std::endl<<"#########################################"<<std::endl;

    // for(int i = 0; i<2; i++)
    // {
    //   V_error_p[i] = X_PID[i] - V_measured[i]; 
    //   V_error_i[i] = V_error_i[i] + V_error_p[i] * time_loop;
    //   if (V_error_p[i] - V_error_p_i[i] != 0) V_error_d[i] = (V_error_p[i] - V_error_p_i[i]) / time_loop;
    //   V_error_p_i[i] = V_error_p[i];
    //   V_PID[i] = Velocity_P_gain[i] * V_error_p[i] + Velocity_I_gain[i] * V_error_i[i] + Velocity_D_gain[i] * V_error_d[i];
    // }




//-------커맨드 생성기------//
  i++;
  double Amp = M_PI / 6;
  double period = 8;
  X_cmd[0] = 0.5 * Amp * sin(2* M_PI * time_loop * i / period) + 0.01;


//--------버터워스---------//

  second_order_butterworth();
  fourth_order_butterworth();

//--------------------------------------
  error_gain[0] = bw_2nd_output[2] * 3;

  JT.resize(2,2);
  V_PID.resize(2,1);

//  tau_loop = JT * V_PID; onelink
	tau_loop = X_PID;
	tau_loop[1] = 0;

  // // 중력 매트릭스 (OCM 있을 때)--------------------------------
  //	 tau_gravity << (mass1 * CoM1 * cos(angle_measured[0]) + mass2 * Link1 * cos(angle_measured[0]) + mass2 * CoM2 * cos(angle_measured[0] + angle_measured[1])) * 9.81 - offset_1,
	 				mass2 * CoM2 * cos(angle_measured[0] + angle_measured[1]) * 9.81 - offset_2;

	// 중력 매트릭스 (1링크에다가 OCM) onelink
	tau_gravity << 	mass2 * CoM2 * cos(angle_measured[0] + angle_measured[1]) * 9.81 - offset_2,
	0;

}


  void TorqJ::second_order_butterworth()
{ 
  bw_2nd_input.resize(3,1);
  bw_2nd_output.resize(3,1);
  
  wc = tan(M_PI * cut_off_freq * time_loop);
  wc2 = wc*wc;
  b0_2nd = (wc2) / (1 + sqrt(2)*wc + wc2);
  b1_2nd = 2 * b0_2nd;
  b2_2nd = b0_2nd;
  a0_2nd = 1;
  a1_2nd = 2 * (wc2 - 1) / (1 + sqrt(2) * wc + wc2);
  a2_2nd = (1 - sqrt(2) * wc + wc2) / (1 + sqrt(2) * wc + wc2);


  bw_2nd_input[2] = X_error_p[0];

  bw_2nd_output[2] = b0_2nd * bw_2nd_input[2] + b1_2nd * bw_2nd_input[1] + b2_2nd * bw_2nd_input[0] - a1_2nd * bw_2nd_output[1] - a2_2nd * bw_2nd_output[0];

  bw_2nd_output[0] = bw_2nd_output[1];
  bw_2nd_output[1] = bw_2nd_output[2];

  bw_2nd_input[0] = bw_2nd_input[1];
  bw_2nd_input[1] = bw_2nd_input[2];
  //---------------------------------------
}


  void TorqJ::fourth_order_butterworth()
{
  bw_4th_output.resize(4,1);
  bw_4th_input.resize(4,1);
  
  wc = tan(M_PI * cut_off_freq * time_loop);
  wc2 = wc*wc;
  wc3 = wc*wc2;
  wc4 = wc2*wc2;

  b0_4th = wc4 / (1 + 4 * sqrt(2) * wc + 8 * wc2 + 4 * sqrt(2) * wc3 + wc4);
  b1_4th = (4 * wc4) / (1 + 4 * sqrt(2) * wc + 8 * wc2 + 4 * sqrt(2) * wc3 + wc4);
  b2_4th = (6 * wc4) / (1 + 4 * sqrt(2) * wc + 8 * wc2 + 4 * sqrt(2) * wc3 + wc4);
  b3_4th = (4 * wc4) / (1 + 4 * sqrt(2) * wc + 8 * wc2 + 4 * sqrt(2) * wc3 + wc4);
  b4_4th = (wc4) / (1 + 4 * sqrt(2) * wc + 8 * wc2 + 4 * sqrt(2) * wc3 + wc4);
  a0_4th = 1;
  a1_4th = (4 * wc4 - 4 * wc2) / (1 + 4 * sqrt(2) * wc + 8 * wc2 + 4 * sqrt(2) * wc3 + wc4);
  a2_4th = (6 * wc4 - 8 * wc2) / (1 + 4 * sqrt(2) * wc + 8 * wc2 + 4 * sqrt(2) * wc3 + wc4);
  a3_4th = (4 * wc4 - 4 * wc2) / (1 + 4 * sqrt(2) * wc + 8 * wc2 + 4 * sqrt(2) * wc3 + wc4);
  a4_4th = (wc4) / (1 + 4 * sqrt(2) * wc + 8 * wc2 + 4 * sqrt(2) * wc3 + wc4);

  bw_4th_input[4] = X_error_p[0];

  bw_4th_output[4] = b0_4th * bw_4th_input[4] + b1_4th * bw_4th_input[3] + b2_4th * bw_4th_input[2] + b3_4th * bw_4th_input[1] + b4_4th * bw_4th_input[0] - a1_4th * bw_4th_output[3] - a2_4th * bw_4th_output[2] - a3_4th * bw_4th_output[1] - a4_4th * bw_4th_output[0];

  bw_4th_output[0] = bw_4th_output[1];
  bw_4th_output[1] = bw_4th_output[2];
  bw_4th_output[2] = bw_4th_output[3];
  bw_4th_output[3] = bw_4th_output[4];

  bw_4th_input[0] = bw_4th_input[1];
  bw_4th_input[1] = bw_4th_input[2];
  bw_4th_input[2] = bw_4th_input[3];
  bw_4th_input[3] = bw_4th_input[4];
}


void TorqJ::calc_taudes()
{
  tau_des = tau_loop;

  ROS_WARN("update!");
//  std::cout<<JT<<std::endl<<"---------------------------------------"<<std::endl;
//  std::cout<<V_PID<<std::endl<<"***************************************"<<std::endl;
//  std::cout<<tau_des<<std::endl<<"#########################################"<<std::endl;
}

void TorqJ::PublishCmdNMeasured()
{
  sensor_msgs::JointState joint_cmd;
  sensor_msgs::JointState joint_measured;

  // tau_des를 publish
  joint_cmd.header.stamp = ros::Time::now();
  joint_cmd.position.push_back(X_cmd[0]); // cartesian space
  joint_cmd.position.push_back(angle_measured[0]); // cartesian space
  joint_cmd.velocity.push_back(X_cmd[0]); // cartesian space
  joint_cmd.velocity.push_back(angle_measured[0]); // cartesian space
  joint_cmd.effort.push_back(tau_des[0]); // joint space
  joint_cmd.effort.push_back(tau_des[1]); // joint space
  joint_command_pub_.publish(joint_cmd);

  joint_measured.header.stamp = ros::Time::now();
  joint_measured.position.push_back(measured_effort);
  joint_measured.position.push_back(X_error_p[0]);
  joint_measured.velocity.push_back(bw_2nd_output[2]);
  joint_measured.velocity.push_back(bw_4th_output[4]);
  joint_measured_pub_.publish(joint_measured);
}


int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "TorqJ");
  TorqJ torqJ;

  ros::Rate loop_rate(200); //250


	time_i = ros::Time::now().toSec();

  while (ros::ok())
  {
    time_f = ros::Time::now().toSec();
		time_loop = time_f - time_i;
		time_i = ros::Time::now().toSec();

    torqJ.calc_des();
    torqJ.calc_taudes();
    torqJ.PublishCmdNMeasured();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}