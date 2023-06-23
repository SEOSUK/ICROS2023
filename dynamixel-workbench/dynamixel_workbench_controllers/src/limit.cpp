#include "dynamixel_workbench_controllers/limit.h"

Limit::Limit()
: node_handle_(""),
  priv_node_handle_("~")
{
  robot_name_   = node_handle_.param<std::string>("robot_name", "dasom");

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initPublisher();
  initSubscriber();

  ROS_INFO("Limit node start");
}

Limit::~Limit()
{
  ROS_INFO("Bye!");
  ros::shutdown();
}

void Limit::initPublisher()
{
  dasom_joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/dasom/goal_dynamixel_position", 10);
}

void Limit::initSubscriber()
{
  goal_joint_states_sub_ = node_handle_.subscribe("/dasom/position_limit", 10, &Limit::goaljointCallback, this);
}

void Limit::goaljointCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  dynamixel_workbench_msgs::DasomDynamixel dynamixel_;

  if (!(-M_PI < msg->position.at(0) && msg->position.at(0) < M_PI))
  {
    ROS_ERROR("Error[001] : Joint1 is limited!\n");
    
    dynamixel_.flag.push_back(1);
  }
  else
  {
    ROS_INFO("Done[001] : Joint1 is Moving!\n");
    dynamixel_.flag.push_back(0);
    dynamixel_.position.push_back(msg->position.at(0));
  } 

  if (!(-M_PI_2 < msg->position.at(1) && msg->position.at(1) < 1.53))
  {
    ROS_ERROR("Error[002] : Joint2 is limited!\n");
    
    dynamixel_.flag.push_back(1);
  }
  else
  {
    dynamixel_.flag.push_back(0);
    dynamixel_.position.push_back(msg->position.at(1));
  }
    
  if (!(-M_PI_2 < msg->position.at(2) && msg->position.at(2) < 1.53))
  {
    ROS_ERROR("Error[003] : Joint3 is limited!\n");
    
    dynamixel_.flag.push_back(1);
  }
  else
  {
    dynamixel_.flag.push_back(0);
    dynamixel_.position.push_back(msg->position.at(2));
  } 

  dasom_joint_states_pub_.publish(dynamixel_);

}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "limit");
  Limit limit;


  ros::Rate loop_rate(250);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}