#include <ros/ros.h>
#include <std_msgs/Float32.h>
//
#include <qudpsocket.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "atv_rotary_reciever");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ROS_INFO("Run ATV rotary encoder communication node");

  //get parameters
  double timer_t = pnh.param<double>("timer_t", 0.1);
  std::string sender_ip = pnh.param<std::string>("sender_ip", "192.168.1.79");
  int send_port = pnh.param<int>("send_port", 223457);
  int recv_port = pnh.param<int>("recv_port", 223456);

  //UDP sockets
  QUdpSocket send_sock;
  QUdpSocket recv_sock;
  recv_sock.bind(recv_port);

  return 0;
}