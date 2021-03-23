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
  double publish_rate = pnh.param<double>("publish_rate", 0.1);
  std::string sender_ip = pnh.param<std::string>("server_ip", "192.168.1.79");
  int send_port = pnh.param<int>("send_port", 22346);
  int recv_port = pnh.param<int>("recv_port", 12346);

  //publisher
  ros::Publisher pub_wheel_vel = nh.advertise<std_msgs::Float32>("/vel", 3);

  //UDP sockets
  QUdpSocket send_sock;
  QUdpSocket recv_sock;
  if (!recv_sock.bind(recv_port))
  {
    ROS_WARN("Rotary port bind error");
    exit(1);
  }
  ROS_DEBUG("Rotary port bind success");

  //recieve data struct
  struct Recv_t
  {
    unsigned long pulse;
    double v;
  } recv;
  recv.pulse = 0;
  recv.v = 0.0;

  ros::Rate rate(1.0 / publish_rate);
  while (ros::ok())
  {
    // recieve process
    if (recv_sock.waitForReadyRead(33))
    {
      while (recv_sock.bytesAvailable() > 0)
      {
        ROS_DEBUG("Read UDP data");
        recv_sock.readDatagram((char *)&recv,
                               sizeof(Recv_t));
      }
      ROS_DEBUG("v=%.2f", recv.v);

      //publish rear wheel velocity
      std_msgs::Float32 wheel_vel;
      wheel_vel.data = recv.v;
      pub_wheel_vel.publish(wheel_vel);
    }
    else
    {
      // ROS_WARN("Failure wait for ready to read : %s", recv_sock.errorString().toStdString().c_str());
    }

    ros::spinOnce();
    rate.sleep();
  }

  ROS_DEBUG("Shutdown ATV rotary encoder communication node");

  return 0;
}