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

  //publisher
  ros::Publisher pub_wheel_vel = nh.advertise<std_msgs::Float32>("/soma/wheel_vel", 3);

  //UDP sockets
  QUdpSocket send_sock;
  QUdpSocket recv_sock;
  recv_sock.bind(recv_port);

  //recieve data struct
  struct Recv_t
  {
    long pulse[2];
    double v;
    double d[2];
    double T;
  } recv;
  recv.pulse[0] = 0;
  recv.pulse[1] = 1;
  recv.v = 0.0;
  recv.d[0] = 0.0;
  recv.d[1] = 0.0;
  recv.T = 0.0;


  ros::Rate rate(1.0/timer_t);
  while (ros::ok())
  {
    // recieve process
    if (recv_sock.waitForReadyRead(33))
    {
      while (recv_sock.bytesAvailable() > 0)
      {
        recv_sock.readDatagram((char *)&recv,
                               sizeof(Recv_t));
      }
    }
    else
    {
    }



    //publish
    std_msgs::Float32 wheel_vel;
    wheel_vel.data = recv.v;
    pub_wheel_vel.publish(wheel_vel);

    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("Shutdown ATV rotary encoder communication node");

  return 0;
}