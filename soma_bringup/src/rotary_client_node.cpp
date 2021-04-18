#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <qudpsocket.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rotary_client_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // get parameters
  double pub_rate = pnh.param<double>("publish_rate", 1.0); //Hz
  //
  std::string sender_ip = pnh.param<std::string>("server_ip", "192.168.1.79"); // RaspberryPi address
  int send_port = pnh.param<int>("send_port", 22346);                          // send port to RaspberryPi
  int recv_port = pnh.param<int>("recv_port", 12346);                          // recv port from RaspberryPi

  //publishers
  ros::Publisher pub_wheel_vel = nh.advertise<std_msgs::Float32>("/wheel_vel", 5);

  ROS_INFO("Try to bind ...");
  //Create UDP sockets
  QUdpSocket send_sock;
  QUdpSocket recv_sock;
  // bind
  if (!recv_sock.bind(recv_port))
  {
    ROS_WARN("Recieve port bind error");
    ROS_WARN("%s", recv_sock.errorString().toStdString().c_str());
    exit(1);
  }

  ros::Rate rate(pub_rate);

  while (ros::ok())
  {
    ROS_INFO("Read available socket data ...");
    if (recv_sock.waitForReadyRead(33))
    {
      struct Recv_t
      {
        unsigned long pulse;
        double v;
        Recv_t() : pulse(0), v(0.0) {}
      } recv;

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
      if (recv_sock.error() != QAbstractSocket::SocketError::SocketTimeoutError)
      {
        ROS_WARN("Failure wait for ready to read : (%d) %s",
                 recv_sock.error(),
                 recv_sock.errorString().toStdString().c_str());
      }
    }

    //
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}