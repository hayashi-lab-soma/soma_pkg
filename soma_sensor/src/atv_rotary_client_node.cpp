/**
 * SOMA ATV　用
 * 前輪に仕込んだロータリエンコーダによるタイヤ回転速度を
 * std_msgs/Float32でパブリッシュする用のノード
 * */
/**
 * RotaryEncoder --GPIO-> Raspberry Pi --Ethernet-> NUC1 
 * 注意) RaspeberryPi側のプログラムをミスってて，192.168.1.11にしかデータをUDP送信しないことになってるのであしからず
 * つまりこのクライアントノードは192.168.1.11でしか意味がない
 * */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
//
#include <qudpsocket.h>

//recieve data struct
struct Recv_t
{
  unsigned long pulse;
  double v;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "atv_rotary_client");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ROS_INFO("Run ATV rotary encoder client node");

  //get arguments
  double publish_rate = pnh.param<double>("publish_rate", 0.1); //Hz
  ros::Rate rate(1.0 / publish_rate);
  //
  std::string sender_ip = pnh.param<std::string>("server_ip", "192.168.1.79");
  int send_port = pnh.param<int>("send_port", 22346); //送信ポート
  int recv_port = pnh.param<int>("recv_port", 12346); //受信ポート

  //publisher
  ros::Publisher pub_wheel_vel = nh.advertise<std_msgs::Float32>("/vel", 3);

  //Create UDP sockets
  QUdpSocket send_sock;
  QUdpSocket recv_sock;
  // bind
  if (!recv_sock.bind(recv_port))
  {
    ROS_WARN("Rotary port bind error");
    exit(1);
  }
  ROS_DEBUG("Rotary port bind success");

  //
  while (ros::ok())
  {
    // recieve process
    if (recv_sock.waitForReadyRead(33))
    {
      Recv_t recv;
      recv.pulse = 0;
      recv.v = 0.0;

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
      ROS_WARN("Failure wait for ready to read : %s",
               recv_sock.errorString().toStdString().c_str());
    }

    ros::spinOnce();
    rate.sleep();
  }

  ROS_DEBUG("Shutdown ATV rotary encoder communication node");
  return 0;
}