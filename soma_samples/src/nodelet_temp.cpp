#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

namespace soma_smpls
{
  class nodelet_temp : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
    void timer_callback(const ros::TimerEvent &e);

  private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Timer timer;
    //
    ros::Publisher pub;
  }; //class nodelet_temp

  void nodelet_temp::onInit()
  {
    NODELET_INFO("nodelet_temp Init");
    nh = getNodeHandle();
    pnh = getPrivateNodeHandle();

    pub = nh.advertise<std_msgs::String>("message", 3);

    timer = nh.createTimer(ros::Duration(1.0), &nodelet_temp::timer_callback, this);
  }

  void nodelet_temp::timer_callback(const ros::TimerEvent &e)
  {
    NODELET_INFO("Timer Event");
    std_msgs::String str_msg;
    str_msg.data = "hello world";
    pub.publish(str_msg);
  }

} //namespace soma_smpls

PLUGINLIB_EXPORT_CLASS(soma_smpls::nodelet_temp, nodelet::Nodelet);