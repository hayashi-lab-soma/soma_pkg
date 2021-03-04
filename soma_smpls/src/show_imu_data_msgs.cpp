#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

void callback_ImuStr(const std_msgs::StringConstPtr &msgs)
{
    ROS_INFO(msgs->data.c_str());
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ShowImuData");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Subscriber sub = nh.subscribe("imu_data_str", 3, callback_ImuStr);

    ros::spin();

    return 0;
}