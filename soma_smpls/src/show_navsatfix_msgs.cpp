#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"



void callback_navsatfix(const sensor_msgs::NavSatFixConstPtr &msgs)
{

    switch (msgs->status.status)
    {
    case sensor_msgs::NavSatStatus::STATUS_NO_FIX:
        printf("Unable to fix position\n");
        break;
    case sensor_msgs::NavSatStatus::STATUS_FIX:
        printf("Unaugmented fix\n");
        break;
    case sensor_msgs::NavSatStatus::STATUS_SBAS_FIX:
        printf("with satellite based augmentation\n");
        break;
    case sensor_msgs::NavSatStatus::STATUS_GBAS_FIX:
        printf("with ground based augmentation\n");
        break;
    default:
        printf("Unkown status : %d", msgs->status.status);
        break;
    }

    printf("  Alt:%f\n  Lat:%f\n  Lon:%f\n",
           msgs->altitude, msgs->latitude, msgs->longitude);

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ShowNavSatFix");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Subscriber sub = nh.subscribe("fix", 3, callback_navsatfix);

    ros::spin();

    return 0;
}