#include <ros/ros.h>
#include <sensor_msgs/Image.h>

using namespace  std;

void  im_showCallback(const sensor_msgs::Image::ConstPtr& msg)
{

    ROS_INFO("you have got camera image.congratulation!!! ");
}

int main(int arc,char** arv)
{
    ros::init(arc, arv, "image_show");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/camera/color/image_raw", 1000, im_showCallback);





    ros::spin();
    return 0;
}