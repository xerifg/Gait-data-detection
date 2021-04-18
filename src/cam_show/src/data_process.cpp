#include "ros/ros.h"
#include "geometry_msgs/Point.h"

using namespace std;

void datacallback(const geometry_msgs::Point::ConstPtr& msg)
{
    ROS_INFO("You have getted datas");
    cout<<msg->x<<endl;


}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"data_process");

    ros::NodeHandle n;

    ros::Subscriber data_sub = n.subscribe("pre_data",1,datacallback);

    ros::spin();

    return 0;

}