
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/OctomapStamped.h>

ros::Publisher octomap_pub;

void octomap_callback(const octomap_msgs::Octomap::ConstPtr& msg)
{
    // Publish the octomap for visualization
    octomap_pub.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_publisher");
    ros::NodeHandle nh;

    octomap_pub = nh.advertise<octomap_msgs::OctomapStamped>("/octomap", 10);
    ros::Subscriber sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_full", 10, octomap_callback);

    ros::spin();

    return 0;
}
