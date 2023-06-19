#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <octomap/octomap.h>

class OctomapToCostmapConverter
{
public:
    OctomapToCostmapConverter()
        : tf_listener_(tf_buffer_)
    {
        ros::NodeHandle nh("~");

        // Parameters
        nh.param("obstacle_height_threshold", obstacle_height_threshold_, 0.2);

        // Subscribers
        octomap_sub_ = nh.subscribe("/octomap_full", 1, &OctomapToCostmapConverter::octomapCallback, this);
        odom_sub_ = nh.subscribe("/odom", 1, &OctomapToCostmapConverter::odomCallback, this);

        // Publisher
        costmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    }

    void octomapCallback(const octomap_msgs::OctomapConstPtr& msg)
    {
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg));
        if (!octree)
        {
            ROS_ERROR("Failed to convert Octomap message to octree");
            return;
        }

        nav_msgs::OccupancyGrid costmap = convertToCostmap(octree);
        costmap_pub_.publish(costmap);

        delete octree;
    }

    void odomCallback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        robot_height_ = msg->pose.position.z;
    }

    nav_msgs::OccupancyGrid convertToCostmap(const octomap::OcTree* octree)
    {
        nav_msgs::OccupancyGrid costmap;

        // Define the size of the map
        double x_min, y_min, z_min, x_max, y_max, z_max;
        octree->getMetricMin(x_min, y_min, z_min);
        octree->getMetricMax(x_max, y_max, z_max);

        costmap.header.stamp = ros::Time::now();
        costmap.header.frame_id = "odom";
        costmap.info.resolution = octree->getResolution();
        costmap.info.width = (x_max - x_min) / costmap.info.resolution;
        costmap.info.height = (y_max - y_min) / costmap.info.resolution;
        costmap.info.origin.position.z = z_min;
        costmap.info.origin.position.y = y_min;
        costmap.info.origin.position.x = x_min;

        costmap.data.resize(costmap.info.width * costmap.info.height, -1);

        for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it)
        {
            if (octree->isNodeOccupied(*it))
            {
                octomap::point3d point = it.getCoordinate();

                if (std::abs(point.z() - robot_height_) < obstacle_height_threshold_)
                {
                    continue;
                }

                // Convert 3D coordinates to costmap index
                int index_x = std::floor((point.x() - x_min) / costmap.info.resolution);
                int index_y = std::floor((point.y() - y_min) / costmap.info.resolution);

                if(index_x < 0 || index_y < 0 || index_x >= costmap.info.width || index_y >= costmap.info.height)
                {
                    continue;
                }

                int costmap_index = index_y * costmap.info.width + index_x;

                // Set the costmap cell as occupied
                costmap.data[costmap_index] = 100;  // 100 signifies occupied
            }
        }

        return costmap;
    }

private:
    ros::Subscriber octomap_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher costmap_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    double obstacle_height_threshold_;
    double robot_height_ = 0.0;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_to_costmap_converter");

    OctomapToCostmapConverter converter;

    ros::spin();

    return 0;
}
