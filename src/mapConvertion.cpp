#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <vector>
#include <std_msgs/Float64.h>

ros::Publisher map_pub;
nav_msgs::OccupancyGrid map2d;

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
    try {
        // Convert the Octomap to a vector
        std::vector<int8_t> octomap_data(msg->data.begin(), msg->data.end());

        // Extract the dimensions from the Octomap message
        int num_cells = octomap_data.size();

        // Assuming the Octomap is a full occupancy octree, set the binary flag accordingly
        bool binary = true;

        // Extract the resolution from the Octomap message
        double resolution = msg->resolution;

        int width = std::sqrt(num_cells);
        int height = num_cells / width;

        std::vector<int8_t> map2d_data(width * height, 0);
        for (int i = 0; i < num_cells; ++i) {
            map2d_data[i] = octomap_data[i];
        }

        // Normalize the values to range from 0 to 100
        int max_value = *std::max_element(map2d_data.begin(), map2d_data.end());
        for (int i = 0; i < num_cells; ++i) {
            map2d_data[i] = static_cast<int8_t>((map2d_data[i] / static_cast<double>(max_value)) * 100);
        }

        // Update the 2D map message
        map2d.header = msg->header;
        map2d.info.resolution = resolution;
        map2d.info.width = width;
        map2d.info.height = height;
        map2d.data = map2d_data;

        // Publish the 2D map
        map_pub.publish(map2d);
    }
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("Failed to convert Octomap to 2D map: " << e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_to_2dmap");
    ros::NodeHandle nh;

    // Create publisher for 2D map
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 10);

    // Create subscriber to octomap_full topic
    ros::Subscriber sub = nh.subscribe("/octomap_full", 10, octomapCallback);

    ros::spin();

    return 0;
}