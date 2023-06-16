#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/random.h>
#include <vector>
#include "iostream"
using namespace std;
using namespace octomap;

// Function prototypes
void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);
std::pair<std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::RGB>>, std::vector<geometry_msgs::Point>> clusterFrontiers(const std::vector<octomap::OcTree::iterator>& frontiers);
geometry_msgs::Point selectGoalPoint(const std::vector<geometry_msgs::Point>& centers);
visualization_msgs::MarkerArray createVisualizationMarkers(const std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::RGB>>& clusters);
bool isFrontierVoxel(octomap::OcTree::iterator& it, octomap::OcTree* octree);


// publishers and subscribers
ros::Publisher markerPub;
ros::Publisher goalPub;

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
    
    // Check if the conversion was successful
    if(!octree){
        ROS_ERROR("Failed to create OcTree from Octomap message!");
        return;
    }

    // Detected frontiers
    std::vector<octomap::OcTree::iterator> frontiers;

    for(octomap::OcTree::iterator it = octree->begin(), end = octree->end(); it != end; ++it) {
        if (isFrontierVoxel(it, octree)) {  // Pass the octree variable as a parameter
            frontiers.push_back(it);
        }
    }   

    auto results = clusterFrontiers(frontiers);
    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::RGB>> clusters = results.first;
    std::vector<geometry_msgs::Point> cluster_centers = results.second;
    visualization_msgs::MarkerArray markers;
    markers = createVisualizationMarkers(clusters);

    markerPub.publish(markers);

    // Publish the goal point to the navigation node
    // select the goal point from the cluster centers
    if (!cluster_centers.empty()) {
        geometry_msgs::PointStamped goal_point;
        goal_point.point = selectGoalPoint(cluster_centers);  // Function to implement selection criteria
        goalPub.publish(goal_point);
    }
    
    // Free the OcTree
    delete octree;
}

bool isFrontierVoxel(octomap::OcTree::iterator& it, octomap::OcTree* octree) {
    int unknown_count = 0;
    int known_count = 0;

    if(octree->isNodeOccupied(*it)) {
        return false;
    }

    // Count the number of unknown and known neighbors
    for (octomap::OcTree::leaf_bbx_iterator neighbor = octree->begin_leafs_bbx(it.getCoordinate() - octomap::point3d(1,1,1), 
                                                                                it.getCoordinate() + octomap::point3d(1,1,1)); 
         neighbor != octree->end_leafs_bbx(); ++neighbor) {
        if (octree->isNodeOccupied(*neighbor)) {
            known_count++;
        } else {
            unknown_count++;
        }
    }
    double unknown_ratio = static_cast<double>(unknown_count) / (known_count + unknown_count);
    // A voxel is considered a frontier if it has at least one known neighbor (it is adjacent to known space)
    // and the number of unknown neighbors is greater than a certain threshold (it is adjacent to unknown space)
    // The exact threshold depends on your specific application and may need to be tuned
    if (known_count > 0 && unknown_ratio > 0.5) {
        return true;
    } else {
        return false;
    }
}

geometry_msgs::Point selectGoalPoint(const std::vector<geometry_msgs::Point>& centers) {
    // For simplicity, just returning the first point in the cluster. 
    // Replace with your own logic for selecting goal point
    return centers.front();
}

std::pair<std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::RGB>>, std::vector<geometry_msgs::Point>> clusterFrontiers(const std::vector<octomap::OcTree::iterator>& frontiers) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& frontier : frontiers) {
        pcl::PointXYZ point;
        point.x = frontier.getX();
        point.y = frontier.getY();
        point.z = frontier.getZ();
        cloud->points.push_back(point);
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.2); // 2cm
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::RGB>> clusters;
    pcl::common::UniformGenerator<int> gen(0, 255);

    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point_idx : indices.indices)
            cloud_cluster->points.push_back(cloud->points[point_idx]);
        
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::RGB color(gen.run(), gen.run(), gen.run());

        clusters.push_back(std::make_pair(cloud_cluster, color));
    }

    std::vector<geometry_msgs::Point> centers;
    for (const auto& pair : clusters) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pair.first;
        // Compute centroid manually
        geometry_msgs::Point center;
        center.x = 0.0;
        center.y = 0.0;
        center.z = 0.0;
        for (const auto& point : cloud->points) {
            center.x += point.x;
            center.y += point.y;
            center.z += point.z;
        }
        center.x /= cloud->points.size();
        center.y /= cloud->points.size();
        center.z /= cloud->points.size();
        centers.push_back(center);
    }

    return std::make_pair(clusters, centers);
}

visualization_msgs::MarkerArray createVisualizationMarkers(const std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::RGB>>& clusters) {
    visualization_msgs::MarkerArray markers;
    
    for (size_t i = 0; i < clusters.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = clusters[i].first;
        
        // Compute centroid manually
        pcl::PointXYZ centroid;
        centroid.x = 0.0;
        centroid.y = 0.0;
        centroid.z = 0.0;
        for (const auto& point : cloud->points) {
            centroid.x += point.x;
            centroid.y += point.y;
            centroid.z += point.z;
        }
        centroid.x /= cloud->points.size();
        centroid.y /= cloud->points.size();
        centroid.z /= cloud->points.size();

        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time();
        marker.ns = "clusters";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point point;
        point.x = centroid.x;
        point.y = centroid.y;
        point.z = centroid.z;
        marker.pose.position = point;

        marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
        marker.color.a = 1.0; 
        marker.color.r = clusters[i].second.r / 255.0;
        marker.color.g = clusters[i].second.g / 255.0;
        marker.color.b = clusters[i].second.b / 255.0;

        markers.markers.push_back(marker);
    }
    
    return markers;
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_subscriber");
    ros::NodeHandle nh;
    std::string topic_name = "/octomap_full";
    ros::Subscriber sub = nh.subscribe<octomap_msgs::Octomap>(topic_name, 1, octomapCallback);
    markerPub = nh.advertise<visualization_msgs::MarkerArray>("clusters", 1);
    goalPub = nh.advertise<geometry_msgs::PointStamped>("move_base_simple/goal", 1);
    ros::spin();

    return 0;
}