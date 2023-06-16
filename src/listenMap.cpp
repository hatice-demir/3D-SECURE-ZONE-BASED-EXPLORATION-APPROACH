#include "octomap/octomap.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "iostream"
#include "vector"
#include "queue"
#include "set"
#include "visualization_msgs/MarkerArray.h"
#include "ros/ros.h"
#include "octomap/OcTree.h"
#include "algorithm"
#include "cmath"
using namespace std;
using namespace octomap;

ros::Publisher markerPub;
std::vector<octomap::OcTree::leaf_iterator> frontierCells; // Frontier hücrelerini saklamak için vektör

// Öbekleme işlemini gerçekleştiren işlev
std::vector<std::vector<octomap::OcTree::leaf_iterator>> clusterFrontierCells(const std::vector<octomap::OcTree::leaf_iterator>& frontierCells) {
    std::vector<std::vector<octomap::OcTree::leaf_iterator>> clusters;  // Öbekleri tutmak için vektör

    for (const auto& cell : frontierCells) {
        bool isAddedToCluster = false;

        // Mevcut hücreyi uygun öbeğe eklemeye çalış
        for (auto& cluster : clusters) {
            if (cluster.size() < 200) {
                // Mevcut öbeye ekle
                cluster.push_back(cell);
                isAddedToCluster = true;
                break;
            }
        }

        // Mevcut öbeklere eklenemediyse, yeni bir öbek oluştur ve hücreyi eklemeyi dene
        if (!isAddedToCluster) {
            std::vector<octomap::OcTree::leaf_iterator> newCluster;
            newCluster.push_back(cell);
            clusters.push_back(newCluster);
        }
    }

    return clusters;
}

// İki hücre arasındaki mesafeyi hesaplayan işlev
double distanceBetweenCells(const octomap::OcTree::leaf_iterator& cell1, const octomap::OcTree::leaf_iterator& cell2) {
    // Hücrelerin koordinatlarını al
    double x1 = cell1.getX();
    double y1 = cell1.getY();
    double z1 = cell1.getZ();

    double x2 = cell2.getX();
    double y2 = cell2.getY();
    double z2 = cell2.getZ();

    // Hücreler arasındaki Euclidean mesafesini hesapla
    double distance = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2) + std::pow(z1 - z2, 2));

    return distance;
}

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {

    // OctoMap mesajını OctoMap veri yapısına dönüştür
    octomap::AbstractOcTree* abstractOcTree = octomap_msgs::msgToMap(*msg);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(abstractOcTree);

    // Frontier hücreleri saklamak için set oluştur
    //std::set<octomap::OcTreeNode*> frontier_cells;
    frontierCells.clear();

    //std::vector<octomap::OcTree::leaf_iterator> frontierCells; // Frontier hücrelerini saklamak için vektör

    // Hücre koordinatlarını yazdır ve frontier noktaları bul
    int num_cells = 0;
    for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {
        octomap::point3d point = it.getCoordinate();
        //ROS_INFO_STREAM(num_cells << ".hucre koordinatlari: (" << point.x() << ", " << point.y() << ", " << point.z() << ")");
        num_cells++;

        // Vokselin durumunu kontrol et
        if (!octree->isNodeOccupied(*it)) {

            int isFrontier = 0;

            // Vokselin komşularını kontrol et
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0)
                            continue;
                        
                        octomap::point3d neighbor_point = it.getCoordinate() + octomap::point3d(dx, dy, dz);
                        octomap::OcTreeNode* neighbor_node = octree->search(neighbor_point);

                        // Komşu hücre bilinmeyen veya boş hücre ise, bu hücre tekil sınır hücresidir
                        if (neighbor_node == nullptr || !octree->isNodeOccupied(neighbor_node)) {
                            isFrontier = isFrontier + 1;
                            if(isFrontier > 25) {
                                //frontier_cells.insert(&(*it));
                                frontierCells.push_back(it);
                                break;
                            }
                        }
                    }
                    if (isFrontier > 25)
                        break;
                }
                if (isFrontier > 25)
                    break;
            }
        }
    }

    // Hücre sayısını yazdır
    ROS_INFO_STREAM("Hucre sayisi: " << num_cells);
    // Tekil sınır hücre sayısını yazdır
    ROS_INFO_STREAM("Frontier hucre sayisi: " << frontierCells.size());

    for (const octomap::OcTree::leaf_iterator& frontierCell : frontierCells)
    {
        octomap::point3d cellCenter = frontierCell.getCoordinate();
        //std::cout << "X: " << cellCenter.x() << ", Y: " << cellCenter.y() << ", Z: " << cellCenter.z() << std::endl;

    }

    // frontierCells vektöründeki hücreleri öbeklere grupla
    std::vector<std::vector<octomap::OcTree::leaf_iterator>> clusters = clusterFrontierCells(frontierCells);

    // Her bir öbek için hücreleri yazdır ve farklı renkte markerlar oluştur
    int clusterCount = 1;
    visualization_msgs::MarkerArray markers;
    int markerId = 0;

    for (const auto& cluster : clusters) {
        std::cout << "Cluster " << clusterCount << " (" << cluster.size() << " cells):" << std::endl;
        double sumX = 0.0;
        double sumY = 0.0;
        double sumZ = 0.0;
        for (const auto& cell : cluster) {
            //std::cout << "(" << cell.getX() << ", " << cell.getY() << ", " << cell.getZ() << ")" << std::endl;
            sumX += cell.getX();
            sumY += cell.getY();
            sumZ += cell.getZ();
        }
        double averageX = sumX / cluster.size();
        double averageY = sumY / cluster.size();
        double averageZ = sumZ / cluster.size();

        std::cout << "Average coordinates: (" << averageX << ", " << averageY << ", " << averageZ << ")" << std::endl;
        std::cout << std::endl;

        // Create a marker for the cluster
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "clusters";
        marker.id = markerId++;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        // Assign a different color to each cluster
        double r = static_cast<double>(rand()) / RAND_MAX;
        double g = static_cast<double>(rand()) / RAND_MAX;
        double b = static_cast<double>(rand()) / RAND_MAX;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;

        // Set marker properties
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        // Add the cells of the cluster to the marker
        for (const auto& cell : cluster) {
            octomap::point3d cellCenter = cell.getCoordinate();
            geometry_msgs::Point p;
            p.x = cellCenter.x();
            p.y = cellCenter.y();
            p.z = cellCenter.z();
            marker.points.push_back(p);
        }

        // Add the marker to the marker array
        markers.markers.push_back(marker);

        clusterCount++;
    }

    markerPub.publish(markers);

    // 2B yükseklik haritasını oluştur
    //std::vector<std::vector<double>> heightMap = createHeightMap(octree, 0.1);

    // 2B yükseklik haritasını yazdır
    /*
    for (int y = 0; y < heightMap.size(); ++y) {
        for (int x = 0; x < heightMap[y].size(); ++x) {
            std::cout << heightMap[y][x] << " ";
        }
        std::cout << std::endl;
    }*/

    delete octree;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_subscriber");
    ros::NodeHandle nh;
    std::string topic_name = "/octomap_full";
    ros::Subscriber sub = nh.subscribe<octomap_msgs::Octomap>(topic_name, 1, octomapCallback);
    markerPub = nh.advertise<visualization_msgs::MarkerArray>("clusters", 1);
    ros::spin();

    return 0;
}
