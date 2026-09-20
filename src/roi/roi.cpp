#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cmath>

int main(){
    const float x_min = -10000, x_max = 10000;
    const float y_min = -10000, y_max = 10000;
    const float z_min = -2000, z_max = 2000;

    pcl::PointCloud<pcl::PointXYZ> roi_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test.pcd", *cloud) == -1){
        PCL_ERROR ("Couldn't read file test.pcd \n");
        return (-1);
    }
    for (const auto& point : *cloud){
        if (!std::isfinite(point.x) ||
            !std::isfinite(point.y) ||
            !std::isfinite(point.z)){
                continue;
            }
        if (point.x >= x_min && point.x <= x_max &&
            point.y >= y_min && point.y <= y_max &&
            point.z >= z_min && point.z <= z_max){
                roi_cloud.push_back(point);
            }
    }

    std::cout << "原始点数：" << cloud->size() << '\n';
    std::cout << "ROI 点数：" << roi_cloud.size() << '\n';

    pcl::io::savePCDFileASCII("roi.pcd", roi_cloud);

    return (0);
} 
