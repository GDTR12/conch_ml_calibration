#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace bag2pcd
{

namespace fs = std::filesystem;
// typedef pcl::PointXYZRGB PointT;
template<typename PointT>
void rosbag2PointCloud(const fs::path& bag_path, const std::string& lidar_topic, std::vector<pcl::shared_ptr<pcl::PointCloud<PointT>>>& ret)
{
    using Cloud = pcl::PointCloud<PointT>;
    using CloudPtr = pcl::shared_ptr<pcl::PointCloud<PointT>>;
    try {
        rosbag::Bag bag;
        bag.open(bag_path, rosbag::bagmode::Read);

        rosbag::View view(bag, rosbag::TopicQuery({lidar_topic}));
        if (view.size() == 0){
            throw std::invalid_argument("lidar topic has no msg!");
            return;
        }

        for (const rosbag::MessageInstance& msg : view) {
            sensor_msgs::PointCloud2::ConstPtr cloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();
            if (cloud_msg == nullptr) continue;
            CloudPtr pcl_cloud = pcl::make_shared<Cloud>();

            pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
            ret.push_back(pcl_cloud);
        }
        
        bag.close();
    } 
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("Error reading rosbag: " << e.what());
        return;
    }
}
    
} // namespace bag2pcd

