#ifndef DOWNSAMPLING_H
#define DOWNSAMPLING_H

#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>

class Downsampling
{
    public:
        explicit Downsampling(ros::NodeHandle nh);
        ~Downsampling();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber pcl_sub;
        ros::Publisher pcl_downsampled_pub;

        void pclCallback(const sensor_msgs::PointCloud2& cloud_msg);
};


#endif /* DOWNSAMPLING_H */