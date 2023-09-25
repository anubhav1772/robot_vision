#ifndef PREPROCESSING_H
#define PREPROCESSING_H

#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// EuclideanClusterExtraction
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>

#include <random>
#include <vector>

#include "geometry_msgs/Point.h"
#include "robot_vision/SegmentedClusterCentroids.h"

// color intensities [0, 255]
const int inten[] = {0, 42, 99, 128, 192, 255};

class Preprocess
{
    public:
        explicit Preprocess(ros::NodeHandle nh);
        ~Preprocess();
        void generate_random_colors(int num_of_clusters);

    private:
        ros::NodeHandle nh_;
        std::vector<uint32_t> colors_list; 
        ros::Subscriber pcl_sub;
        ros::Publisher pcl_outlers_removed_pub;
        ros::Publisher pcl_downsampled_pub;
        ros::Publisher pcl_inliers_pub;
        ros::Publisher pcl_outliers_pub;
        ros::Publisher pcl_passthroughx_pub;
        ros::Publisher pcl_passthroughy_pub;
        ros::Publisher pcl_passthroughz_pub;
        ros::Publisher pcl_cloud_table_pub;
        ros::Publisher pcl_cloud_objects_pub;
        ros::Publisher pcl_clusters_pub;
        ros::Publisher pcl_clusters_centroid_pub;
        ros::Publisher pcl_centroids_pub;

        //ros::Publisher cloud_cluster_centroids_pub;
        //robot_vision::SegmentedClusterCentroids centroids_msg;
        std::random_device dev;

        void pclCallback(const sensor_msgs::PointCloud2& cloud_msg);
};

#endif /* PREPROCESSING_H */