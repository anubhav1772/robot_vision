#include "robot_vision/rviz_visual_tools.h"

/*
* This node publishes labeled axis markers at all the detected clusters' centroids.
*/

RvizMarker::RvizMarker(ros::NodeHandle nh)
    : nh_(nh)
{
    pcl_sub = nh_.subscribe("/transformed_centroids", 1, &RvizMarker::pclCallback, this);
    //visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("camera_rgb_optical_frame", "/rviz_visual_markers"));
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world", "/rviz_visual_markers"));
}
    
void RvizMarker::pclCallback(const sensor_msgs::PointCloud2& cloud_msg)
{
    ROS_INFO("Point Cloud Clusters' centroids Received.");

    pcl::PointCloud<pcl::PointXYZRGB> *p_centroids = new pcl::PointCloud<pcl::PointXYZRGB>();
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> sp_pcl_centroids(p_centroids);
    pcl::fromROSMsg(cloud_msg, *p_centroids);
            
    ROS_INFO("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
    ROS_INFO("Clusters' Centroids Stats:");
    ROS_INFO("Height: %d", sp_pcl_centroids->height);
    ROS_INFO("Width: %d", sp_pcl_centroids->width);

    int size = sp_pcl_centroids->height*sp_pcl_centroids->width;
    ROS_INFO("Total data points : %d", size);
    ROS_INFO("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
            
    int i=0;
    pcl::PointXYZRGB point;
    Eigen::Isometry3d pose;
    while(i<size)
    {
        point = (*sp_pcl_centroids)[i];
        //pose.translation().x() = point.x;
        //pose.translation().y() = point.y;
        //pose.translation().z() = point.z;
        pose.translation() = Eigen::Vector3d(point.x, point.y, point.z);
        //pose = pose * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
        visual_tools_->publishAxisLabeled(pose, "object"+std::to_string(i+1));
        i++;
    }
    visual_tools_->trigger();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "axes_rviz_markers");
    ros::NodeHandle nh;

    RvizMarker marker(nh);

    // Spin until ROS is shutdown
    while (ros::ok())
        ros::spin();

    return 0;
}