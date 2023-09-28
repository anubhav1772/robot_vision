#include "robot_vision/downsampling.h"

/*
* ROS node for downsampling point cloud data.
*/

Downsampling::Downsampling(ros::NodeHandle nh)
    : nh_(nh)
{
    pcl_sub = nh_.subscribe("/camera/depth_registered/points", 1, &Downsampling::pclCallback, this);
    pcl_downsampled_pub = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_downsampled", 1);
}

Downsampling::~Downsampling(){}

void Downsampling::pclCallback(const sensor_msgs::PointCloud2& cloud_msg)
{
    ROS_INFO("Point Cloud Cluster Received.");

    pcl::PointCloud<pcl::PointXYZRGB> *p_cloud = new pcl::PointCloud<pcl::PointXYZRGB>();
    pcl::PointCloud<pcl::PointXYZRGB> *downsampled = new pcl::PointCloud<pcl::PointXYZRGB>();

    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > sp_pcl_cloud(p_cloud);
    pcl::fromROSMsg(cloud_msg, *p_cloud);

    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > sp_pcl_downsampled_cloud(downsampled);

    ROS_INFO("---------------------------------");
    ROS_INFO("Point Cloud Cluster Stats:");
    ROS_INFO("Height: %d", sp_pcl_cloud->height);
    ROS_INFO("Weight: %d", sp_pcl_cloud->width);
    ROS_INFO("Total data points : %d", sp_pcl_cloud->height*sp_pcl_cloud->width);
    // ROS_INFO("",*p_cloud);
    ROS_INFO("---------------------------------");
            
    ROS_INFO("Starting Voxel Grid Downsampling...");
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(sp_pcl_cloud);
    vox.setLeafSize(0.01f, 0.01f, 0.01f);
    vox.filter(*downsampled);
    // ROS_INFO("Total data points after downsampling: %d", sp_pcl_cloud->height*sp_pcl_cloud->width);

    sensor_msgs::PointCloud2 pcl_downsampled;
    pcl::toROSMsg(*downsampled, pcl_downsampled);
    pcl_downsampled_pub.publish(pcl_downsampled);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "downsampling");
    ros::NodeHandle nh;

    Downsampling downsampling(nh);

    // Spin until ROS is shutdown
    while (ros::ok())
        ros::spin();

    return 0;
}
