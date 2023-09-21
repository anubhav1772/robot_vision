#ifndef CLOUD_TRANSFORMER_H
#define CLOUD_TRANSFORMER_H

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

class CloudTransformer
{
public:
  explicit CloudTransformer(ros::NodeHandle nh);
  ~CloudTransformer();

private:
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Publisher pcl_pub_;
  tf::TransformListener listener_;
  sensor_msgs::PointCloud2::Ptr buffer_;
  tf::StampedTransform transform;

  void pclCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_msg);

};

#endif
