#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

/*
* This node transforms point cloud data from /front_camera_rgb_optical_frame frame to /world frame
*/

class CloudTransformer
{
public:
  explicit CloudTransformer(ros::NodeHandle nh)
    : nh_(nh) 
  {
    // pcl_sub_ = nh_.subscribe("/front_camera/depth_registered/points", 1, &CloudTransformer::pclCallback, this);
    pcl_sub_ = nh_.subscribe("/front/pcl_downsampled", 1, &CloudTransformer::pclCallback, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/front_transformed_point_cloud", 1);

    buffer_.reset(new sensor_msgs::PointCloud2);
    buffer_->header.frame_id = "/front_camera_rgb_optical_frame";
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Publisher pcl_pub_;
  tf::TransformListener listener_;
  sensor_msgs::PointCloud2::Ptr buffer_;
  tf::StampedTransform transform;

  void pclCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_msg)
  {
    try
    {
      ros::Time now = ros::Time::now();
      ros::Time past = now - ros::Duration(1.0);

      bool transformPossible = listener_.waitForTransform("/world", 
                                                          (*pcl_msg).header.frame_id, 
                                                          (*pcl_msg).header.stamp, 
                                                          ros::Duration(1.0));
      if (transformPossible == false)
      {
        ROS_INFO("Transform not possible");
      }   
      else{
        ROS_INFO("Transform possible");
      }

      listener_.lookupTransform("/world", (*pcl_msg).header.frame_id, past, transform);
    }
    catch(tf::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
    };

    pcl_ros::transformPointCloud("/world", *pcl_msg, *buffer_, listener_);
    pcl_pub_.publish(buffer_);
  }
};  // end of class CloudTransformer

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_tf");
  ros::NodeHandle nh;

  CloudTransformer transform_cloud(nh);

  // spin until ROS is shutdown
  while (ros::ok())
    ros::spin();

  return 0;
}


/*
  bool tf::TransformListener::waitForTransform (const std::string &target_frame, 
                                                const std::string &source_frame, 
                                                const ros::Time &time, 
                                                const ros::Duration &timeout, 
                                                const ros::Duration &polling_sleep_duration=ros::Duration(0.01), 
                                                std::string *error_msg=NULL) const 
  Test if source_frame can be transformed to target_frame at time time.

  The waitForTransform() methods return a bool whether the transform can be evaluated. 
  It will sleep and retry every polling_duration until the duration of timeout has been passed. 
  It will not throw. If you pass a non NULL string pointer it will fill the string error_msg in 
  the case of an error. (Note: That this takes notably more resources to generate the error message.)

  bool tf::TransformListener::waitForTransform (const std::string &target_frame, 
                                                const ros::Time &target_time, 
                                                const std::string &source_frame, 
                                                const ros::Time &source_time, 
                                                const std::string &fixed_frame, 
                                                const ros::Duration &timeout, 
                                                const ros::Duration &polling_sleep_duration=ros::Duration(0.01), 
                                                std::string *error_msg=NULL) const 
  Test if source_frame can be transformed to fixed_frame at time source_time, and 
  from there can be transformed into target_frame at target_time.

  void tf::TransformListener::lookupTransform (const std::string &target_frame, 
                                               const ros::Time &target_time, 
                                               const std::string &source_frame, 
                                               const ros::Time &source_time, 
                                               const std::string &fixed_frame, 
                                               StampedTransform &transform) const 
  Fill transform with the transform from source_frame to fixed_frame at source_time, 
  chained with the transform from fixed_frame to target_frame at target_time 
*/