#ifndef RVIZ_VISUAL_TOOLS_H
#define RVIZ_VISUAL_TOOLS_H

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

class RvizMarker
{
    public:
        explicit RvizMarker(ros::NodeHandle nh);
        ~RvizMarker();
    
    private:
        ros::NodeHandle nh_;
        ros::Subscriber pcl_sub;
        rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

        void pclCallback(const sensor_msgs::PointCloud2& cloud_msg);
};

#endif /* RVIZ_VISUAL_TOOLS_H */