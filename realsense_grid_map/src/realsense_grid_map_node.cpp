#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <Eigen/Dense>

class GridHeightMapNode
{
public:
    GridHeightMapNode()
    {
        pointcloud_sub_ = nh_.subscribe("/camera/depth/color/points", 1, &GridHeightMapNode::pointCloudCallback, this);
        grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_height_map", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher grid_map_pub_;

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        grid_map::GridMap map({"elevation"});
        map.setFrameId("map");
        map.setGeometry(grid_map::Length(10.0, 10.0), 0.1); // can change parameter from testing

        map["elevation"].setConstant(NAN); // should I use another number?

        for (const auto &point : cloud->points)
        {
            if (!std::isfinite(point.z))
                continue;  

            grid_map::Position position(point.x, point.y);
            if (map.isInside(position))
            {
         
                map.atPosition("elevation", position) = point.z;
            }
        }

        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(map, message);
        grid_map_pub_.publish(message);

        ROS_INFO("Published Grid Height Map");
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grid_height_map_node");
    GridHeightMapNode node;
    ros::spin();
    return 0;
}


// #include <ros/ros.h>
// #include <sensor_msgs/Image.h>
// #include <grid_map_ros/grid_map_ros.hpp>
// #include <grid_map_cv/grid_map_cv.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <Eigen/Dense>
// #include <fstream>

// class RealSenseGridMap {
//  public:
//     RealSenseGridMap() {
//         depth_sub_ = nh_.subscribe("/camera/depth/image_rect_raw", 1,
//                                    &RealSenseGridMap::depthCallback, this);
//         grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map",
//                                                               1, true);
//     }

//     void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
//         cv_bridge::CvImagePtr cv_ptr;
//         grid_map::GridMap map({"elevation"});
//         map.setFrameId("map");
//         map.setGeometry(grid_map::Length(1, 1), 0.05);

//         for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
//             grid_map::Position position;
//             map.getPosition(*it, position);
//             int x = (position.x() + 0.5) * cv_ptr->image.cols;
//             int y = (position.y() + 0.5) * cv_ptr->image.rows;

//             if (x >= 0 && x < cv_ptr->image.cols &&
//                 y >= 0 && y < cv_ptr->image.rows) {
//                 // mm to m
//                 float depth = cv_ptr->image.at<uint16_t>(y, x) * 0.001;
//                 map.at("elevation", *it) = depth;
//             }
//         }

//         grid_map_msgs::GridMap message;
//         grid_map::GridMapRosConverter::toMessage(map, message);
//         grid_map_pub_.publish(message);
//     }

//  private:
//     ros::NodeHandle nh_;
//     ros::Subscriber depth_sub_;
//     ros::Publisher grid_map_pub_;
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "realsense_grid_map_node");
//     RealSenseGridMap node;
//     ros::spin();
//     return 0;
// }


// // roslaunch realsense_grid_map quad_with_realsense.launch

