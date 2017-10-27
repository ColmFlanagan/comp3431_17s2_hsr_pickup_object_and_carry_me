#include <ros/ros.h>
#include <tmc_darknet_msgs/Object.h>
#include <tmc_darknet_msgs/Objects.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <math.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>        // std::
class Subscribe_And_Publish
{
private:
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::NodeHandle n;
public:


    Subscribe_And_Publish()
    {
        sub = n.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > ("/camera/camera_modelet_manager/camera/depth_registered/points", 5, &Subscribe_And_Publish::callback, this);

        pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("feature_coordinates", 500);
    }

    void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
    {

    }

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
    ros::init(argc, argv, "front_end");
    Subscribe_And_Publish SAPObject;

    ros::spin();

    return 0;
}
