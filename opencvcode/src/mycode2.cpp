#include <ros/ros.h>
#include <iostream>
// PCL specific includes
 #include <pcl_conversions/pcl_conversions.h>
 #include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl/filters/frustum_culling.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
void cloud_cb(const pcl::PCLPointCloud2ConstPtr& input)
{

  //pcl::PCLPointCloud2 cloud_filtered;



    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pub.publish(temp_cloud);

pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;



	pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2);
	pcl::PCLPointCloud2::Ptr cloud_pass(new pcl::PCLPointCloud2);
	pcl::PassThrough<pcl::PCLPointCloud2> pass_through_filter;
    pass_through_filter.setInputCloud (input);
    pass_through_filter.filter (*cloud_pass);

    //voxel grid filter
    sor.setInputCloud(cloud_pass);
    sor.setLeafSize(0.05, 0.05, 0.05);
    sor.filter(*cloud_filtered);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*input,*temp_cloud);


pcl::FrustumCulling<pcl::PointXYZ> fc;
fc.setInputCloud (temp_cloud);
 fc.setVerticalFOV (45);
 fc.setHorizontalFOV (30);
 fc.setNearPlaneDistance (0);
 fc.setFarPlaneDistance (15);
Eigen::Matrix4f camera_pose;
 Eigen::Matrix4f pose_orig;
pose_orig << 0, 0, 1, 0,
              0,-1, 0, 0,
              1, 0, 0, 0,
              0, 0, 0, 1;
 Eigen::Matrix4f cam2robot;
 cam2robot << 0, 0, 1, 0,
              0,-1, 0, 0,
              1, 0, 0, 0,
              0, 0, 0, 1;
 Eigen::Matrix4f pose_new = pose_orig * cam2robot;
 fc.setCameraPose (pose_orig);
// .. read or input the camera pose from a registration algorithm.
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
fc.filter (*target);


	pub.publish(target);
}

int main (int argc, char** argv){
// Initialize ROS
ros::init (argc, argv, "sabir");
ros::NodeHandle nh;

 // Create a ROS subscriber for the input point cloud
ros::Subscriber sub = nh.subscribe ("/hsrb/head_rgbd_sensor/depth_registered/points", 1, cloud_cb);

// Create a ROS publisher for the output point cloud
pub = nh.advertise<PointCloud> ("output", 1);

// Spin
ros::spin ();
}
