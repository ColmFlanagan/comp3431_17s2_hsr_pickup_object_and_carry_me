#include <ros/ros.h>
#include <myvis/Table.h>
#include <myvis/Tables.h>
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
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>        // std::abs
#include <unistd.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <visualization_msgs/Marker.h>
#include <pcl/common/common.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
tf::TransformListener *tf_listener; 

class Subscribe_And_Publish
{
private:
  ros::Publisher pub;
  ros::Publisher pub2;
  ros::Publisher pubTable;

  ros::Subscriber sub;
  ros::NodeHandle nh;

public:


  Subscribe_And_Publish()
  {
    pub2 = nh.advertise<visualization_msgs::Marker>("vm",1);
    sub = nh.subscribe<PointCloud>("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, &Subscribe_And_Publish::callback, this);
    pub = nh.advertise<PointCloud>("table_cluster", 1);
    pubTable = nh.advertise<myvis::Tables>("Tables", 1);

  }


  void callback(const PointCloud::ConstPtr& cloud)
  {
    PointCloud::Ptr flat(new PointCloud);
    PointCloud::Ptr cloud_f (new PointCloud);
    PointCloud::Ptr cloud_pass(new PointCloud);

    pcl::PassThrough<pcl::PointXYZ> pass_through_filter;
    //pass through filter
    pass_through_filter.setInputCloud (cloud);
    pass_through_filter.filter (*cloud_pass);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    PointCloud::Ptr cloud_filtered (new PointCloud);
    vg.setInputCloud (cloud_pass);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    PointCloud::Ptr cloud_plane (new PointCloud ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    int i=0, nr_points = (int) cloud_filtered->points.size ();

    while (cloud_filtered->points.size () > 0.01 * nr_points)
    {

      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0){break;}

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloud_plane);

      if(coefficients->values[0] > -0.3 && coefficients->values[0] < 0.3 ){
        if(coefficients->values[1] > -1.3 && coefficients->values[1] < 1.3 ){
          if(coefficients->values[2] > -0.3 && coefficients->values[2] < 0.3 ){
              PointCloud::Ptr cxa(new PointCloud);

              pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
              sor.setInputCloud(cloud_plane);
              sor.setMeanK(100);
              sor.setStddevMulThresh(0.5);
              sor.filter(*cxa);
              *flat += *cxa;
          } 
        }
      }
      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloud_filtered = *cloud_f;

    }

    flat->header.frame_id = "head_rgbd_sensor_rgb_frame";
    flat->header.stamp = ros::Time::now().toNSec();


    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (flat);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.1); // 2cm
    ec.setMinClusterSize (500);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (flat);
    ec.extract (cluster_indices);
    int count = 0;

    myvis::Tables tableArray;
    tableArray.Tables[cluster_indices.size()];

 for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    count++;
    PointCloud::Ptr cloud_cluster (new PointCloud);
    PointCloud::Ptr cloud_cluster2 (new PointCloud);

    cloud_cluster->header.frame_id = "head_rgbd_sensor_rgb_frame";
    cloud_cluster->header.stamp = ros::Time::now().toNSec();
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (flat->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    float averageX = 0.0f;
    float averageY = 0.0f;
    float averageZ = 0.0f;
    ros::Time now = ros::Time::now();

    tf::StampedTransform transform;
    try{
      //usleep(10000);
      tf_listener->waitForTransform("/base_footprint", "/head_rgbd_sensor_rgb_frame" ,now, ros::Duration(2.0));
      tf_listener->lookupTransform("/base_footprint", "/head_rgbd_sensor_rgb_frame",  now, transform);
      pcl_ros::transformPointCloud  ( *cloud_cluster,*cloud_cluster2,transform);
      cloud_cluster2->header.frame_id = "base_link";

      for(size_t i = 0; i < cloud_cluster2->points.size();i++){
        averageX= averageX+ cloud_cluster2->points[i].x;
        averageY= averageY+cloud_cluster2->points[i].y;
        averageZ= averageZ+cloud_cluster2->points[i].z;

      }
      averageX = averageX/cloud_cluster2->points.size();
      averageY = averageY/cloud_cluster2->points.size();
      averageZ = averageZ/cloud_cluster2->points.size();
      std::cout << "table "<< count <<" at " << averageX << ", " << averageY << ", " << averageZ << ", " << " data points." << std::endl;

      visualization_msgs::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp =  ros::Time::now();
      marker.ns = "basic_shapes";
      marker.id = count;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = averageX;
      marker.pose.position.y = averageY;
      marker.pose.position.z = averageZ;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
      marker.pose.orientation.w = 0;

      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;

      marker.color.r = 0.5;
      marker.color.g = 0.5;
      marker.color.b = 0.5;
      marker.color.a = 1;

      marker.lifetime = ros::Duration();

      pub2.publish(marker);
      pcl::PointXYZ minPt, maxPt;
      pcl::getMinMax3D (*cloud_cluster2, minPt, maxPt);
      std::cout << "Max x: " << maxPt.x << std::endl;
      std::cout << "Max y: " << maxPt.y << std::endl;
      std::cout << "Max z: " << maxPt.z << std::endl;
      std::cout << "Min x: " << minPt.x << std::endl;
      std::cout << "Min y: " << minPt.y << std::endl;
      std::cout << "Min z: " << minPt.z << std::endl;
      pcl_conversions::toPCL(ros::Time::now(), cloud_cluster2->header.stamp);
      pub.publish(*cloud_cluster2);

    }catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }

};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "table_pcl");
  tf_listener    = new tf::TransformListener();   
  Subscribe_And_Publish SAPObject;

  ros::spin();
}
