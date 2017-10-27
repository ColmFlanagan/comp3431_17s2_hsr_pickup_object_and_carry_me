#include <ros/ros.h>
#include <myvis/Object.h>
#include <myvis/Objects.h>
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
#include <pcl/common/common.h>
#include <std_msgs/String.h>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
tf::TransformListener *tf_listener; 

class Subscribe_And_Publish
{
private:
    ros::Publisher pub;
    ros::Publisher pubObs;
    ros::Subscriber sub;
    ros::Subscriber Objects_sub;
    ros::Subscriber topick_sub;
    
    std_msgs::String topick;
	myvis::Objects obs;
	 
    ros::NodeHandle nh;

public:


    Subscribe_And_Publish()
    {
		topick_sub = nh.subscribe("topick", 1, &Subscribe_And_Publish::topick_callback, this);
		Objects_sub = nh.subscribe("Objects", 1, &Subscribe_And_Publish::Objects_callback, this);
		sub = nh.subscribe<PointCloud>("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, &Subscribe_And_Publish::callback, this);
		pub = nh.advertise<PointCloud>("pcl_cluster", 1);
		pubObs = nh.advertise<myvis::Object>("Objects_final", 1);
    }

	void Objects_callback(const myvis::Objects::ConstPtr& msg)
	{
		obs = *msg;
	}
	void topick_callback(const std_msgs::String::ConstPtr& msg)
	{
		std::cout << msg->data.c_str()<< std::endl;
		topick.data = msg->data.c_str();
	}
	void callback(const PointCloud::ConstPtr& cloud)
	{
		PointCloud::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
		PointCloud::Ptr cloud_pass(new PointCloud);
		pcl::PassThrough<pcl::PointXYZ> pass_through_filter;
		//pass through filter
		pass_through_filter.setInputCloud ((*cloud).makeShared());
		pass_through_filter.filter (*cloud_pass);

		// Create the filtering object: downsample the dataset using a leaf size of 2cm
		pcl::VoxelGrid<pcl::PointXYZ> vg;
		PointCloud::Ptr cloud_filtered (new PointCloud);
		vg.setInputCloud ((*cloud_pass).makeShared());
		vg.setLeafSize (0.02f, 0.02f, 0.02f);
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
		seg.setMaxIterations (1000);
		seg.setDistanceThreshold (0.1);

		int i=0, nr_points = (int) cloud_filtered->points.size ();
		while (cloud_filtered->points.size () > 0.3 * nr_points)
		{
					
			seg.setInputCloud ((*cloud_filtered).makeShared());
			seg.segment (*inliers, *coefficients);

			if (inliers->indices.size () == 0)
			{
			  break;
			}

			// Extract the planar inliers from the input cloud
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud ((*cloud_filtered).makeShared());
			extract.setIndices (inliers);
			extract.setNegative (false);

			// Get the points associated with the planar surface
			extract.filter (*cloud_plane);

			// Remove the planar inliers, extract the rest
			extract.setNegative (true);
			extract.filter (*cloud_f);
			*cloud_filtered = *cloud_f;

		}

		// Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud ((*cloud_filtered).makeShared());

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance (0.02); // 2cm
		ec.setMinClusterSize (50);
		ec.setMaxClusterSize (2500);
		ec.setSearchMethod (tree);
		ec.setInputCloud ((*cloud_filtered).makeShared());
		ec.extract (cluster_indices);

		// array of nearest point cloud cluster to detected object
		std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > cloud_clusters_ptr (obs.length);

		int flag = 0;	// flag to make sure array is not empty 
		int j = 0;

		int rank = 0;

		float closest[ obs.length];  // an array to store the closes distance beteween object and point cloud
		for(size_t k = 0; k < obs.length ;k++){
				closest[k] = 100.0f;
		}

		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			PointCloud::Ptr cloud_cluster (new PointCloud);
			cloud_cluster->header.frame_id = "map";
			cloud_cluster->header.stamp = ros::Time::now().toNSec();
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			  cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			j++;

			cloud_cluster->header.frame_id = cloud->header.frame_id;
			cloud_cluster->header.stamp = ros::Time::now().toNSec();
			float averageX = 0.0f;
			float averageY = 0.0f;
			float averageZ = 0.0f;


			for(size_t i = 0; i < cloud_cluster->points.size();i++){
				averageX= averageX+ cloud_cluster->points[i].x;
				averageY= averageY+cloud_cluster->points[i].y;
				averageZ= averageZ+cloud_cluster->points[i].z;
			}

			averageX = averageX/cloud_cluster->points.size();
			averageY = averageY/cloud_cluster->points.size();
			averageZ = averageZ/cloud_cluster->points.size();
			float ave = sqrt(averageX*averageX + averageY*averageY + averageZ*averageZ);
			float dist = 0.1f;

			for(size_t k = 0; k < obs.length ;k++){

				float Xdist = obs.Objects[k].x;
				float Ydist = obs.Objects[k].y;
				float Zdist =  obs.Objects[k].z;

				myvis::Objects obs2;
				if(averageX < (Xdist +dist) && averageX > (Xdist-dist)){
						if(averageZ < (Zdist +dist) && averageZ > (Zdist-dist)){

								float distbetween = std::abs(Xdist - averageX)	+  std::abs(Zdist - averageZ) +  std::abs(Ydist - averageY);				
								if(closest[k] > distbetween){
									closest[k] = distbetween;
									cloud_clusters_ptr[k] = cloud_cluster;
										if(obs.Objects[k].name == topick.data){
											std::cout << "found Object " << j << " ." << std::endl;
											std::cout << "x " << averageZ << " ." << std::endl;
											std::cout << "y " << averageX << " ." << std::endl;
											std::cout << "z " << averageY << " ." << std::endl;

											flag = 1;
											rank = k;
										}
								}
			  					std::cout << "found "<< obs.Objects[k].name << " at j : " << j << " ." << std::endl;
			  				
						}

				}
			}
		
	     
	  	}

		if(flag == 1){
			PointCloud::Ptr pcl_out (new PointCloud);
			ros::Time now = ros::Time::now();
			tf::StampedTransform transform;
			try{
				tf_listener->waitForTransform("/map", "/head_rgbd_sensor_rgb_frame" ,now, ros::Duration(2.0));
				tf_listener->lookupTransform("/map", "/head_rgbd_sensor_rgb_frame",  now, transform);
				pcl_ros::transformPointCloud 	( *cloud_clusters_ptr[rank],(*pcl_out),transform);
				float averageXG = 0.0f;
				float averageYG = 0.0f;
				float averageZG = 0.0f;


				for(size_t i = 0; i < pcl_out->points.size();i++){
					averageXG= averageXG+ pcl_out->points[i].x;
					averageYG= averageYG+ pcl_out->points[i].y;
					averageZG= averageZG+ pcl_out->points[i].z;
				}
				averageXG = averageXG/pcl_out->points.size();
				averageYG = averageYG/pcl_out->points.size();
				averageZG = averageZG/pcl_out->points.size();
				std::cout << "Global X" << averageXG <<std::endl;
				std::cout << "Global Y" << averageYG <<std::endl;
				std::cout << "Global Z" << averageZG <<std::endl;
				pcl::PointXYZ minPt, maxPt;
				pcl::getMinMax3D (*pcl_out, minPt, maxPt);
				myvis::Object abc;
				abc.x = averageXG;
				abc.y = averageYG;
				abc.camera_y = (maxPt.z - minPt.z);
				abc.camera_x = (maxPt.y - minPt.y);

				abc.z = averageZG;
				pubObs.publish(abc);
				std::cout << "Height " << (maxPt.z - minPt.z) <<std::endl;
				std::cout << "width " << (maxPt.y - minPt.y) <<std::endl;

				pcl_out->header.frame_id = "map";
				pcl_out->header.stamp = ros::Time::now().toNSec();

				pcl_conversions::toPCL(ros::Time::now(), pcl_out->header.stamp);
				pub.publish(*pcl_out);

			}

			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
	 	}
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sub_pcl");
	tf_listener    = new tf::TransformListener();		
	Subscribe_And_Publish SAPObject;

	ros::spin();
}
