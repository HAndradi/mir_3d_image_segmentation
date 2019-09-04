#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <my_pcl_tutorial/Clusters.h>
#include <my_pcl_tutorial/Cluster_Pixels.h>
#include <my_pcl_tutorial/Pixel_Coord.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher cloud_pub;
ros::Publisher clusters_pub;

void 
//cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input)
{
  //Downsample by x3
  int scale = 3;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_downsampled->width = input->width / scale;
  cloud_downsampled->height = input->height / scale;
  cloud_downsampled->points.resize(cloud_downsampled->width * cloud_downsampled->height);
  for( size_t i = 0, ii = 0; i < cloud_downsampled->height; ii += scale, i++){
    for( size_t j = 0, jj = 0; j < cloud_downsampled->width; jj += scale, j++){
      cloud_downsampled->at(j, i) = input->at(jj, ii); //at(column, row)
    }
  }

  //Remove points further than 1m away from the camera in z-direction
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_downsampled);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_through(new pcl::PointCloud<pcl::PointXYZ>);
  pass.filter (*cloud_pass_through);

  //Voxel grid filter for uniform pointcloud
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_pass_through);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  sor.filter (*cloud_filtered);

  //Estimate most dominant plane coefficients
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  //std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

  //Project plane model inliers to plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ProjectInliers<pcl::PointXYZ> project_inliers;
  project_inliers.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
  project_inliers.setInputCloud(cloud_filtered);
  project_inliers.setModelCoefficients(coefficients);
  project_inliers.setIndices(inliers);
  project_inliers.setCopyAllData(false);
  project_inliers.filter(*plane);

  //Compute plane convex hull
  pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConvexHull<pcl::PointXYZ> convex_hull;
  convex_hull.setInputCloud(plane);
  convex_hull.reconstruct(*hull);

  //Extract points inside polygonal prism of plane convex hull
  pcl::PointIndices::Ptr segmented_cloud_inliers(new pcl::PointIndices);
  pcl::ExtractPolygonalPrismData<pcl::PointXYZ> extract_polygonal_prism;
  extract_polygonal_prism.setInputPlanarHull(hull);
  extract_polygonal_prism.setInputCloud(cloud_downsampled);
  double z_min = 0.01;
  double z_max = 0.3;
  extract_polygonal_prism.setHeightLimits(z_min, z_max);
  extract_polygonal_prism.segment(*segmented_cloud_inliers);

/**
  // Extract the inliers
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud_downsampled);
  extract.setIndices (segmented_cloud_inliers);
  extract.setNegative (false);
  pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  extract.filter (*segmented_cloud);
**/

  //Find clusters of the inlier points
  pcl::search::OrganizedNeighbor<pcl::PointXYZ>::Ptr tree (new pcl::search::OrganizedNeighbor<pcl::PointXYZ>);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_downsampled);
  ec.setIndices (segmented_cloud_inliers);
  ec.extract (cluster_indices);

  my_pcl_tutorial::Clusters Clusters;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    my_pcl_tutorial::Cluster_Pixels Cluster_Pixels;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster->points.push_back (cloud_downsampled->points[*pit]); //*
      my_pcl_tutorial::Pixel_Coord Pixel_Coord;
      Pixel_Coord.y = (*pit / cloud_downsampled->width);
      Pixel_Coord.x = (*pit % cloud_downsampled->width);
      Cluster_Pixels.cluster_pixels.push_back(Pixel_Coord);
    }
    Clusters.clusters.push_back(Cluster_Pixels);
  }  
  cloud_cluster->width = cloud_cluster->points.size ();
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;
  //std::cerr << "cluster inliers: " << cloud_cluster->width << std::endl;

  sensor_msgs::PointCloud2 output;
  //pcl::toROSMsg(*segmented_cloud, output);
  pcl::toROSMsg(*cloud_cluster, output);
  output.header.frame_id = "cam_link";
  output.header.stamp = ros::Time::now();

  // Publish the cloud data
  cloud_pub.publish (output);

  // Publish the cluster data
  pcl_conversions::fromPCL(input->header.stamp, Clusters.header.stamp);
  Clusters.height = cloud_downsampled->height;
  Clusters.width = cloud_downsampled->width;
  Clusters.scale = scale;
  clusters_pub.publish (Clusters);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > ("camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Create a ROS publisher for the output point cloud
  clusters_pub = nh.advertise<my_pcl_tutorial::Clusters> ("clusters", 1);
  
  // Spin
  ros::spin ();
}
