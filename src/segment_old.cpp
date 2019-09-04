#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
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

ros::Publisher pub;

void 
//cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input)
{
  //Downsample by x3
  int scale1 = 3;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled_1(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_downsampled_1->width = input->width / scale1;
  cloud_downsampled_1->height = input->height / scale1;
  cloud_downsampled_1->points.resize(cloud_downsampled_1->width * cloud_downsampled_1->height);
  for( size_t i = 0, ii = 0; i < cloud_downsampled_1->height; ii += scale1, i++){
    for( size_t j = 0, jj = 0; j < cloud_downsampled_1->width; jj += scale1, j++){
      cloud_downsampled_1->at(j, i) = input->at(jj, ii); //at(column, row)
    }
  }

/**
  //Downsample by another x3
  int scale2 = 3;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled_2(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_downsampled_2->width = cloud_downsampled_1->width / scale2;
  cloud_downsampled_2->height = cloud_downsampled_1->height / scale2;
  cloud_downsampled_2->points.resize(cloud_downsampled_2->width * cloud_downsampled_2->height);
  for( size_t i = 0, ii = 0; i < cloud_downsampled_2->height; ii += scale2, i++){
    for( size_t j = 0, jj = 0; j < cloud_downsampled_2->width; jj += scale2, j++){
      cloud_downsampled_2->at(j, i) = cloud_downsampled_1->at(jj, ii); //at(column, row)
    }
  }
**/

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (input);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_through(new pcl::PointCloud<pcl::PointXYZ>);
  pass.filter (*cloud_pass_through);

  pcl::VoxelGrid<pcl::PointXYZ> sor0;
  sor0.setInputCloud (cloud_pass_through);
  sor0.setLeafSize (0.003f, 0.003f, 0.003f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_0(new pcl::PointCloud<pcl::PointXYZ>);
  sor0.filter (*cloud_filtered_0);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_filtered_0);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  //sor.setLeafSize (0.003f, 0.003f, 0.003f);
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
  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

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
  extract_polygonal_prism.setInputCloud(cloud_downsampled_1);
  double z_min = 0.01;
  double z_max = 0.3;
  extract_polygonal_prism.setHeightLimits(z_min, z_max);
  extract_polygonal_prism.segment(*segmented_cloud_inliers);

  // Extract the inliers
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud_downsampled_1);
  extract.setIndices (segmented_cloud_inliers);
  extract.setNegative (false);
  pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  extract.filter (*segmented_cloud);

  sensor_msgs::PointCloud2 output;
  //pcl::toROSMsg(*cloud_downsampled_1, output);
  pcl::toROSMsg(*segmented_cloud, output);
  output.header.frame_id = "cam_link";

//  sensor_msgs::PointCloud2 output;
//  output = *input;
  // Publish the data.
  pub.publish (output);
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
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
