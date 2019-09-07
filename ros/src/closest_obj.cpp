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
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/norms.h>
#include <Eigen/Eigenvalues>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher cloud_pub;
ros::Publisher pose_pub;
bool publish_output_pc;
std::string output_pc_frame;
float z_threshold;
float x_threshold;

void cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input)
{
    //Downsample by x3
    int scale = 3;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_downsampled->width = input->width / scale;
    cloud_downsampled->height = input->height / scale;
    cloud_downsampled->points.resize(cloud_downsampled->width * cloud_downsampled->height);
    for( size_t i = 0, ii = 0; i < cloud_downsampled->height; ii += scale, i++)
    {
        for( size_t j = 0, jj = 0; j < cloud_downsampled->width; jj += scale, j++)
        {
            cloud_downsampled->at(j, i) = input->at(jj, ii); //at(column, row)
        }
    }

    //Remove points further than 1m away from the camera in z-direction
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_downsampled);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, z_threshold);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_through(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter (*cloud_pass_through);

    //Only keep points in the middle strip
    pass.setInputCloud (cloud_pass_through);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-x_threshold, x_threshold);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_through2(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter (*cloud_pass_through2);

    /* //Find clusters of the inlier points */
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> clusters_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (400);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_pass_through2);
    ec.extract (clusters_indices);

    float closest_dist = 100.0;
    Eigen::Vector4f closest_centroid(0.0, 0.0, 0.0, 0.0);
    Eigen::Vector4f zero_point(0.0, 0.0, 0.0, 0.0);
    for (size_t i = 0; i < clusters_indices.size(); i++)
    {
        const pcl::PointIndices& cluster_indices = clusters_indices[i];
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_pass_through2, cluster_indices, centroid);
        float dist = pcl::L2_Norm(centroid, zero_point, 3);
        if (dist < closest_dist)
        {
            closest_dist = dist;
            closest_centroid = centroid;
        }
    }
    /* std::cout << closest_dist << std::endl; */
    /* std::cout << closest_centroid << std::endl; */

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = closest_centroid[0];
    pose_stamped.pose.position.y = closest_centroid[1];
    pose_stamped.pose.position.z = closest_centroid[2];
    pose_stamped.header.frame_id = input->header.frame_id;
    pose_stamped.header.stamp = ros::Time::now();
    pose_pub.publish(pose_stamped);

    if (publish_output_pc)
    {
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_pass_through2, output);
        output.header.frame_id = output_pc_frame;
        output.header.stamp = ros::Time::now();

        // Publish the cloud data
        cloud_pub.publish (output);
    }

}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pcl_closest_obj");
    ros::NodeHandle nh("~");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > ("input", 1, cloud_cb);

    nh.param<bool>("publish_output_pc", publish_output_pc, "true");
    nh.param<std::string>("output_pc_frame", output_pc_frame, "base_link");
    nh.param<float>("x_threshold", x_threshold, 0.05);
    nh.param<float>("z_threshold", z_threshold, 1.0);

    if (publish_output_pc)
    {
        // Create a ROS publisher for the output point cloud
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    }

    pose_pub = nh.advertise<geometry_msgs::PoseStamped> ("output_pose", 1);

    // Spin
    ros::spin ();
}
