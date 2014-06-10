#include "shot.h"
typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::SHOT352 SHOT352;
typedef pcl::SHOT1344 SHOT1344;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<NormalType> NormalCloud;
typedef pcl::PointCloud<SHOT352> DescriptorCloudShot352;
typedef pcl::PointCloud<SHOT1344> DescriptorCloudShot1344;
typedef sensor_msgs::PointCloud2 KeypointMsg;
typedef object_recognition::Shot352_bundle Shot352Msg;
typedef object_recognition::Shot1344_bundle Shot1344Msg;


#include <iostream>
#include <dirent.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/time.h>
#include <ros/header.h>
#include <fstream>
#include <boost/foreach.hpp>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/narf.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

// Initialize subscriber and publisher for keypoints and descriptors
ros::Publisher pub_keypoints;
ros::Publisher pub_descriptors_Shot352;
ros::Publisher pub_descriptors_Shot1344;
ros::Subscriber sub;

string pcd_path;

//Algorithm params for Shot descriptor
double cloud_ss_, descr_rad_ ;
string output_frame;

// Point clouds for object, world and its normals, keypoints and descriptors
PointCloud::Ptr cloud;
PointCloud::Ptr cloud_keypoints;
NormalCloud::Ptr cloud_normals;
DescriptorCloudShot352::Ptr cloud_descriptors_shot352;
DescriptorCloudShot1344::Ptr cloud_descriptors_shot1344;
KeypointMsg::Ptr output_keypoints;
Shot352Msg::Ptr output_descriptors_shot352;
Shot1344Msg::Ptr output_descriptors_shot1344;

pcl::NormalEstimationOMP<PointType, NormalType> norm_est_cloud;
pcl::PointCloud<int> sampled_indices_cloud;
pcl::UniformSampling<PointType> uniform_sampling_cloud;
pcl::SHOTEstimationOMP<PointType, NormalType, SHOT352> descr_est_shot352;
pcl::SHOTColorEstimationOMP<PointType, NormalType, SHOT1344> descr_est_shot1344;
void toROSMsg(const DescriptorCloudShot352 &input, Shot352Msg &output)
{
    output.descriptors.resize(input.size());
    for (int j = 0 ; j < input.size() ; ++j)
    {
        std::copy(input[j].descriptor, input[j].descriptor + 352 , output.descriptors[j].descriptor.begin());
        std::copy(input[j].rf, input[j].rf + 9, output.descriptors[j].rf.begin());
    }
}
void toROSMsg(const DescriptorCloudShot1344 &input, Shot1344Msg &output)
{
    output.descriptors.resize(input.size());
    for (int j = 0 ; j < input.size() ; ++j)
    {
        std::copy(input[j].descriptor, input[j].descriptor + 1344 , output.descriptors[j].descriptor.begin());
        std::copy(input[j].rf, input[j].rf + 9, output.descriptors[j].rf.begin());
    }
}

void pointcloud_incoming (const sensor_msgs::PointCloud2ConstPtr& input)
{
    ros::NodeHandle nh("~");
    //
    // retrieve all parameter variables from server or set to a default value
    //
    nh.param<double>("cloud_ss" , cloud_ss_ , 0.01 );
    nh.param<double>("descr_rad", descr_rad_, 0.02 );
    nh.param<string>("output_frame", output_frame, "pcd_frame");


    //
    // create all neccessary objects
    //
    output_keypoints = KeypointMsg::Ptr (new KeypointMsg);
    output_descriptors_shot352 = Shot352Msg ::Ptr (new Shot352Msg );
    output_descriptors_shot1344= Shot1344Msg::Ptr (new Shot1344Msg);

    cloud               			= PointCloud::Ptr     				(new PointCloud    ());
    cloud_keypoints     			= PointCloud::Ptr     				(new PointCloud    ());
    cloud_normals       			= NormalCloud::Ptr						(new NormalCloud   ());
    cloud_descriptors_shot352 = DescriptorCloudShot352::Ptr	(new DescriptorCloudShot352());
    cloud_descriptors_shot1344= DescriptorCloudShot1344::Ptr(new DescriptorCloudShot1344());

    //
    // convert ROS message to PCL message
    //
    fromROSMsg(*input, *cloud);


    //
    // compute normals
    //
    cout << "... computing normals from cloud ..." << endl;
    norm_est_cloud.setKSearch (10);
    norm_est_cloud.setInputCloud (cloud);
    norm_est_cloud.compute (*cloud_normals);


    //
    //  Downsample cloud to extract keypoints
    //
    cout << "... downsampling cloud ..." << endl;
    uniform_sampling_cloud.setInputCloud (cloud);
    uniform_sampling_cloud.setRadiusSearch (cloud_ss_);
    uniform_sampling_cloud.compute (sampled_indices_cloud);
    pcl::copyPointCloud (*cloud, sampled_indices_cloud.points, *cloud_keypoints);
    std::cout << "Cloud total points: " << cloud->size () << "; Selected Keypoints: " << cloud_keypoints->size () << std::endl;


    //
    // Extract descriptors
    //
    cout << "... extracting descriptors from cloud ..." << endl;
    descr_est_shot352.setInputCloud (cloud_keypoints);
    descr_est_shot352.setRadiusSearch (descr_rad_);
    descr_est_shot352.setInputNormals (cloud_normals);
    descr_est_shot352.setSearchSurface (cloud);
    descr_est_shot352.compute (*cloud_descriptors_shot352);

    descr_est_shot1344.setInputCloud (cloud_keypoints);
    descr_est_shot1344.setRadiusSearch (descr_rad_);
    descr_est_shot1344.setInputNormals (cloud_normals);
    descr_est_shot1344.setSearchSurface (cloud);
    descr_est_shot1344.compute (*cloud_descriptors_shot1344);

    //
    // Convert to ROS message
    //
    pcl::toROSMsg(*cloud_keypoints, *output_keypoints);
    toROSMsg(*cloud_descriptors_shot352, *output_descriptors_shot352);
    toROSMsg(*cloud_descriptors_shot1344, *output_descriptors_shot1344);
    output_keypoints->header.frame_id = output_frame;

    // check if anyone subscribed to keypoints
    if (pub_keypoints.getNumSubscribers() == 0)
    {
        ROS_WARN("No keypoint subscriber found");
        while (pub_keypoints.getNumSubscribers() == 0 && ros::ok())
        {
            ros::Duration(1).sleep();
        }
        ROS_WARN("Subscriber now available, wait another 1 second");
        // give some time to set up connection
        ros::Duration(1).sleep();
    }

    //
    // Publish Keypoints and Descriptors
    //
    pub_keypoints.publish(*output_keypoints);
    pub_descriptors_Shot352.publish(*output_descriptors_shot352);
    pub_descriptors_Shot1344.publish(*output_descriptors_shot1344);
}
