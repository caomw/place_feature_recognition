#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <pcl/console/parse.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <fstream>
#include "visualise.h"

#include "3Dsurf.h"
#include "3Dbrisk.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>


using namespace std;
using namespace message_filters;

pcl::PointCloud<surfDepth>  lastSurf;
pcl::PointCloud<briskDepth> lastBrisk;

void callback(const sensor_msgs::ImageConstPtr &image,  const sensor_msgs::PointCloud2ConstPtr &depth,
              ros::NodeHandle *nh, ros::Publisher *spub, ros::Publisher *bpub)
{
    // Advertise topics
    *spub = nh->advertise<pcl::PointCloud<surfDepth> > ("/camera/depth/surf", 1);
    *bpub = nh->advertise<pcl::PointCloud<surfDepth> > ("/camera/depth/brisk",1);

    // Get 3D Features
    GPUSurf(image, depth, 400);
    pcl::PointCloud<surfDepth> dimensionalSurf = depthSurf(image, depth, 400);
    pcl::PointCloud<briskDepth> dimensionalBrisk = depthBrisk(image, depth);

    // Configure message headers
    ros::Time time_st = ros::Time::now();
    dimensionalSurf.header.stamp = time_st.toNSec()/1e3;
    dimensionalSurf.header.frame_id  = depth->header.frame_id ;

    dimensionalBrisk.header.stamp = time_st.toNSec()/1e3;
    dimensionalBrisk.header.frame_id  = depth->header.frame_id ;

    // Publish custom Surf feature message
    spub->publish (dimensionalSurf);
    bpub->publish (dimensionalBrisk);

    // print found depth extractible features
    //std::cout << dimensionalSurf.size() << " Surf features,\t" << dimensionalBrisk.size() << " Brisk features.\n";
    //std::cout << "\e[A";

    // atempt compare
    if(lastSurf.size() > 0 && dimensionalSurf.size() > 0)
    {
        //std::cout << lastSurf.size() << "\t" <<  dimensionalSurf.size() << std::endl;
        SDMatch(lastSurf, dimensionalSurf);
        BDMatch(lastBrisk, dimensionalBrisk);
        //myicp(lastSurf, dimensionalSurf);
    }else
    {
        std::cout << "First loop " << dimensionalSurf.size() << " Surf, " << dimensionalBrisk.size() << " Brisk features.\n";
    }
    lastSurf = dimensionalSurf;
    lastBrisk = dimensionalBrisk;

}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "hole_detection");
    std::cout << "Running\n";
    ros::Publisher surfPub;
    ros::Publisher briskPub;

    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_color", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub(nh, "/camera/depth/points", 1);

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), image_sub, depth_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, &nh, &briskPub, &surfPub));

    ros::spin();

    return 0;

}