#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <pcl/console/parse.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <fstream>
#include "visualise.h"

#include "3Dsurf.h"
#include "3Dsift.h"
#include "3Dbrisk.h"
#include "3Dbrief.h"
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
pcl::PointCloud<siftDepth>  lastSift;
pcl::PointCloud<briskDepth> lastBrisk;
pcl::PointCloud<briefDepth> lastBrief;

void callback(const sensor_msgs::ImageConstPtr &image,  const sensor_msgs::PointCloud2ConstPtr &depth,
              ros::NodeHandle *nh, ros::Publisher *spub, ros::Publisher *bpub, ros::Publisher *cpub)
{
    // Advertise topics
    *cpub = nh->advertise<pcl::PointCloud<surfDepth> > ("/cloud",1);
    *spub = nh->advertise<pcl::PointCloud<surfDepth> > ("/camera/depth/surf", 1);
    *bpub = nh->advertise<pcl::PointCloud<surfDepth> > ("/camera/depth/brisk",1);

    // Get 3D Features
    GPUSurf(image, depth, 400);
    pcl::PointCloud<briskDepth> dimensionalBrisk =  depthBrisk(image, depth);

    pcl::PointCloud<surfDepth>  dimensionalSurf;// =   depthSurf(image, depth, 400);
    pcl::PointCloud<siftDepth>  dimensionalSift =   depthSift(image, depth, 400);
    pcl::PointCloud<briefDepth> dimensionalBrief;// =  depthBrief(image, depth);

    pcl::PointCloud<surfDepth> tempSurf;
    pcl::PointCloud<briskDepth> tempBrisk;

    // Configure message headers
    ros::Time time_st = ros::Time::now();
    dimensionalSurf.header.stamp = time_st.toNSec()/1e3;
    dimensionalSurf.header.frame_id  = depth->header.frame_id ;

    dimensionalBrisk.header.stamp = time_st.toNSec()/1e3;
    dimensionalBrisk.header.frame_id  = depth->header.frame_id ;

    // Publish custom Surf feature message
    //spub->publish (dimensionalSurf);
    //bpub->publish (dimensionalBrisk);

    // print found depth extractible features
    //std::cout << dimensionalSurf.size() << " Surf features,\t" << dimensionalBrisk.size() << " Brisk features.\n";
    //std::cout << "\e[A";

    // atempt compare


    if(lastBrisk.size() > 0 && dimensionalBrisk.size() > 0)
    {
        // get Matches
        tempBrisk = BDMatch(lastBrisk, dimensionalBrisk);
        siftDMatch(lastSift, dimensionalSift);
        //tempSurf = SDMatch(lastSurf, dimensionalSurf);
     /*   SDMatch(lastSurf, dimensionalSurf);

        briefDMatch(lastBrief, dimensionalBrief);
       */ //myicp(lastSurf, dimensionalSurf);
    }
    else
    {
        std::cout << "First loop " << dimensionalSurf.size() << " Surf, " << dimensionalBrisk.size() << " Brisk features.\n";
    }
    tempBrisk.header.stamp = time_st.toNSec()/1e3;
    tempBrisk.header.frame_id  = depth->header.frame_id ;

    //tempSurf.header.stamp = time_st.toNSec()/1e3;
    //tempSurf.header.frame_id  = depth->header.frame_id ;

    bpub->publish(tempBrisk);
    //spub->publish(tempSurf);
    cpub->publish(depth);

    lastSurf = dimensionalSurf;
    lastSift = dimensionalSift;
    lastBrisk = dimensionalBrisk;
    lastBrief = dimensionalBrief;
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "hole_detection");
    std::cout << "Running\n";
    ros::Publisher cloud;
    ros::Publisher surfPub;
    ros::Publisher briskPub;

    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_color", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub(nh, "/camera/depth/points", 1);

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), image_sub, depth_sub);
    sync.registerCallback(boost::bind(callback, _1, _2, &nh, &briskPub, &surfPub, &cloud));

    ros::spin();

    return 0;
}
