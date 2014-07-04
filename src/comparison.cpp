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

    /*
    ros::Rate myTime(2);
    while(ros::ok)
    {
        myTime.sleep();
        ros::spinOnce();
    }
    */

    ros::spin();

    return 0;

/*
    if(pcl::console::find_argument (argc, argv, "-h") >= 0 || argc < 3){
    printUsage(argv[0]);
    return 0;
    }

    std::string foregroundFileName;
    if(pcl::console::parse (argc, argv, "-f", foregroundFileName) >= 0){
    if(!queryFile(foregroundFileName, argv[0])) return 0;
    }

    std::string backgroundFileName;
    if(pcl::console::parse (argc, argv, "-b", backgroundFileName) >= 0){
    if(!queryFile(backgroundFileName, argv[0])) return 0;
    }

    std::vector<std::string> topics;
    topics.push_back(std::string("/head_xtion/depth/points"));
    topics.push_back(std::string("/head_xtion/rgb/image_color"));

//    topics.push_back(std::string("/camera/rgb/image_color"));
//    topics.push_back(std::string("/camera/depth/points"));

    rosbag::Bag bagFore, bagBack;
    bagFore.open(foregroundFileName, rosbag::bagmode::Read);
    bagBack.open(backgroundFileName, rosbag::bagmode::Read);

    rosbag::View viewB(bagBack, rosbag::TopicQuery(topics));
    rosbag::View viewF(bagFore, rosbag::TopicQuery(topics));

    int imCount= 0;
    int pCount = 0;

    sensor_msgs::Image::Ptr     lastI;
    sensor_msgs::PointCloud2::Ptr   lastP;

    //sensor_msgs::Image::Ptr     i;
    //sensor_msgs::PointCloud2::Ptr   p;

    BOOST_FOREACH(rosbag::MessageInstance const m, viewB){
    try { // Handle instantiation breaks
        sensor_msgs::Image::Ptr     i = m.instantiate<sensor_msgs::Image>();
        sensor_msgs::PointCloud2::Ptr   p = m.instantiate<sensor_msgs::PointCloud2>();

        if(i != NULL)
        {
        lastI = i;
        imCount++;
        cv::waitKey(10);
        }
        if(p != NULL)
        {
        lastP = p;
        pCount++;
        }
        if(imCount == pCount && imCount + pCount > 0)
        {
        pcl::PointCloud<surfDepth> dimensionalSurf = depthSurf(lastI, lastP, 1000);
        std::cout << dimensionalSurf.size() << std::endl;

        cv::waitKey(10);
        }
    }
    catch (const rosbag::BagFormatException& e)
    {
        std::cout << e.what() << "!" << std::endl;
        continue;
    }
    }
    BOOST_FOREACH(rosbag::MessageInstance const m, viewF){
    try { // Handle instantiation breaks
        sensor_msgs::Image::Ptr 		i = m.instantiate<sensor_msgs::Image>();
        if(i != NULL)
        {
        // copy prev code
        }
    }
    catch (const rosbag::BagFormatException& e)
    {
        std::cout << e.what() << "!" << std::endl;
        continue;
    }
    }

    bagFore.close();
    bagBack.close();
    */
}
/*
bool queryFile(std::string fileName, char * progName)
{
    if(fileName.substr(fileName.find_last_of(".") + 1) != "bag")
    {
    ROS_ERROR("Unable to convert none .bag file");
    printUsage(progName);

    return false;
    }
    char realFile[200];
    strcpy(realFile, fileName.c_str());
    std::ifstream my_file(realFile);
    if (!my_file) {
    ROS_ERROR("Cannot Locate %s", realFile);
    printUsage(progName);
    return false;
    }
    return true;
}

void extractDepth(const sensor_msgs::PointCloud2::Ptr p, int x, int y)
{
  //  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  //  pcl::fromROSMsg (*p, point_cloud);

    //std::cout << (float)p->data[0] << ", " << (float)p->data[4] << ", " << (float)p->data[8] << ", " << std::endl;
 //   std::cout << *reinterpret_cast<const float*>(p->data[0]) << " | " << *reinterpret_cast<const float*>(p->data[4]) << " | " << *reinterpret_cast<const float*>(p->data[8]) << std::endl;
    //	cout << *reinterpret_cast<const float*>(p->data[168]) << " | " << *reinterpret_cast<const float*>(p->data[172]) << " | " << *reinterpret_cast<const float*>(p->data[176]) << endl;

    pcl::PointCloud< pcl::PointXYZ > PointCloudXYZ;

    pcl::fromROSMsg(*p,PointCloudXYZ);
    std::cout << "width is " << PointCloudXYZ.width << std::endl;
    std::cout << "height is " << PointCloudXYZ.height << std::endl;
    int cloudsize = (PointCloudXYZ.width) * (PointCloudXYZ.height);
    for (int i=0; i< cloudsize; i++){
    std::cout << "(x,y,z) = " << PointCloudXYZ.points[i] << std::endl;
    }

    PointCloudXYZ.points[0].x;
}

void printUsage(const char* progName)
{
    std::cout << "\nExample: " << progName << "-b scene.bag -f changed.bag \nOptions:\n"
    << "--------------------------------------------------\n"
    << "-b Background bag image\n"
    << "-f foreground bag image \n"
    << "-h This Help\n"
    << "--------------------------------------------------\n\n";
}

*/
