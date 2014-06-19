#include <iostream>
#include <math.h>
#include <pcl/console/parse.h>
#include <ros/ros.h>
#include <fstream>
#include "surf.h"
#include "narf.h"
#include "visualise.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

void printUsage(const char*);
bool queryFile(std::string, char*);
void extractDepth(sensor_msgs::PointCloud2::Ptr, int, int);

struct myDescriptor
{
    float x, y, z;
    float descriptor[64]; // if cv::mat->type enums to 5 float (32f)
};
// in line to follow convension // ?
inline std::ostream& operator << (std::ostream& os, const myDescriptor& p)
{
    // to do
   /*os << p.x<<","<<p.y<<","<<p.z<<" - "<<p.roll*360.0/M_PI<<"deg,"<<p.pitch*360.0/M_PI<<"deg,"<<p.yaw*360.0/M_PI<<"deg - ";
   for (int i = 0; i < 36; ++i)
     os << (i == 0 ? "(" : "") << p.descriptor[i] << (i < 35 ? ", " : ")");
   return (os);
   */
    //http://docs.pointclouds.org/1.0.0/point__types_8hpp_source.html  << for moar
}

pcl::PointCloud<myDescriptor> depthSurf(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2::Ptr p, int hessian)
{
    pcl::PointCloud<myDescriptor> depthFeatures;

    cv::Mat image(conversions(msg));
    surfStruct surfObj;

    if(msg->encoding != "bgr8" ){
        ROS_ERROR("Unsupported image encoding:");
        return depthFeatures;  // Return Null image
    }

    cv::Size s = image.size();
    if(s.height < 1)return depthFeatures;

    cv::SurfFeatureDetector detector(hessian);

    detector.detect(image, surfObj.keypoints);

    //-- Step 2: Calculate descriptors (feature vectors)
    cv::SurfDescriptorExtractor extractor;
    extractor.compute( image, surfObj.keypoints, surfObj.descriptors);
    std::cout << "Surf:" <<surfObj.descriptors.size() << "\n";

    s = surfObj.descriptors.size();
    if(s.height < 1)return depthFeatures;

    //-- Step 3: Start Conversion to 3D
    pcl::PointCloud< pcl::PointXYZ > PointCloudXYZ;
    pcl::fromROSMsg(*p,PointCloudXYZ);

    for(int i = 0; i < s.height; i++)
    {
        myDescriptor temp;

        temp.x = surfObj.keypoints[i].pt.x;
        temp.y = surfObj.keypoints[i].pt.y;
        temp.z = PointCloudXYZ.points[round(surfObj.keypoints[i].pt.y)*PointCloudXYZ.height + round(surfObj.keypoints[i].pt.x)].z;
        for(int j = 0; j < 64; j ++){
            temp.descriptor[j] = surfObj.descriptors.at<float>(i,j);
        }
        depthFeatures.push_back(temp);
    }


    std::cout << surfObj.descriptors.size() << "\t hello";
    return depthFeatures;
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "bags");

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

    std::vector <surfStruct> foreFeatures;
    std::vector <surfStruct> backFeatures;

    pcl::PointCloud<myDescriptor> worked;

    int imCount= 0;
    int pCount = 0;

    sensor_msgs::Image::Ptr         lastI;
    sensor_msgs::PointCloud2::Ptr   lastP;

    //sensor_msgs::Image::Ptr         i;
    //sensor_msgs::PointCloud2::Ptr   p;
    visualisation visualiser;

    BOOST_FOREACH(rosbag::MessageInstance const m, viewB){
        try { // Handle instantiation breaks
            sensor_msgs::Image::Ptr         i = m.instantiate<sensor_msgs::Image>();
            sensor_msgs::PointCloud2::Ptr   p = m.instantiate<sensor_msgs::PointCloud2>();

            if(i != NULL)
            {
                lastI = i;
                foreFeatures.push_back(surf(i, 1000));   // get Surf Features
                imCount++;
                visualiser.visualise(foreFeatures[0].keypoints, conversions(i));
                cv::waitKey(10);
            }
            if(p != NULL)
            {
                lastP = p;
                pCount++;
            }
            if(imCount == pCount && imCount + pCount > 0)
            {
                pcl::PointCloud<myDescriptor> dimensionalSurf = depthSurf(lastI,lastP,1000);
                std::cout << dimensionalSurf.size() << std::endl;

              //  extractDepth(lastP, 0, 0);
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
                backFeatures.push_back(surf(i, 1000));   // get Surf Features
                std::cout << backFeatures[0].descriptors.type() << "\t\n";
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
}

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
  /*  pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg (*p, point_cloud);

    std::cout << (float)p->data[0] << ", " << (float)p->data[4] << ", " << (float)p->data[8] << ", " << std::endl;
 //   std::cout << *reinterpret_cast<const float*>(p->data[0]) << " | " << *reinterpret_cast<const float*>(p->data[4]) << " | " << *reinterpret_cast<const float*>(p->data[8]) << std::endl;
    //	cout << *reinterpret_cast<const float*>(p->data[168]) << " | " << *reinterpret_cast<const float*>(p->data[172]) << " | " << *reinterpret_cast<const float*>(p->data[176]) << endl;
    */
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
