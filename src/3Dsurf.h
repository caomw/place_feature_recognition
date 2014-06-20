#ifndef DEPTHSURF_H
#define DEPTHSURF_H

#include "features.h"
#include "opencv2/core/core.hpp"
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <std_msgs/Int16.h>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cvwimage.h>
#include <sensor_msgs/Image.h>

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"

#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

struct surfStruct {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat     descriptors;
};

struct myDescriptor
{
    PCL_ADD_POINT4D                  // import logical XYZ + padding
    float descriptor[64];            // if cv::mat->type enums to 5 float (32f)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (myDescriptor,           // here we assume a XYZ + "descriptor" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, descriptor, descriptor)
);



// define how to output results
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

sensor_msgs::Image conversions(cv::Mat mat);
cv::Mat conversions(const sensor_msgs::ImageConstPtr&);

pcl::PointCloud<myDescriptor> depthSurf(const sensor_msgs::ImageConstPtr&, const sensor_msgs::PointCloud2::Ptr, int);

#endif
