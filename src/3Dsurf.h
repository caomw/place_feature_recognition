#ifndef DEPTHSURF_H
#define DEPTHSURF_H

#include "features.h"
#include "conversions.h"
#include "opencv2/core/core.hpp"
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <std_msgs/Int16.h>
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

//////////////////////
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
/////////////////////////
struct surfStruct {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat     descriptors;
};

struct surfDepth
{
    PCL_ADD_POINT4D                  // import logical XYZ + padding
    cv::Mat descriptor;            // if cv::mat->type enums to 5 float (32f)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (surfDepth,           // here we assume a XYZ + "descriptor" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, descriptor, descriptor)
);

// define how to output results
inline std::ostream& operator << (std::ostream& os, const surfDepth& p)
{
    // to do
   /*os << p.x<<","<<p.y<<","<<p.z<<" - "<<p.roll*360.0/M_PI<<"deg,"<<p.pitch*360.0/M_PI<<"deg,"<<p.yaw*360.0/M_PI<<"deg - ";
   for (int i = 0; i < 36; ++i)
     os << (i == 0 ? "(" : "") << p.descriptor[i] << (i < 35 ? ", " : ")");
   return (os);
   */
    //http://docs.pointclouds.org/1.0.0/point__types_8hpp_source.html  << for moar
}

pcl::PointCloud<surfDepth> GPUSurf(const sensor_msgs::ImageConstPtr&, const sensor_msgs::PointCloud2ConstPtr&, int);
pcl::PointCloud<surfDepth> depthSurf(const sensor_msgs::ImageConstPtr&, const sensor_msgs::PointCloud2ConstPtr&, int);
void SDMatch(pcl::PointCloud<surfDepth>, pcl::PointCloud<surfDepth>);
void myicp(pcl::PointCloud<surfDepth>, pcl::PointCloud<surfDepth>);
void ransac(pcl::PointCloud<surfDepth>, pcl::PointCloud<surfDepth>);
//pcl::PointCloud<surfDepth> depthSurf(const sensor_msgs::ImageConstPtr&, const sensor_msgs::ImageConstPtr&, int);


#endif
