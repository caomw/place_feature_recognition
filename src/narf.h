#ifndef NARF_H
#define NARF_H

#include "features.h"
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

struct narfStruct {
  pcl::PointCloud<pcl::Narf36> narf_descriptors;
  pcl::RangeImage rangeImg;
};

class narfClass : public feature
{
public:
  virtual void extractFeatures()
  {
  	std::cout << "I should be derived base class.";
  }
};

narfStruct narf(const sensor_msgs::PointCloud2::Ptr msg);
void extractPointXYZ(const sensor_msgs::PointCloud2::Ptr);
#endif
