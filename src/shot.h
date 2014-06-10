#ifndef SHOT_H
#define SHOT_H

#include "features.h"
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <object_recognition/Shot352.h>
#include <object_recognition/Shot352_bundle.h>
#include <object_recognition/Shot1344.h>
#include <object_recognition/Shot1344_bundle.h>
#include <stdio.h>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>



void pointcloud_incoming (const sensor_msgs::PointCloud2ConstPtr&);

#endif
