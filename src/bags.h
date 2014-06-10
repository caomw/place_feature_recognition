#ifndef BAG_H
#define BAG_H

#include <iostream>
#include <dirent.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/header.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <fstream>
#include <boost/foreach.hpp>
#include <vector>
#include <string>
#include <algorithm>

// Bag Message Types //
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int16.h>
#include <cv_bridge/cv_bridge.h>

#include "surf.h"
#include "narf.h"
#include "shot.h"
#include "visualise.h"

class bagHandler
{
public:
    bagHandler();
    void extract();
    bool visBool;
    bool robotData;
    bool printOutput;
    bool isRobot;
    std::string inputFolder;
    std::string outputBag;
    std::string backgroundFileName;
    surfStruct lastSurf;
    cv::Mat lastFrame;
};

#endif
