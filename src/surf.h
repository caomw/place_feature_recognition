 #ifndef SURF_H
#define SURF_H

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

struct surfStruct {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat     descriptors;
};

struct surfWords {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat     rawDescriptors;
    cv::Mat     filteredDescriptors;
    std::string name;
};


sensor_msgs::Image conversions(cv::Mat mat);
cv::Mat conversions(const sensor_msgs::ImageConstPtr&);

class surfClass : public feature
{
public:
    virtual void extractFeatures()
    {
        std::cout << "I should be derived base class.";
    }
};

surfStruct surf(const sensor_msgs::ImageConstPtr&, int);
cv::Mat surfConverter();

void DMatch(surfStruct, cv::Mat, surfStruct, cv::Mat);
cv::Mat kmeans(cv::Mat);
#endif
