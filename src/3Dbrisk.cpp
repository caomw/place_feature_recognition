#include "3Dbrisk.h"
#include "conversions.h"
#include "visualise.h"
#include <stdlib.h>
#include <math.h>

visualisation BVisualiser("Brisk");

pcl::PointCloud<briskDepth> depthBrisk(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr &depth)
{
    pcl::PointCloud<briskDepth> depthFeatures;

    cv::Mat image(conversions(msg));
    briskStruct briskObj;

    if(msg->encoding != "bgr8" )
    {
        ROS_ERROR("Unsupported image encoding:");
        return depthFeatures;  // Return Null image
    }

    // Check image size big enough
    cv::Size s = image.size();
    if(s.height < 1)return depthFeatures;

    // Convert sensor message
    pcl::PointCloud< pcl::PointXYZ > depthPoints;
    pcl::fromROSMsg(*depth,depthPoints);

    // Assigning stable BRISK constants
    int Thresh = 60;
    int Octave = 4;
    float PatternScales=1.0f;

    // Detect the keypoints using traditional BRISK Detector
    cv::BRISK briskDetector(Thresh, Octave,PatternScales);
    briskDetector.create("Feature2D.BRISK");
    briskDetector.detect(image, briskObj.keypoints);

    // Extract Features
    briskDetector.compute(image, briskObj.keypoints, briskObj.descriptors);


    s = briskObj.descriptors.size();
    if(s.height < 1)return depthFeatures;


    // Start Conversion to 3D
    for(int i = 0; i < s.height; i++)
    {
        int x = round(briskObj.keypoints[i].pt.x);
        int y = round(briskObj.keypoints[i].pt.y);

        // only permit featrues where range can be extracted
        if(!isnan(depthPoints.points[depthPoints.width*y+x].x) && !isnan(depthPoints.points[depthPoints.width*y+x].x) && !isnan(depthPoints.points[depthPoints.width*y+x].x))
        {
            briskDepth temp;

            temp.x = depthPoints.points[depthPoints.width*y+x].x;
            temp.y = depthPoints.points[depthPoints.width*y+x].y;
            temp.z = depthPoints.points[depthPoints.width*y+x].z;

            for(int j = 0; j < 60; j ++){
                temp.descriptor[j] = briskObj.descriptors.at<float>(i,j);
            }
            depthFeatures.push_back(temp);
        }
    }

    BVisualiser.visualise(briskObj.keypoints, image);
    cv::waitKey(10);

    return depthFeatures;
}
