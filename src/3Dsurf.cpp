#include "3Dsurf.h"
#include "visualise.h"
#include <stdlib.h>
#include <math.h>

sensor_msgs::Image conversions(cv::Mat mat)
{
    sensor_msgs::Image sensorImg;

    sensorImg.height = mat.rows;
    sensorImg.width = mat.cols;
    sensorImg.encoding = sensor_msgs::image_encodings::BGR8;
    sensorImg.is_bigendian = false;
    sensorImg.step = mat.step;
    size_t size = mat.step * mat.rows;
    sensorImg.data.resize(size);
    memcpy((char*)(&sensorImg.data[0]), mat.data, size);

    return sensorImg;
}
visualisation temp;
cv::Mat conversions(const sensor_msgs::ImageConstPtr& image)
{
    cv::Mat x;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        if(image->encoding == "bgr8" ){
            cv_ptr = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::BGR8);
        }
        if(image->encoding == "32FC1"){
            cv_ptr = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::TYPE_32FC1);
        }
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception returning Null image: %s",  e.what());
        return x;
    }
    return cv_ptr->image;
}

pcl::PointCloud<myDescriptor> depthSurf(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr &depth, int hessian)
{
    // Convert sensor message
    pcl::PointCloud< pcl::PointXYZ > depthPoints;
    pcl::fromROSMsg(*depth,depthPoints);


/*
    std::cout << "\n______________\n";
    for(int y = 0; y < 5; y++)
    {
        for(int x = 0; x < 640; x ++)
        {


        }
    }
*/
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

    s = surfObj.descriptors.size();
    if(s.height < 1)return depthFeatures;


            //(std::vector<cv::KeyPoint> features,cv::Mat image)

    //-- Step 3: Start Conversion to 3D
    /*for(int i = 0 ; i < depthPoints.height * depthPoints.width; i++)
    {
        myDescriptor temp;
        temp.x = depthPoints.points[i].x;
        temp.y = depthPoints.points[i].y;
        temp.z = depthPoints.points[i].z;
        depthFeatures.push_back(temp);
    }*/


    for(int i = 0; i < s.height; i++)
    {
        int x = round(surfObj.keypoints[i].pt.x);
        int y = round(surfObj.keypoints[i].pt.y);

        if(!isnan(depthPoints.points[640*y+x].x) && !isnan(depthPoints.points[640*y+x].x) && !isnan(depthPoints.points[640*y+x].x)){

        myDescriptor temp;

        temp.x = depthPoints.points[640*y+x].x;
        temp.y = depthPoints.points[640*y+x].y;
        temp.z = depthPoints.points[640*y+x].z;

        for(int j = 0; j < 64; j ++){
            temp.descriptor[j] = surfObj.descriptors.at<float>(i,j);
        }
        depthFeatures.push_back(temp);
    }
    }

    temp.visualise(surfObj.keypoints, image);
    cv::waitKey(10);
    return depthFeatures;
}
