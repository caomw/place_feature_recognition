#include "3Dsurf.h"

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

cv::Mat conversions(const sensor_msgs::ImageConstPtr& image)
{
    cv::Mat x;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::BGR8);
    }

   catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception returning Null image: %s",  e.what());
        return x;
    }
    return cv_ptr->image;
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
