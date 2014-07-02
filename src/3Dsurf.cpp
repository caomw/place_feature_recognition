#include "3Dsurf.h"
#include "conversions.h"
#include "visualise.h"
#include <stdlib.h>
#include <math.h>

// globallly initialise visualiser
visualisation SVisualiser("Surf");

#include "opencv2/gpu/gpu.hpp"
pcl::PointCloud<surfDepth> GPUSurf(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr &depth, int hessian)
{
    pcl::PointCloud<surfDepth> depthFeatures;

    cv::Mat image(conversions(msg));
    surfStruct surfObj;

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

    // Detect the keypoints using SURF Detector
    cv::SurfFeatureDetector detector(hessian);

    detector.detect(image, surfObj.keypoints);

    // Calculate descriptors (feature vectors)
    cv::SurfDescriptorExtractor extractor;
    extractor.compute( image, surfObj.keypoints, surfObj.descriptors);

    s = surfObj.descriptors.size();
    if(s.height < 1)return depthFeatures;

    return depthFeatures;
}

pcl::PointCloud<surfDepth> depthSurf(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr &depth, int hessian)
{
    pcl::PointCloud<surfDepth> depthFeatures;

    cv::Mat image(conversions(msg));
    surfStruct surfObj;

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

    // Detect the keypoints using SURF Detector
    cv::SurfFeatureDetector detector(hessian);

    detector.detect(image, surfObj.keypoints);

    // Calculate descriptors (feature vectors)
    cv::SurfDescriptorExtractor extractor;
    extractor.compute( image, surfObj.keypoints, surfObj.descriptors);

    s = surfObj.descriptors.size();
    if(s.height < 1)return depthFeatures;


    // Start Conversion to 3D
    for(int i = 0; i < s.height; i++)
    {
        int x = round(surfObj.keypoints[i].pt.x);
        int y = round(surfObj.keypoints[i].pt.y);

        // only permit featrues where range can be extracted
        if(!isnan(depthPoints.points[depthPoints.width*y+x].x) && !isnan(depthPoints.points[depthPoints.width*y+x].x) && !isnan(depthPoints.points[depthPoints.width*y+x].x))
        {
            surfDepth temp;

            temp.x = depthPoints.points[depthPoints.width*y+x].x;
            temp.y = depthPoints.points[depthPoints.width*y+x].y;
            temp.z = depthPoints.points[depthPoints.width*y+x].z;

            for(int j = 0; j < 64; j ++){
                temp.descriptor[j] = surfObj.descriptors.at<float>(i,j);
            }
            depthFeatures.push_back(temp);
        }
    }

    SVisualiser.visualise(surfObj.keypoints, image);
    cv::waitKey(10);
    return depthFeatures;
}

void matcher(pcl::PointCloud<surfDepth> a, pcl::PointCloud<surfDepth> b)
{

    try
    {
        cv::Mat descriptorsA(a.size(), 64, CV_8UC1 );
        cv::Mat descriptorsB(b.size(), 64, CV_8UC1 );

        for(int i =0; i < a.size(); i++)
        {
            cv::Mat tempRow = cv::Mat(1, 64, CV_8UC1 , &a[i].descriptor);
            descriptorsA.push_back(cv::Mat(tempRow));
        }

        for(int i =0; i < b.size(); i++)
        {
            cv::Mat tempRow = cv::Mat(1, 64, CV_8UC1 , &b[i].descriptor);
            descriptorsB.push_back(cv::Mat(tempRow));
        }

        cv::FlannBasedMatcher flanMatch;
        std::vector< cv::DMatch > matches;
        flanMatch.match( descriptorsA, descriptorsB, matches );

        double max_dist = 0; double min_dist = 0.9;

          //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < descriptorsA.rows; i++ )
        {
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }

        printf("-- Max dist : %f \n", max_dist );
        printf("-- Min dist : %f \n", min_dist );

        std::vector< cv::DMatch > good_matches;

        for( int i = 0; i < descriptorsA.rows; i++ )
        { if( matches[i].distance < 2*min_dist )
            {
                good_matches.push_back( matches[i]);
            }
        }
    }
    catch (const std::exception &exc)
    {
        // catch anything thrown within try block that derives from std::exception
        std::cerr << exc.what();
    }
}


void myicp(pcl::PointCloud<surfDepth> a, pcl::PointCloud<surfDepth> b)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

    for(int i =0; i < a.size(); i++)
    {
        pcl::PointXYZ temp;
        temp.x = a[i].x;
        temp.y = a[i].y;
        temp.z = a[i].z;
        cloud_in->push_back(temp);
    }

    for(int i =0; i < b.size(); i++)
    {
        pcl::PointXYZ temp;
        temp.x = b[i].x;
        temp.y = b[i].y;
        temp.z = b[i].z;
        cloud_out->push_back(temp);
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

}

void ransac(pcl::PointCloud<surfDepth> a, pcl::PointCloud<surfDepth> b)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB;
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
    for(int i =0; i < a.size(); i++)
    {
        pcl::PointXYZ temp;
        temp.x = a[i].x;
        temp.y = a[i].y;
        temp.z = a[i].z;
        cloudA->push_back(temp);
    }

    for(int i =0; i < b.size(); i++)
    {
        pcl::PointXYZ temp;
        temp.x = b[i].x;
        temp.y = b[i].y;
        temp.z = b[i].z;
        cloudB->push_back(temp);
    }

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr pA (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloudA));
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr pB (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloudB));
}
