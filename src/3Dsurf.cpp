#include "3Dsurf.h"
#include "conversions.h"
#include "visualise.h"
#include <stdlib.h>
#include <math.h>

// globallly initialise visualiser
visualisation SVisualiser("Surf");

cv::Mat currentImg, lastImg;
std::vector<cv::KeyPoint> currentKeypoints, lastKeypoints;

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

    // Detect the keypoints using Star Detector
    int maxSize=16;
    int responseThreshold=30;
    int lineThresholdProjected = 10;
    int lineThresholdBinarized=8;
    int suppressNonmaxSize=5;

    //cv::StarFeatureDetector starDetector(maxSize, responseThreshold, lineThresholdProjected, lineThresholdBinarized, suppressNonmaxSize);
    //starDetector.detect(image, surfObj.keypoints);

    // Assigning stable BRISK constants
    int Thresh = 30;
    int Octave = 3;
    float PatternScales=1.0f;

    //briskDetector.create("Feature2D.SURF");
    //briskDetector.detect(image, surfObj.keypoints);
    //cv::BRISK briskDetector(Thresh, Octave,PatternScales);

    int nfeatures=500;
    float scaleFactor=1.2f;
    int nlevels=8;
    int edgeThreshold=31;
    int firstLevel=0;
    int WTA_K=2;
    int scoreType=cv::ORB::HARRIS_SCORE;
    int patchSize=31;

    cv::ORB orbDetector(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize);
    orbDetector.create("Feature2D.ORB");
    orbDetector.detect(image, surfObj.keypoints);

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
    //SURF(double hessianThreshold, int nOctaves=4, int nOctaveLayers=2, bool extended=true, bool upright=false )
    cv::SurfFeatureDetector detector(hessian, 4, 2, true, true);

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

            temp.descriptor = surfObj.descriptors.row(i);
            depthFeatures.push_back(temp);
        }
    }

    //SVisualiser.visualise(surfObj.keypoints, image);
    //cv::waitKey(10);

    lastKeypoints = currentKeypoints;
    lastImg = currentImg;
    currentKeypoints = surfObj.keypoints;
    currentImg = image;
    return depthFeatures;
}

pcl::PointCloud<surfDepth> SDMatch(pcl::PointCloud<surfDepth> a, pcl::PointCloud<surfDepth> b)
{
    pcl::PointCloud<surfDepth> pclMatch;
    try
    {
        cv::Mat descriptorsA;
        cv::Mat descriptorsB;
        for(int i =0; i < a.size(); i++)
        {
            descriptorsA.push_back(a[i].descriptor);
        }

        for(int i =0; i < b.size(); i++)
        {
            descriptorsB.push_back(b[i].descriptor);
        }

        std::vector<std::vector <cv::DMatch> > matches;
        cv::BFMatcher matcher;
        matcher.knnMatch(descriptorsA, descriptorsB, matches, 2);
        std::vector<cv::DMatch> good_matches;
        for (int i = 0; i < matches.size(); ++i)
        {
            const float ratio = 0.02; // As in Lowe's paper;
            if (matches[i][0].distance < ratio )
            {
                std::cout << matches[i][0].distance << "\t";
                good_matches.push_back(matches[i][0]);
            }
        }

        std::cout << "\n\n"<<
                     matches[0][0].distance << "\t" << matches[0][1].distance << std::endl <<
                     matches[1][0].distance << "\t" << matches[1][1].distance << std::endl <<
                     matches[2][0].distance << "\t" << matches[2][1].distance << std::endl << std::endl;
        // find 3 best matching
    /*    cv::FlannBasedMatcher matcher;
        std::vector< cv::DMatch > matches;

        matcher.match( descriptorsA, descriptorsB, matches );

        double max_dist = 0; double min_dist = 1000;

        StdDeviation sd;
        double temp[descriptorsA.rows];


        for (int i =0; i < descriptorsA.rows;i++)
        {
            double dist = matches[i].distance;
            if(max_dist<dist) max_dist = dist;
            if(min_dist>dist) min_dist = dist;
            //std::cout << dist << "\t";
            temp[i] = dist;
        }

        //std::cout << "\n" << std::endl;
        //std::cout << " Surf max dist " << max_dist << std::endl;
        //std::cout << " Surf min dist " << min_dist << std::endl;

        sd.SetValues(temp, descriptorsA.rows);

        double mean = sd.CalculateMean();
        double variance = sd.CalculateVariane();
        double samplevariance = sd.CalculateSampleVariane();
        double sampledevi = sd.GetSampleStandardDeviation();
        double devi = sd.GetStandardDeviation();

        std::cout << "Surf\t" << descriptorsA.rows << "\t"
                << mean << "\t"
                << variance << "\t"
                << samplevariance << "\t"
                << devi << "\t"
                << sampledevi << "\n";

        std::vector< cv::DMatch > good_matches;

        for (int i=0;i<descriptorsA.rows;i++)
        {
            //if( matches[i].distance < max_dist / 2)
            if( matches[i].distance < 0.03)
            {
                good_matches.push_back(matches[i]);
                pclMatch.push_back(a[i]);
            }
        }
        */
        cv::Mat img_matches;
        cv::drawMatches( lastImg, lastKeypoints, currentImg, lastKeypoints,
                           good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                           std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        cv::imshow("Surf Matches", img_matches);
        cv::waitKey(50);
        //std::cout << good_matches.size() << " Surf features matched from, " << a.size() << ", " << b.size() << " sets." << std::endl;

    }
    catch (const std::exception &exc)
    {
        // catch anything thrown within try block that derives from std::exception
        std::cerr << exc.what();
    }
    return pclMatch;
}


void myicp(pcl::PointCloud<surfDepth> a, pcl::PointCloud<surfDepth> b)
{
    /*
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
*/
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
