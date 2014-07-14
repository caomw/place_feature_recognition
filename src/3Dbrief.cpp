#include "3Dbrief.h"
#include "3Dbrisk.h"
#include "conversions.h"
#include "visualise.h"
#include <stdlib.h>
#include <math.h>

cv::Mat brief_currentImg, brief_lastImg;
std::vector<cv::KeyPoint> brief_currentKeypoints, brief_lastKeypoints;

pcl::PointCloud<briefDepth> depthBrief(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr &depth)
{
    pcl::PointCloud<briefDepth> depthFeatures;

    cv::Mat image(conversions(msg));
    briefStruct briefObj;

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
    int Thresh = 30;
    int Octave = 3;
    float PatternScales=1.0f;

    // Detect the keypoints using traditional BRISK Detector
    cv::BRISK briskDetector(Thresh, Octave,PatternScales);
    briskDetector.create("Feature2D.BRISK");
    briskDetector.detect(image, briefObj.keypoints);

    cv::BriefDescriptorExtractor extractor;

    extractor.compute(image, briefObj.keypoints, briefObj.descriptors);

    s = briefObj.descriptors.size();
    if(s.height < 1)return depthFeatures;

    // Start Conversion to 3D
    for(int i = 0; i < s.height; i++)
    {
        int x = round(briefObj.keypoints[i].pt.x);
        int y = round(briefObj.keypoints[i].pt.y);

        // only permit featrues where range can be extracted
        if(!isnan(depthPoints.points[depthPoints.width*y+x].x) && !isnan(depthPoints.points[depthPoints.width*y+x].x) && !isnan(depthPoints.points[depthPoints.width*y+x].x))
        {
            briefDepth temp;

            temp.x = depthPoints.points[depthPoints.width*y+x].x;
            temp.y = depthPoints.points[depthPoints.width*y+x].y;
            temp.z = depthPoints.points[depthPoints.width*y+x].z;

            temp.descriptor = briefObj.descriptors.row(i);
            depthFeatures.push_back(temp);
        }
    }

    //SVisualiser.visualise(briefObj.keypoints, image);
    //cv::waitKey(10);

    brief_lastKeypoints = brief_currentKeypoints;
    brief_lastImg = brief_currentImg;
    brief_currentKeypoints = briefObj.keypoints;
    brief_currentImg = image;
    return depthFeatures;
}

pcl::PointCloud<briefDepth> briefDMatch(pcl::PointCloud<briefDepth> a, pcl::PointCloud<briefDepth> b)
{
    pcl::PointCloud<briefDepth> pclMatch;
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

        cv::BFMatcher matcher(cv::NORM_HAMMING);
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

       // std::cout << std::endl;
       // std::cout << " Brisk max dist " << max_dist << std::endl;
       // std::cout << " Brisk mins dist " << min_dist << std::endl;

        sd.SetValues(temp, descriptorsA.rows);

        double mean = sd.CalculateMean();
        double variance = sd.CalculateVariane();
        double samplevariance = sd.CalculateSampleVariane();
        double sampledevi = sd.GetSampleStandardDeviation();
        double devi = sd.GetStandardDeviation();

        std::cout << "Brisk\t" << descriptorsA.rows << "\t"
                << mean << "\t"
                << variance << "\t"
                << samplevariance << "\t"
                << devi << "\t"
                << sampledevi << "\n";

        std::vector< cv::DMatch > good_matches;

        for (int i=0;i<descriptorsA.rows;i++)
        {
            if( matches[i].distance<max_dist/2)
            {
                good_matches.push_back(matches[i]);
                pclMatch.push_back(a[i]);
            }
        }

        cv::Mat img_matches;
        cv::drawMatches( brief_lastImg, brief_lastKeypoints, brief_currentImg, brief_lastKeypoints,
                           good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                           std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        cv::imshow("Brief Matches", img_matches);
        cv::waitKey(50);
    }
    catch (const std::exception &exc)
    {
        // catch anything thrown within try block that derives from std::exception
        std::cerr << exc.what();
    }
    return pclMatch;
}
