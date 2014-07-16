#include "3Dbrisk.h"
#include "conversions.h"
#include "visualise.h"
#include <stdlib.h>
#include <math.h>

visualisation BVisualiser("Brisk");
int count = 0;
cv::Mat brisk_currentImg, brisk_lastImg;
std::vector<cv::KeyPoint> brisk_currentKeypoints, brisk_lastKeypoints;

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
    int Thresh = 30;
    int Octave = 3;
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

            temp.descriptor = briskObj.descriptors.row(i);
            depthFeatures.push_back(temp);
        }
    }

    BVisualiser.visualise(briskObj.keypoints, image);
    cv::waitKey(10);

    if(count == 0)
    {
        //brisk_lastKeypoints = brisk_currentKeypoints;
        //brisk_lastImg = brisk_currentImg;


        brisk_lastKeypoints = briskObj.keypoints;
        brisk_lastImg = image;
    }
    brisk_currentKeypoints = briskObj.keypoints;
    brisk_currentImg = image;
    count++;
    return depthFeatures;
}

pcl::PointCloud<briskDepth> BDMatch(pcl::PointCloud<briskDepth> a, pcl::PointCloud<briskDepth> b)
{
    std::cout << "The count is: " << count << std::endl;
    pcl::PointCloud<briskDepth> pclMatch;
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

        //std::cout << std::endl;
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
            //if( matches[i].distance<10)
            if( matches[i].distance<max_dist/2)
            {
                good_matches.push_back(matches[i]);
                pclMatch.push_back(a[i]);
            }
        }

        cv::Mat img_matches;
        cv::drawMatches( brisk_lastImg, brisk_lastKeypoints, brisk_currentImg, brisk_lastKeypoints,
                           good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                           std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

       // cv::imshow("Brisk Matches", img_matches);
        std::vector<cv::Point2f> obj;
        std::vector<cv::Point2f> scene;

        for( int i = 0; i < good_matches.size(); i++ )
        {
          //-- Get the keypoints from the good matches
          obj.push_back( brisk_lastKeypoints[ good_matches[i].queryIdx ].pt );
          scene.push_back( brisk_currentKeypoints[ good_matches[i].trainIdx ].pt );
        }

        cv::Mat H = findHomography( obj, scene, CV_RANSAC );

        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<cv::Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( brisk_lastImg.cols, 0 );
        obj_corners[2] = cvPoint( brisk_lastImg.cols, brisk_lastImg.rows ); obj_corners[3] = cvPoint( 0, brisk_lastImg.rows );
        std::vector<cv::Point2f> scene_corners(4);

        perspectiveTransform( obj_corners, scene_corners, H);

        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        line( img_matches, scene_corners[0] + cv::Point2f( brisk_lastImg.cols, 0), scene_corners[1] + cv::Point2f( brisk_lastImg.cols, 0), cv::Scalar(0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + cv::Point2f( brisk_lastImg.cols, 0), scene_corners[2] + cv::Point2f( brisk_lastImg.cols, 0), cv::Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + cv::Point2f( brisk_lastImg.cols, 0), scene_corners[3] + cv::Point2f( brisk_lastImg.cols, 0), cv::Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + cv::Point2f( brisk_lastImg.cols, 0), scene_corners[0] + cv::Point2f( brisk_lastImg.cols, 0), cv::Scalar( 0, 255, 0), 4 );

        //-- Show detected matches
        cv::imshow( "Good brisk Matches & Object detection", img_matches );
        cv::waitKey(50);
       // std::cout << good_matches.size() << " Brisk features matched from, " << a.size() << ", " << b.size() << " sets." << std::endl;
    }
    catch (const std::exception &exc)
    {
        // catch anything thrown within try block that derives from std::exception
        std::cerr << exc.what();
    }
    return pclMatch;
}
