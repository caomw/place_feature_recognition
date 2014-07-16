#include "3Dsift.h"
#include "3Dbrisk.h"
#include "conversions.h"
#include "visualise.h"
#include <stdlib.h>
#include <math.h>

using namespace cv;

cv::Mat sift_currentImg, sift_lastImg;
std::vector<cv::KeyPoint> sift_currentKeypoints, sift_lastKeypoints;
int scount = 0;
pcl::PointCloud<siftDepth> depthSift(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr &depth, int hessian)
{
    pcl::PointCloud<siftDepth> depthFeatures;

    cv::Mat image(conversions(msg));
    siftStruct siftObj;

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
    briskDetector.create("Feature2D.SIFT");
    briskDetector.detect(image, siftObj.keypoints);

    // Calculate descriptors (feature vectors)
    cv::SiftDescriptorExtractor extractor;
    extractor.compute( image, siftObj.keypoints, siftObj.descriptors);

    s = siftObj.descriptors.size();
    if(s.height < 1)return depthFeatures;


    // Start Conversion to 3D
    for(int i = 0; i < s.height; i++)
    {
        int x = round(siftObj.keypoints[i].pt.x);
        int y = round(siftObj.keypoints[i].pt.y);

        // only permit featrues where range can be extracted
        if(!isnan(depthPoints.points[depthPoints.width*y+x].x) && !isnan(depthPoints.points[depthPoints.width*y+x].x) && !isnan(depthPoints.points[depthPoints.width*y+x].x))
        {
            siftDepth temp;

            temp.x = depthPoints.points[depthPoints.width*y+x].x;
            temp.y = depthPoints.points[depthPoints.width*y+x].y;
            temp.z = depthPoints.points[depthPoints.width*y+x].z;

            temp.descriptor = siftObj.descriptors.row(i);
            depthFeatures.push_back(temp);
        }
    }

    //SVisualiser.visualise(siftObj.keypoints, image);
    //cv::waitKey(10);
    if(scount == 0)
    {
    //sift_lastKeypoints = sift_currentKeypoints;
    //sift_lastImg = sift_currentImg;
        sift_lastKeypoints = siftObj.keypoints;
        sift_lastImg = image;
    }
    sift_currentKeypoints = siftObj.keypoints;
    sift_currentImg = image;
    scount++;
    return depthFeatures;
}

pcl::PointCloud<siftDepth> siftDMatch(pcl::PointCloud<siftDepth> a, pcl::PointCloud<siftDepth> b)
{
    pcl::PointCloud<siftDepth> pclMatch;
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

        std::vector< std::vector < cv::DMatch > > matches;
        cv::BFMatcher matcher(cv::NORM_L2);
        matcher.knnMatch(descriptorsA, descriptorsB, matches, 2);  // Find two nearest matches
        std::vector<cv::DMatch> good_matches;
        for (int i = 0; i < matches.size(); ++i)
        {
            const float ratio = 0.6; // As in Lowe's paper;
            if (matches[i][0].distance < ratio * matches[i][1].distance)
            {
                good_matches.push_back(matches[i][0]);
            }
        }
/*
        cv::BFMatcher matcher(cv::NORM_L2);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptorsA, descriptorsB, matches);

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
       // std::cout << " sift max dist " << max_dist << std::endl;
       // std::cout << " sift mins dist " << min_dist << std::endl;

        sd.SetValues(temp, descriptorsA.rows);

        double mean = sd.CalculateMean();
        double variance = sd.CalculateVariane();
        double samplevariance = sd.CalculateSampleVariane();
        double sampledevi = sd.GetSampleStandardDeviation();
        double devi = sd.GetStandardDeviation();

        std::cout << "Sift\t" << descriptorsA.rows << "\t"
                << mean << "\t"
                << variance << "\t"
                << samplevariance << "\t"
                << devi << "\t"
                << sampledevi << "\n";

        std::vector< cv::DMatch > good_matches;

        for (int i=0;i<descriptorsA.rows;i++)
        {
            if( matches[i].distance< 30)
            {
                good_matches.push_back(matches[i]);
                pclMatch.push_back(a[i]);
            }
        }
*/
        //sift_lastKeypoints = sift_currentKeypoints;
        //sift_lastImg = sift_currentImg;
        cv::Mat img_matches;
        cv::drawMatches( sift_lastImg, sift_lastKeypoints, sift_currentImg, sift_lastKeypoints,
                           good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                           std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        //cv::imshow("sift Matches", img_matches);
        //cv::waitKey(10);
        //-- Localize the object
          std::vector<Point2f> obj;
          std::vector<Point2f> scene;

          for( int i = 0; i < good_matches.size(); i++ )
          {
            //-- Get the keypoints from the good matches
            obj.push_back( sift_lastKeypoints[ good_matches[i].queryIdx ].pt );
            scene.push_back( sift_currentKeypoints[ good_matches[i].trainIdx ].pt );
          }

          Mat H = findHomography( obj, scene, CV_RANSAC );

          //-- Get the corners from the image_1 ( the object to be "detected" )
          std::vector<cv::Point2f> obj_corners(4);
          obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( sift_lastImg.cols, 0 );
          obj_corners[2] = cvPoint( sift_lastImg.cols, sift_lastImg.rows ); obj_corners[3] = cvPoint( 0, sift_lastImg.rows );
          std::vector<Point2f> scene_corners(4);

          perspectiveTransform( obj_corners, scene_corners, H);

          //-- Draw lines between the corners (the mapped object in the scene - image_2 )
          line( img_matches, scene_corners[0] + Point2f( sift_lastImg.cols, 0), scene_corners[1] + Point2f( sift_lastImg.cols, 0), Scalar(0, 255, 0), 4 );
          line( img_matches, scene_corners[1] + Point2f( sift_lastImg.cols, 0), scene_corners[2] + Point2f( sift_lastImg.cols, 0), Scalar( 0, 255, 0), 4 );
          line( img_matches, scene_corners[2] + Point2f( sift_lastImg.cols, 0), scene_corners[3] + Point2f( sift_lastImg.cols, 0), Scalar( 0, 255, 0), 4 );
          line( img_matches, scene_corners[3] + Point2f( sift_lastImg.cols, 0), scene_corners[0] + Point2f( sift_lastImg.cols, 0), Scalar( 0, 255, 0), 4 );

          //-- Show detected matches
          imshow( "Good Matches & Object detection", img_matches );

          waitKey(500);
       // std::cout << good_matches.size() << " sift features matched from, " << a.size() << ", " << b.size() << " sets." << std::endl;

    }
    catch (const std::exception &exc)
    {
        // catch anything thrown within try block that derives from std::exception
        std::cerr << exc.what();
    }
    return pclMatch;
}
