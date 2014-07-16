    #include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include "conversions.h"

const int width_degree  = 57;
const int height_degree = 43;

#define PI 3.14159265;

class Kinect {

    public:

        Kinect(ros::NodeHandle* _n)
        {
            detector.hessianThreshold = 400;

            initialising = true;

            this->n = *_n;
            this->sub = this->n.subscribe("/camera/rgb/image_color", 1, &Kinect::callBack, this);
            //this->sub = this->n.subscribe("/ptu_sweep/rgb/image_color", 1, &Kinect::callBack, this);
            count == 0;
        }

        void callBack(const sensor_msgs::ImageConstPtr& img)
        {
            cv::Mat descriptors, image(conversions(img));
            std::vector<cv::KeyPoint> keypoints;

            // detector && Extract surf
            detector.detect(image, keypoints);
            extractor.compute(image, keypoints, descriptors);

            if(initialising)  // store initial sift
            {
                first_img = conversions(img);
                first_keypoints = keypoints;
                first_descriptors = descriptors;
                initialising = false;

                /*
                first_keypoints = lastK;
                first_descriptors = lastD;
                first_img = lastI;
                lastI = conversions(img);
                lastK = keypoints;
                lastD = descriptors;
                initialising = false;
                */
            }
            else    // perform Match //if(count > 0)
            {
                std::vector< cv::DMatch > matches;
                matcher.match( first_descriptors, descriptors, matches );

                double max_dist = 0; double min_dist = 100;

                //-- Quick calculation of max and min distances between keypoints
                for( int i = 0; i < first_descriptors.rows; i++ )
                {
                    double dist = matches[i].distance;
                    if( dist < min_dist ) min_dist = dist;
                    if( dist > max_dist ) max_dist = dist;
                }

                std::vector< cv::DMatch > good_matches;

                for( int i = 0; i < first_descriptors.rows; i++ )
                {
                    if( matches[i].distance <= 3*min_dist )
                    {
                        good_matches.push_back( matches[i]);
                    }
                }

                cv::Mat img_matches;
                cv::drawMatches( first_img, first_keypoints, image, keypoints,
                             good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                             std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
cv::imshow("Surf Matches", img_matches);
                //-- Localize the object
                  std::vector<cv::Point2f> obj;
                  std::vector<cv::Point2f> scene;

                  for( int i = 0; i < good_matches.size(); i++ )
                  {
                    //-- Get the keypoints from the good matches
                    obj.push_back( first_keypoints[ good_matches[i].queryIdx ].pt );
                    scene.push_back( keypoints[ good_matches[i].trainIdx ].pt );
                  }

/*
                  cv::Mat H = cv::findHomography( obj, scene, CV_RANSAC );



                  cv::Mat result;
                  warpPerspective(first_img,result,H,cv::Size(first_img.cols+image.cols,first_img.rows));
                  cv::Mat half(result,cv::Rect(0,0,image.cols,image.rows));
                  image.copyTo(half);
                  cv::imshow( "Result", result );

                  cv::waitKey(10);

                  //-- Get the corners from the image_1 ( the object to be "detected" )
                  std::vector<cv::Point2f> obj_corners(4);
                  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( first_img.cols, 0 );
                  obj_corners[2] = cvPoint( first_img.cols, first_img.rows ); obj_corners[3] = cvPoint( 0, first_img.rows );
                  std::vector<cv::Point2f> scene_corners(4);

                  perspectiveTransform( obj_corners, scene_corners, H);

                  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
                  line( img_matches, scene_corners[0] + cv::Point2f( first_img.cols, 0), scene_corners[1] + cv::Point2f( first_img.cols, 0), cv::Scalar(0, 255, 0), 4 );
                  line( img_matches, scene_corners[1] + cv::Point2f( first_img.cols, 0), scene_corners[2] + cv::Point2f( first_img.cols, 0), cv::Scalar( 0, 255, 0), 4 );
                  line( img_matches, scene_corners[2] + cv::Point2f( first_img.cols, 0), scene_corners[3] + cv::Point2f( first_img.cols, 0), cv::Scalar( 0, 255, 0), 4 );
                  line( img_matches, scene_corners[3] + cv::Point2f( first_img.cols, 0), scene_corners[0] + cv::Point2f( first_img.cols, 0), cv::Scalar( 0, 255, 0), 4 );

                  //-- Show detected matches
                  imshow( "Good Matches & Object detection", img_matches );
*/
                  cv::waitKey(10);

                float  mytemp, result;

                  float angle = 640 /57;
                  // lets try getting some angles

                int f = 589;// claims 525


                //for(int i  = 0; i < good_matches.size(); i++)
                for(int i  = 0; i < 4; i++)
                {
                      mytemp = atan((scene[i].x-320)/f) * 180 / PI;
                       result = (scene[i].x-320)/angle;
                      /*if(floor(mytemp)==27)
                      {
                        std::cout << mytemp << "°\t";
                      }
                      if(ceil(mytemp)==-27)
                      {
                        std::cout << mytemp << "°\t";
                      }
                      */std::cout << mytemp << "° - " << result << "°\t\t" ;
                  }std::cout << std::endl;


            }
            count++;
        }


protected:
        ros::Subscriber sub;
        ros::NodeHandle n;
        cv::BRISK temp;

        cv::SurfFeatureDetector detector;
        cv::SurfDescriptorExtractor extractor;
        cv::FlannBasedMatcher matcher;

        cv::Mat first_img, first_descriptors;
        std::vector<cv::KeyPoint> first_keypoints;
        bool initialising;
        int count;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Surf");
    ros::NodeHandle n;

    Kinect e(&n);
    ros::spin();

}
