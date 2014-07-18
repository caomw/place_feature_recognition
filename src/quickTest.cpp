#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <boost/foreach.hpp>
#include "conversions.h"
#include <fstream>

#define PI 3.14159265;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::JointState> MySyncPolicy;

struct feature_struct {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
};

class ptu_features
{
    public:
        ptu_features()    // Constructor
        {
            detector.hessianThreshold = 400;
            ptu_start_angle = 1000; // init to number greater than 0-360
            feature_sphere.descriptors = cv::Mat::zeros(0, 64, CV_32F);
            f = 589;                // claims 525
        }

        void callback(const sensor_msgs::ImageConstPtr &img,  const sensor_msgs::JointStateConstPtr &ptu_state)
        {
            std::cout << ".";
            ptu_angle = ptu_state->position[0] * 60;                        // Joint state transformed into 360

            if(ptu_start_angle == 1000)ptu_start_angle = ptu_angle;         // First run?

            extractFeatures(conversions(img), feature_sphere, ptu_angle);   // Extract

            if((ceil(ptu_angle / 10) * 10) == (ceil((ptu_start_angle*-1) / 10) * 10))   // completed
            {
                if(this->read_features.keypoints.size() > 1)
                {
                    match(read_features, feature_sphere);
                }
                else
                {
                    while(!save(feature_sphere));
                    std::cout << "File Saved!" << std::endl;
                }
            }
        }

        void read(std::string fn)
        {
            if(fn.substr(fn.find_last_of(".") + 1) != "yml")
            {
                ROS_ERROR("Unable to convert none .yml file");
                return;
            }
            char realFile[200];
            strcpy(realFile, fn.c_str());
            std::ifstream my_file(realFile);
            if (!my_file) {
                ROS_ERROR("Cannot Locate %s", realFile);
                return;
            }
            else
            {
                cv::FileStorage fs2(fn, cv::FileStorage::READ);

                fs2["descriptors"] >> this->read_features.descriptors;

                cv::FileNode  kptFileNode1 = fs2["keypoints"];
                cv::read( kptFileNode1, this->read_features.keypoints );

                fs2.release();

                std::cout << "Successfully Loaded " << this->read_features.keypoints.size() << " keypoints from "
                          << fn << std::endl;
            }
        }

private:

        void extractFeatures(cv::Mat image, feature_struct &sphere, float angle)
        {
            cv::Mat descriptors;
            std::vector<cv::KeyPoint> keypoints;

            // detector && Extract surf
            detector.detect(image, keypoints);
            extractor.compute(image, keypoints, descriptors);

            for(int i = 0; i < keypoints.size(); i++)
            {
                keypoints[i].angle  = atan((keypoints[i].pt.x-320)/f) * 180 / PI;
                keypoints[i].angle += angle;
            }

            sphere.descriptors.push_back(descriptors);
            sphere.keypoints.insert(sphere.keypoints.end(), keypoints.begin(), keypoints.end() );
        }

        bool save(feature_struct &sphere)
        {
            std::cout << "Enter file name to save:\n";
            std::string fileName;
            std::cin >> fileName;

            if(!fileName.empty())
            {
                if(fileName.substr(fileName.find_last_of(".") + 1) != "yml") {
                    fileName.append(".yml");
                }
                cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
                cv::write(fs, "keypoints", sphere.keypoints);
                cv::write(fs, "descriptors", sphere.descriptors);

                fs.release();
                return true;
            }
            return false;
        }

        bool match(feature_struct &a, feature_struct &b)
        {
            std::cout << a.descriptors.size() << "\t" << b.descriptors.size();
            std::vector< cv::DMatch > matches;
            matcher.match( a.descriptors, b.descriptors, matches );

            double max_dist = 0; double min_dist = 100;

            //-- Quick calculation of max and min distances between keypoints
            for( int i = 0; i < a.descriptors.rows; i++ )
            { double dist = matches[i].distance;
              if( dist < min_dist ) min_dist = dist;
              if( dist > max_dist ) max_dist = dist;
            }

            printf("-- Max dist : %f \n", max_dist );
            printf("-- Min dist : %f \n", min_dist );

            //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
            std::vector< cv::DMatch > good_matches;

            for( int i = 0; i < a.descriptors.rows; i++ )
            { if( matches[i].distance < 3*min_dist )
               { good_matches.push_back( matches[i]); }
            }
            std::cout << good_matches.size() << std::endl;
            std::cout << "fin" << std::endl;
        }

protected:
        int f;
        cv::SurfFeatureDetector detector;
        cv::SurfDescriptorExtractor extractor;
        cv::FlannBasedMatcher matcher;
        feature_struct feature_sphere;
        feature_struct read_features;
        float ptu_start_angle;
        float ptu_angle;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Surf");
    ros::NodeHandle n;
    ptu_features camClass;

    // read old location Data
    if(argc > 1)
        camClass.read(argv[1]);

    message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/ptu_sweep/rgb/image_color", 1);
    message_filters::Subscriber<sensor_msgs::JointState> ptu_sub(n, "/ptu/state", 1);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, ptu_sub);

    sync.registerCallback(boost::bind(&ptu_features::callback, &camClass, _1, _2));
    std::cout << "Waiting for syncornized topics." << std::endl;

    ros::spin();
}
