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
#include <dirent.h>
#include <algorithm>

#define PI 3.14159265;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::JointState> MySyncPolicy;

struct feature_struct {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
};

struct estimation
{
    std::string name;
    int angle;
    int matches;
};

class ptu_features
{
    public:
        ptu_features()    // Constructor
        {
            count = 0;
            detector.hessianThreshold = 400;
            ptu_start_angle = 1000; // init to number greater than 0-360
            //feature_sphere.descriptors = cv::Mat::zeros(0, 64, CV_32F);
            f = 590;                // claims 525
            pointer = 0;
        }

        void callback(const sensor_msgs::ImageConstPtr &img,  const sensor_msgs::JointStateConstPtr &ptu_state)
        {
            count++;

            ptu_angle = ptu_state->position[0] * 60;            // Joint state transformed into 360

            if(ptu_start_angle == 1000)                         // First run?
            {
                std::cout << "Here they come!" << std::endl;
                ptu_start_angle = ptu_angle;
            }

            extractFeatures(conversions(img), feature_sphere, ptu_angle);   // Extract

            if((ceil(ptu_angle / 10) * 10) == (ceil((ptu_start_angle*-1) / 10) * 10))   // completed
            //if(count == 31)
            {
                if(this->read_features.size() > 0)
                {
                    for(int i = 0; i < whereAmI.size(); i ++)
                    {
                        this->pointer = i;
                        match(read_features[i], feature_sphere);
                    }

                    int bestPos = 0;
                    int bestMatch;

                    for(int i = 0; i < whereAmI.size(); i ++)
                    {
                        if(bestPos < whereAmI[i].matches)
                        {
                            bestPos=whereAmI[i].matches;
                            bestMatch = i;
                        }
                    }
                    std::cout << "\nI believe I am at " << whereAmI[bestMatch].name << " and " <<whereAmI[bestMatch].angle << " off angle!"<< std::endl;
                }
                else
                {
                    while(!save(feature_sphere));
                    std::cout << "File Saved!" << std::endl;
                }
            }
        }

        void read(std::string fn, bool firstLoop)
        {
            bool isDir = false;
            DIR *dir;
            struct dirent *ent;
            char name[200];
            if ((dir = opendir (fn.c_str())) != NULL && !firstLoop) {
                while ((ent = readdir(dir)) != NULL )
                {
                    sprintf(name,"%s/%s", fn.c_str(), ent->d_name);
                    read(name, true);// << "\n";
                }
            }
            else
            {
                if(fn.substr(fn.find_last_of(".") + 1) != "yml")
                {
                    return;
                }
                char realFile[200];
                strcpy(realFile, fn.c_str());
                std::ifstream my_file(realFile);
                if (my_file)    // is it a real file
                {
                    feature_struct temp;
                    cv::FileStorage fs2(fn, cv::FileStorage::READ);

                    fs2["descriptors"] >> temp.descriptors; //this->read_features[0].descriptors;

                    cv::FileNode  kptFileNode1 = fs2["keypoints"];
                    cv::read( kptFileNode1, temp.keypoints);// this->read_features[0].keypoints );

                    fs2.release();

                    this->read_features.push_back(temp);

                    estimation guessTemp;
                    guessTemp.angle = 0;
                    guessTemp.matches =0;
                    guessTemp.name = fn;
                    whereAmI.push_back(guessTemp);
                    std::cout << "Successfully Loaded " << this->read_features[this->read_features.size()-1].keypoints.size() << " keypoints from "
                              << fn << std::endl;
                }
            }
        }

private:

        void extractFeatures(cv::Mat image, feature_struct &sphere, float angle)
        {

            //std::cout << "sphere type: " << sphere.descriptors.type() << ".\t";
            cv::Mat descriptors;
            std::vector<cv::KeyPoint> keypoints;

            // detector && Extract surf
            detector.detect(image, keypoints);
            extractor.compute(image, keypoints, descriptors);

            cv::KeyPoint temp;
            for(int i = 0; i < keypoints.size(); i++)
            {
                temp = keypoints[i];
                keypoints[i].pt.x  = -(atan((temp.pt.x-320)/f) * 180 / PI);
                keypoints[i].pt.y  = -(atan((temp.pt.y-240)/f) * 180 / PI);
                keypoints[i].pt.x += angle; // Add PTU angle
            }

            sphere.descriptors.push_back(descriptors);
            sphere.keypoints.insert(sphere.keypoints.end(), keypoints.begin(), keypoints.end() );
/*
            std::ofstream myfile;

            std::ostringstream ss;
            ss << "Aisle2_90|" << angle;
            std::string s(ss.str());

            myfile.open (s.c_str());

            //std::cout << "sphere type: " << sphere.descriptors.type() << ".\t";
            cv::Mat descriptors;
            std::vector<cv::KeyPoint> keypoints;

            // detector && Extract surf
            detector.detect(image, keypoints);
            extractor.compute(image, keypoints, descriptors);

            sphere.descriptors.push_back(descriptors);
            sphere.keypoints.insert(sphere.keypoints.end(), keypoints.begin(), keypoints.end() );
            for(int i = 0 ; i < keypoints.size; i++)
            {
                myfile << keypoints[i].angle << ", "
                          << keypoints[i].pt.x << ", "
                             << keypoints[i].pt.y << ", "
                                << keypoints[i].response << ", "
                                   << keypoints[i].size << ", "
                                      << keypoints[i].octave << ", ";

                for(int j = 0; j < 64; j++)
                {
                    myfile << descriptors.at<float>(i,j);
                    if(j!= 63)
                         myfile << ", ";
                }
                myfile<<std::endl;
            }

            myfile.close();
            */
        }

        bool save(feature_struct &sphere)
        {
            std::cout << "Enter file name to save:\n";
            std::string fileName;
            std::cin >> fileName;


            if(!fileName.empty())
            {
                /*
                std::ofstream myfile;
                myfile.open (fileName.c_str());

                for(int i = 0 ; i < sphere.keypoints.size(); i++)
                {
                    myfile << sphere.keypoints[i].angle << ", "
                              << sphere.keypoints[i].pt.x << ", "
                                 << sphere.keypoints[i].pt.y << ", "
                                    << sphere.keypoints[i].response << ", "
                                       << sphere.keypoints[i].size << ", "
                                          << sphere.keypoints[i].octave << ", ";

                    for(int j = 0; j < 64; j++)
                    {
                        myfile<<sphere.descriptors.at<float>(i,j);
                        if(j!= 63)
                             myfile << ", ";
                    }
                    myfile<<std::endl;
                }

                myfile.close();
                */
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
            int hist[360];
            int roundHist[360];
            for(int i = 0; i < 360; i++)
            {
                hist[i] = 0;
                roundHist[i] = 0;
            }
            std::vector<std::vector <cv::DMatch> > matches;
            cv::BFMatcher matcher;
            matcher.knnMatch( a.descriptors, b.descriptors, matches, 2);

            int NumMatches = 0;
            for (int i = 0; i < matches.size(); ++i)
            {
                const float ratio = 0.08; // As in Lowe's paper;
                if(matches[i][0].distance < ratio)
                {
                    int difAng = ceil(a.keypoints[i].pt.x - b.keypoints[matches[i][0].trainIdx].pt.x);
                    if(difAng < 0)
                        difAng = 360 + difAng;
                    if(std::max(a.keypoints[i].pt.y, b.keypoints[matches[i][0].trainIdx].pt.y)
                            - std::max(a.keypoints[i].pt.y, b.keypoints[matches[i][0].trainIdx].pt.y) < 5)
                    {
                        NumMatches++;
                        hist[difAng]++;
                    }

                }
            }
            int bestPointer = 0;
            int angle = 0;
            for(int i = 0; i < 360; i++)
            {
                if(bestPointer < hist[i])
                {
                    bestPointer=hist[i];
                    angle = i;
                }
            }

            std::cout << "Estimation: " << angle << " degrees, (" << NumMatches << ") Matches" << std::endl;
            whereAmI[this->pointer].angle = angle;
            whereAmI[this->pointer].matches = NumMatches;
        }

protected:
        int count;
        int pointer;
        float f;
        cv::SurfFeatureDetector detector;
        cv::SurfDescriptorExtractor extractor;
        cv::FlannBasedMatcher matcher;
        feature_struct feature_sphere;
        std::vector<feature_struct> read_features;
        float ptu_start_angle;
        float ptu_angle;
        std::vector<estimation> whereAmI;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Surf");

    ros::NodeHandle n;
    ptu_features camClass;

    // read old location Data
    if(argc > 1)
        camClass.read(argv[1], false);

    message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/ptu_sweep/rgb/image_color", 1);
    message_filters::Subscriber<sensor_msgs::JointState> ptu_sub(n, "/ptu/state", 1);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), image_sub, ptu_sub);

    sync.registerCallback(boost::bind(&ptu_features::callback, &camClass, _1, _2));
    std::cout << "Waiting for syncornized topics." << std::endl;

    ros::spin();
}
