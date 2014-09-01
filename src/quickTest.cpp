#include <ros/ros.h>
#include <ros/time.h>
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
#include <geometry_msgs/Pose.h>
#include <boost/foreach.hpp>
#include "conversions.h"
#include <fstream>
#include <dirent.h>
#include <algorithm>
#include <unistd.h>
#include <ctime>

#define PI 3.14159265
#define DISTANCE_FACTOR 0.7

float PTU_UPPERLIMIT = 3;
float PTU_LOWERLIMIT = -3;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::JointState> MySyncPolicy;

int switchNum = 1;

struct feature_struct {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
};

struct estimation
{
    std::string name;
    float angle;
    int matches;
};

class ptu_features
{
    public:
        std::string locationID;

        ptu_features(ros::NodeHandle* _n)    // Constructor
        {
            n = *_n;
            ptu.name.resize(2);
            ptu.position.resize(2);
            ptu.velocity.resize(2);

            ptu_pub = this->n.advertise<sensor_msgs::JointState>("/ptu/cmd", 1);

            myfile.open("tempGraphical.txt");
            count = 0;
            detector.hessianThreshold = 1000;
            ptu_start_angle = 1000; // init to number greater than 0-360
            f = 590;                // claims 525
            pointer = 0;
            lastPtuAng = 1000;
            ptu_iterator = 45;
            ptu_pose = -180;
            ptu_stationary = 0.05;

            this->pose_sub = this->n.subscribe("/robot_pose", 10, &ptu_features::poseCallback, this);
            realRobot_pose = 1000;

            int cluster = 100;
            bow = new cv::BOWKMeansTrainer(cluster,cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, FLT_EPSILON), 1, cv::KMEANS_PP_CENTERS );

        }

        void callback(const sensor_msgs::ImageConstPtr &img,  const sensor_msgs::JointStateConstPtr &ptu_state)
        {
            switch (switchNum)
            {
            case 0:
            {
                std::stringstream ss;
                ss<<this->locationID<< ".jpg";

                cv::imwrite(ss.str(), conversions(img));
                switchNum = 1;
                break;
            }
            case 1:
                for (int i = 0;i<ptu_state->name.size();i++)
                {
                    if (ptu_state->name[i] == "pan") {
                        ptuAng = ptu_state->position[i] * 60;
                    }
                }
                if(lastPtuAng == ptuAng && lastPtuAng != 1000)
                {
                    if(((double) (clock() - stationary_clock) / CLOCKS_PER_SEC) > ptu_stationary)
                    {
                        if(ptu_pose <= 180)
                        {
                            if(ptu_start_angle == 1000)
                            {
                                ptu_start_angle = ptuAng;
                            }else{
                                extractFeatures(conversions(img), feature_sphere, ptuAng);
                            }
                            stationary_clock = clock();
                            moveCam(ptu_pose);

                            ptu_pose += ptu_iterator;
                        }
                        else    // ptu spin complete
                        {
                            std::cout << "the sweep is complete" << std::endl;
                            if(this->read_features.size() > 0)
                            {
                                for(int i = 0; i < whereAmI.size(); i ++)
                                {
                                    this->pointer = i;
                                    myfile << whereAmI[i].name << "\t";
                                    match(read_features[i], feature_sphere);
                                    myfile << "\n";
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
                                locationID = whereAmI[bestMatch].name;
                                std::cout << "\nI believe I am at " << locationID << " and " <<whereAmI[bestMatch].angle << " off angle!"<< std::endl;
                                myfile.close();

                                int a = whereAmI[bestMatch].angle;
                                moveCam(((-int(a)-180) % 360)+180);
                                switchNum = 2;
                                sleep(5);

                            }
                            else
                            {
                                moveCam(0);
                                while(!save(feature_sphere));
                                std::cout << "File Saved!" << std::endl;
                                std::exit(1);
                            }
                        }
                    }
                }else
                    stationary_clock = clock();

                lastPtuAng = ptuAng;
            break;
            case 2:
                    finalComparason(conversions(img));


                break;
            default:
                std::cout << "UNKNOWN SWITCH STATEMENT CLOSING" << std::endl;
                std::exit(1);
                break;
            }

           /* for (int i = 0;i<ptu_state->name.size();i++)
            {
                if (ptu_state->name[i] == "pan") ptuAng = ptu_state->position[i];
                if (this->lastPtuAng == ptuAng)
                {
                    catch_ptu = true;
                    moveCam(180);
                    if(ptu_start_angle == 1000)                         // First run?
                    {
                        std::cout << "Here they come!" << std::endl;
                        ptu_start_angle = ptu_angle;
                    }
                    std::cout << "same" << std::endl;
                    //extractFeatures(conversions(img), feature_sphere, ptu_angle);   // Extract
                }else catch_ptu = false;

            }
            lastPtuAng = ptuAng;

/*
            if(!catch_ptu)
            {
                for (int i = 0;i<ptu_state->name.size();i++)
                {
                    if (ptu_state->name[i] == "pan") ptuAng = ptu_state->position[i];
                    if (this->lastPtuAng == ptuAng)
                    {
                        catch_ptu = true;
                        moveCam();
                        if(ptu_start_angle == 1000)                         // First run?
                        {
                            std::cout << "Here they come!" << std::endl;
                            ptu_start_angle = ptu_angle;
                        }
                        extractFeatures(conversions(img), feature_sphere, ptu_angle);   // Extract
                    }else catch_ptu = false;

                }
                lastPtuAng = ptuAng;
            }
*/
            /*
            ptu_angle = ptu_state->position[0] * 60;            // Joint state transformed into 360

            if(ptu_start_angle == 1000)                         // First run?
            {
                std::cout << "Here they come!" << std::endl;
                ptu_start_angle = ptu_angle;
            }

            extractFeatures(conversions(img), feature_sphere, ptu_angle);   // Extract
            //std::cout << count << "\t" << ptu_angle << std::endl;

            //if(count == 31)
            if((ceil(ptu_angle / 10) * 10) == (ceil((ptu_start_angle*-1) / 10) * 10))   // completed
            {
                if(this->read_features.size() > 0)
                {
                    for(int i = 0; i < whereAmI.size(); i ++)
                    {
                        this->pointer = i;
                        myfile << whereAmI[i].name << "\t";
                        match(read_features[i], feature_sphere);
                        myfile << "\n";
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
                    myfile.close();
                    std::exit(1);
                }
                else
                {
                    while(!save(feature_sphere));
                    std::cout << "File Saved!" << std::endl;
                    std::exit(1);
                }
            }
            count++;
*/
        }                // output each degree
        // std::cout << hist[i] << "\t" << roundHist[i] << std::endl;

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

                    guessTemp.name = fn.substr(0, fn.find_last_of("."));
                    whereAmI.push_back(guessTemp);

                    std::cout << "Successfully Loaded " << this->read_features[this->read_features.size()-1].keypoints.size() << " keypoints from " << fn << std::endl;
                }
            }
        }

        bool CheckCoherentRotation(cv::Mat_<double>& R) {
            if(fabsf(determinant(R))-1.0 > 1e-07) {
                std::cerr << "det(R) != +-1.0, this is not a rotation matrix" << std::endl;
                return false;
            }
            return true;
        }

        void finalComparason(cv::Mat img_2)
        {

            std::stringstream ss;
            ss<<this->locationID<< ".jpg";
            std::cout << ss.str() << std::endl;
            cv::Mat img_1 = cv::imread( ss.str(), CV_LOAD_IMAGE_GRAYSCALE );



            if( !img_1.data || !img_2.data )
            { return; }

            //-- Step 1: Detect the keypoints using SURF Detector
            int minHessian = 400;
            cv::SurfFeatureDetector detector( minHessian );
            std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
            detector.detect( img_1, keypoints_1 );
            detector.detect( img_2, keypoints_2 );

            //-- Step 2: Calculate descriptors (feature vectors)
            cv::SurfDescriptorExtractor extractor;
            cv::Mat descriptors_1, descriptors_2;
            extractor.compute( img_1, keypoints_1, descriptors_1 );
            extractor.compute( img_2, keypoints_2, descriptors_2 );

            //-- Step 3: Matching descriptor vectors with a brute force matcher
            cv::BFMatcher matcher(cv::NORM_L1, true);
            std::vector< cv::DMatch > prematches, matches;
            matcher.match( descriptors_1, descriptors_2, prematches );

            for(unsigned int i = 0; i < prematches.size(); i++)
                if(prematches[i].distance < 0.5)
                    matches.push_back(prematches[i]);

            //-- Draw matches
            cv::Mat img_matches;
            cv::drawMatches( img_1, keypoints_1, img_2, keypoints_2, matches, img_matches );
            //-- Show detected matches
            cv::namedWindow( "Matches", CV_WINDOW_NORMAL );
            cv::imshow("Matches", img_matches );
            cv::waitKey(33);


            //-- Step 4: calculate Fundamental Matrix
            std::vector<cv::Point2f>imgpts1,imgpts2;
            for( unsigned int i = 0; i<matches.size(); i++ )
            {
            // queryIdx is the "left" image
            imgpts1.push_back(keypoints_1[matches[i].queryIdx].pt);
            // trainIdx is the "right" image
            imgpts2.push_back(keypoints_2[matches[i].trainIdx].pt);
            }
            cv::Mat F =  cv::findFundamentalMat(imgpts1, imgpts2, cv::FM_RANSAC, 0.1, 0.99);

            std::cout << "["<< F.at<double>(0, 0) << "\t" << F.at<double>(0, 1) << "\t" << F.at<double>(0, 2) << std::endl;
            std::cout << F.at<double>(1, 0) << "\t" << F.at<double>(1, 1) << "\t" << F.at<double>(1, 2) << std::endl;
            std::cout << F.at<double>(2, 0) << "\t" << F.at<double>(2, 1) << "\t" << F.at<double>(2, 2) << "]" << std::endl << std::endl;

            //-- Step 5: calculate Essential Matrix
            double data[] = {525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0};//Camera Matrix
            cv::Mat K(3, 3, CV_64F, data);
            cv::Mat_<double> E = K.t() * F * K; //according to HZ (9.12)

            //-- Step 6: calculate Rotation Matrix and Translation Vector
            cv::Matx34d P;
            cv::Matx34d P1;
            //decompose E to P' , HZ (9.19)
            cv::SVD svd(E,cv::SVD::MODIFY_A);
            cv::Mat svd_u = svd.u;
            cv::Mat svd_vt = svd.vt;
            cv::Mat svd_w = svd.w;
            cv::Matx33d W(0,-1,0,1,0,0,0,0,1);//HZ 9.13
            cv::Mat_<double> R = svd_u * cv::Mat(W) * svd_vt; //HZ 9.19
            cv::Mat_<double> t = svd_u.col(2); //u3

            if (!CheckCoherentRotation (R)) {
            std::cout<<"resulting rotation is not coherent\n";
            P1 = 0;
            return;
            }

            P1 = cv::Matx34d(R(0,0),R(0,1),R(0,2),t(0),
                         R(1,0),R(1,1),R(1,2),t(1),
                         R(2,0),R(2,1),R(2,2),t(2));

            //-- Step 7: Reprojection Matrix and rectification data
            cv::Mat R1, R2, P1_, P2_, Q;
            cv::Rect validRoi[2];
            double dist[] = { -0.03432, 0.05332, -0.00347, 0.00106, 0.00000};
            cv::Mat D(1, 5, CV_64F, dist);

            cv::stereoRectify(K, D, K, D, img_1.size(), R, t, R1, R2, P1_, P2_, Q, CV_CALIB_ZERO_DISPARITY, 1, img_1.size(),  &validRoi[0], &validRoi[1] );
        }

        void moveCam(float angle)
        {
            std::cout << angle << std::endl;
            ptu.name[0] ="tilt";
            ptu.name[1] ="pan";
            ptu.position[0] = 0.0;
            ptu.position[1] = angle / 60;
            ptu.velocity[0] = 1.0;
            ptu.velocity[1] = 1.0;
            if(ptu.position[1] <= PTU_UPPERLIMIT && ptu.position[1] >= PTU_LOWERLIMIT)
            {
                ptu_pub.publish(ptu);
            }
            else
            {
                ptu.position[1] = 0;    // reset to nill
                ptu_pub.publish(ptu);
                std::exit(1);
            }
        }

private:
        void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
        {
            realRobot_pose = msg->orientation.z * 180;
        }

        void extractFeatures(cv::Mat image, feature_struct &sphere, float angle)
        {
            //cv::Rect myROI(264, 0, 112, 480); // Crop image for
            cv::Rect myROI(65, 0, 510, 480); // Crop image for#
            image = image(myROI);

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
                keypoints[i].pt.x  = -atan((temp.pt.x-320)/f) * 180 / PI;
                keypoints[i].pt.y  = -atan((temp.pt.y-240)/f) * 180 / PI;

                keypoints[i].pt.x += angle; // Add PTU angle
            }

            sphere.descriptors.push_back(descriptors);
            sphere.keypoints.insert(sphere.keypoints.end(), keypoints.begin(), keypoints.end() );
        }

        bool save(feature_struct &sphere)
        {
            int dictionarySize=1000;

            cv::TermCriteria tc(CV_TERMCRIT_ITER,100,0.001);
            //retries number
            int retries=1;
            //necessary flags
            int flags = cv::KMEANS_PP_CENTERS;
            //Create the BoW (or BoF) trainer
            cv::BOWKMeansTrainer bowTrainer(dictionarySize,tc,retries,flags);
            cv::Mat output = bowTrainer.cluster(sphere.descriptors);

            cv::Mat old = sphere.descriptors;

            cv::FlannBasedMatcher matcher;
            std::vector< cv::DMatch > matches;
            matcher.match( output, old, matches );

            std::vector<cv::KeyPoint> keypoints;

            for(int i = 0; i < matches.size(); i++)
            {
                keypoints.push_back(sphere.keypoints[matches[i].trainIdx]);
            }

            if(locationID.empty())
            {
                moveCam(0);
                std::cout << "Enter file name to save:\n";
                std::cin >> locationID;
            }

            if(!locationID.empty())
            {
                if(locationID.substr(locationID.find_last_of(".") + 1) != "yml") {
                    locationID.append(".yml");
                }
                cv::FileStorage fs(locationID, cv::FileStorage::WRITE);
                //cv::write(fs, "keypoints", sphere.keypoints);
                //cv::write(fs, "descriptors", sphere.descriptors);

                cv::write(fs, "keypoints", keypoints);
                cv::write(fs, "descriptors", output);

                fs.release();

                return true;
            }
            return false;
        }

        bool match(feature_struct &a, feature_struct &b)
        {
            float hist[360];
            float roundHist[360];
            for(int i = 0; i < 360; i++)
            {
                hist[i] = 0;
                roundHist[i] = 0;
            }

            //std::vector<std::vector <cv::DMatch> > matches;
            std::vector<std::vector <cv::DMatch> > preProcessedMatches;
            std::vector<cv::DMatch> matches;

            cv::BFMatcher matcher;

            if(a.descriptors.rows > 1 && b.descriptors.rows > 1)
            {
                matcher.knnMatch( a.descriptors, b.descriptors, preProcessedMatches, 2);

                std::cout << "Matches " << preProcessedMatches.size() << std::endl;
                int NumMatches = 0;
                for(   int i=0; i < preProcessedMatches.size(); i++)
                {
                    if (preProcessedMatches[i].size() == 2)
                    {
                        if (preProcessedMatches[i][0].distance < preProcessedMatches[i][1].distance * DISTANCE_FACTOR)
                        {
                            cv::DMatch match = cv::DMatch(preProcessedMatches[i][0].queryIdx, preProcessedMatches[i][0].trainIdx, preProcessedMatches[i][0].distance);
                            matches.push_back(match);
                            NumMatches++;
                        }else{
                            cv::DMatch match = cv::DMatch(preProcessedMatches[i][0].queryIdx, -1, preProcessedMatches[i][0].distance);
                            matches.push_back(match);
                        }
                    }
                    else if (preProcessedMatches[i].size() == 1)
                    {
                        cv::DMatch match = cv::DMatch(preProcessedMatches[i][0].queryIdx, preProcessedMatches[i][0].trainIdx, preProcessedMatches[i][0].distance);
                        matches.push_back(match);
                    }else{
                        cv::DMatch match = cv::DMatch(preProcessedMatches[i][0].queryIdx, -1, preProcessedMatches[i][0].distance);
                        matches.push_back(match);
                    }

                    if(std::max(a.keypoints[i].pt.y, b.keypoints[matches[i].trainIdx].pt.y)
                            - std::max(a.keypoints[i].pt.y, b.keypoints[matches[i].trainIdx].pt.y) < 50)
                    {
                        // NORMAL CODE
                        int difAng = ceil(a.keypoints[i].pt.x - b.keypoints[matches[i].trainIdx].pt.x);
                        if(difAng < 0)
                            difAng = 360 + difAng;// + round(realRobot_pose);
                        NumMatches++;
                        //hist[difAng]++;
                        if(matches[i].distance < 0.15)
                        {
                            hist[difAng] += 1- matches[i].distance;
                        }
                    }
                }


                int bestPointer = 0;
                float angle = 0;
                std::cout << std::endl;
                for(int i = 0; i < 360; i++)
                {
                    myfile << hist[i] << "\t";
                    for(int j = 0; j < 5; j++)
                    {
                        roundHist[i] += hist[(i-2+j)%360];
                    }
                    //roundHist[i] /= 5;
                    if(bestPointer < hist[i])
                    {
                        bestPointer=hist[i];
                        angle = (float)i;
                    }
                    // output each degree
                   // std::cout << hist[i] << "\t" << roundHist[i] << std::endl;
                }
                //std::cout << std::endl;
                std::cout << "Estimation for: " << whereAmI[pointer].name << " " << angle << " degrees, (" << bestPointer << ") Matches" << std::endl;

                whereAmI[this->pointer].angle = angle;
                whereAmI[this->pointer].matches = NumMatches;
            }
        }

protected:
        ros::NodeHandle n;
        int count;
        unsigned int pointer;
        float f;
        cv::SurfFeatureDetector detector;
        cv::SurfDescriptorExtractor extractor;
        cv::FlannBasedMatcher matcher;
        feature_struct feature_sphere;
        std::vector<feature_struct> read_features;
        float ptu_start_angle;
        float ptu_angle;
        std::vector<estimation> whereAmI;
        std::ofstream myfile;

        // new
        ros::Publisher  ptu_pub;
        ros::Subscriber pose_sub;
        sensor_msgs::JointState ptu;

        float lastPtuAng, ptuAng;
        float ptu_pose;
        int ptu_iterator; // in degrees
        float ptu_stationary;
        clock_t stationary_clock;
        float realRobot_pose;

        cv::Ptr<cv::DescriptorMatcher> matcher2;
        cv::Ptr<cv::DescriptorExtractor> extractor2;
        cv::Ptr<cv::BOWImgDescriptorExtractor> dextract;
        cv::SurfFeatureDetector detector2();
        cv::Ptr<cv::BOWKMeansTrainer> bow;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "Surf");

    ros::NodeHandle n;
    ptu_features camClass(&n);

    bool getData = false;    // true for runs

    // read old location Data
    if(argc > 1)
        if(getData)
        {
            camClass.locationID = argv[1];
            switchNum = 0;
        }else camClass.read(argv[1], false);

        //camClass.locationID = argv[1];
        //camClass.read(argv[1], false);

    message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/head_xtion/rgb/image_color", 1);
    message_filters::Subscriber<sensor_msgs::JointState> ptu_sub(n, "/ptu/state", 1);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(33), image_sub, ptu_sub);

    sync.registerCallback(boost::bind(&ptu_features::callback, &camClass, _1, _2));

    std::cout << "Waiting for syncornized topics." << std::endl;

    ros::spin();
}
