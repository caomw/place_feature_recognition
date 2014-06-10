#include "bags.h"

using namespace boost;


bagHandler::bagHandler()    // Blank init
{
    visBool = false;
    backgroundFileName = "";
    printOutput = true;
    isRobot = false;
}

void bagHandler::extract()
{
    surfClass * x = new surfClass;
    narfClass * n = new narfClass;
    visualisation visualiser;

    rosbag::Bag bag;

    DIR *dir;
    struct dirent *ent;
    char name[200];

    ros::Time::init();
    rosbag::Bag myBag;

    std::vector<cv::Mat> backgroundVect;
    std::vector<cv::Mat> objectVect;

    myBag.open(outputBag, rosbag::bagmode::Write);
    int bagCount = 0;
    //int someCount=0;
    if ((dir = opendir (inputFolder.c_str())) != NULL) {
        while ((ent = readdir (dir)) != NULL )
        {
            sprintf(name,"%s/%s", inputFolder.c_str(), ent->d_name);

            std::string testExtension(name);
            if(testExtension.substr(testExtension.find_last_of(".")+1) == "bag")
            {
                // Interpret which bag to use.
                int bagNum;
                std::istringstream (((std::string)name).substr((((std::string)name).find_last_of("_") + 1),1)) >> bagNum;

                std::cout << ent->d_name << "\t";

                // Open Bags
                bag.open(name, rosbag::bagmode::Read);

                // define topics to sub too
                std::vector<std::string> topics;

                if(isRobot){      // Add Robot topics
                    topics.push_back(std::string("/camera/rgb/image_color")); topics.push_back(std::string("/camera/depth/points"));
                    } else {        // Add Openni topics
                    topics.push_back(std::string("/head_xtion/depth/points")); topics.push_back(std::string("/head_xtion/rgb/image_color"));
                }

                rosbag::View view(bag, rosbag::TopicQuery(topics));
                bagCount+=view.size();

                // Iterate through messages in bag
                BOOST_FOREACH(rosbag::MessageInstance const m, view){

                    try { // Handle instantiation breaks
                        sensor_msgs::PointCloud2::Ptr 	p = m.instantiate<sensor_msgs::PointCloud2>();
                        sensor_msgs::Image::Ptr 		i = m.instantiate<sensor_msgs::Image>();

                        // Pointcloud// SURF
                        if(p != NULL && false)
                        {
                            narfStruct ns;
                            ns = narf(p);   // get Narf Features

                            if(visBool)visualiser.visualise(ns);

                            p->header.frame_id = "/camera_depth_frame";
                            p->header.stamp = ros::Time();

                            myBag.write(topics[1], ros::Time::now(), *p);   // cloud
                            sensor_msgs::PointCloud2 narf2;
                            pcl::toROSMsg(ns.narf_descriptors,narf2);
                            myBag.write("/narfFeatures", ros::Time::now(), narf2);
                        }

                        // Image Topic
                        if(i != NULL)
                        {
                            surfStruct surfFeatures;
                            surfFeatures = surf(i, 1000);   // get Surf Features
                            cv::Size s = surfFeatures.descriptors.size();

                            // Perform DMatch
                            cv::Size l = lastFrame.size();;
                            if(l.height > 1)
                            {
                                DMatch(lastSurf, lastFrame, surfFeatures, conversions(i));
                            }

                            if(visBool && s.height > 1)
                            {
                            //    visualiser.visualise(surfFeatures.keypoints, conversions(i));
                                cv::waitKey(10);
                            }
                            myBag.write(topics[0], ros::Time::now(), i);    // Image
                            myBag.write("/surfFeatures", ros::Time::now(), conversions(surfFeatures.descriptors));  // Surf Features

                            if(s.height > 1){
                                lastSurf = surfFeatures;
                                lastFrame = conversions(i);
                                if(name == backgroundFileName){   // background image?
                                    backgroundVect.push_back(surfFeatures.descriptors);
                                } else {                           // forground image?
                                    objectVect.push_back(surfFeatures.descriptors);
                                }
                            }
                        }
                    }
                    catch (const rosbag::BagFormatException& e)
                    {
                        cout << "| " << e.what() << "!";
                        continue;
                    }
                }
                std::cout <<  std::endl;
            }
            bag.close();
        }
    }
    cout << "background size: " <<backgroundVect.size() << "\n object size: "<<objectVect.size() << std::endl;
    std::cout << "Examined " << bagCount << " Files" << std::endl;
    myBag.close();
}

