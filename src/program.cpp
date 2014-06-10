#include <iostream>
#include <pcl/console/parse.h>
#include <ros/ros.h>
#include "bags.h"
#include "live.h"

void printUsage(const char* progName);

int main (int argc, char** argv)
{
    ros::init(argc, argv, "bags");
    bagHandler bh;

    if (pcl::console::find_argument (argc, argv, "-h") >= 0)
    {
        printUsage (argv[0]);
        return 0;
    }

    if (pcl::console::find_argument (argc, argv, "-r") >= 0)
    {
        bh.isRobot = true;
    }

    if(pcl::console::parse (argc, argv, "-e", bh.backgroundFileName) >= 0)
    {
        if(bh.backgroundFileName.substr(bh.backgroundFileName.find_last_of(".") + 1) != "bag")
        {
            ROS_ERROR("Unable to convert none .bag file");
            printUsage(argv[0]);
            return 0;
        }
    }
    if (pcl::console::find_argument (argc, argv, "-v") >= 0){
        bh.visBool++;
    }

    if(pcl::console::find_argument(argc, argv, "-l") >= 0){
        printf("Connecting to Live camera");
        ros::NodeHandle n1;
        ros::NodeHandle n2;
        Live e(&n1, &n2);
        ros::spin();
    }

    if(pcl::console::parse (argc, argv, "-o", bh.outputBag) >= 0)
    {
        if(bh.outputBag.substr(bh.outputBag.find_last_of(".") + 1) != "bag") {
            bh.outputBag.append(".bag");
        }
    } else {
        printUsage (argv[0]);
        return 0;
    }

    if(pcl::console::parse (argc, argv, "-i", bh.inputFolder) >= 0)
    {
        bh.extract();   // Perform the bag extractions
    }

    if(argc <= 1)
        printUsage (argv[0]);
    return 0;
}

void printUsage(const char* progName)
{
    std::cout << "\nOptions:\n"
    << "--------------------------------------------------\n"
    << "-l use live camera feed\n"
    << "-i Input folder location of \".bag\" files\n"
    << "-o Output folder location to store rosbag features\n"
    << "-h This Help\n"
    << "--------------------------------------------------\n\n";
}
