#ifndef LIVE_H
#define LIVE_H

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

class Live {

    public:

        Live(ros::NodeHandle*,ros::NodeHandle*);
        void imageCallBack(const sensor_msgs::Image&);
        void depthCallBack(const sensor_msgs::PointCloud2Ptr& msg);
    protected:

        ros::Subscriber sub;
        ros::NodeHandle n1, n2;
        ros::Publisher pub;
};

#endif
