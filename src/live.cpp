#include <iostream>
#include "live.h"
#include "narf.h"
#include "surf.h"

Live::Live(ros::NodeHandle* _n1, ros::NodeHandle* _n2)
{
    printf("asdasda");
    this->n1 = *_n1;
    this->n2 = *_n2;
    this->sub = this->n1.subscribe("/camera/rgb/image_color", 1, &Live::imageCallBack, this);
    this->sub = this->n2.subscribe("/camera/depth_registered/points", 1, &Live::depthCallBack, this);
}


void Live::imageCallBack(const sensor_msgs::Image& msg)
{
    std::cout << "image\n";
    ros::spinOnce();
}
void Live::depthCallBack(const sensor_msgs::PointCloud2Ptr& msg)
{
    std::cout << "depth\n";
    narf(msg);
    ros::spinOnce();
}



/*#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>

class Live {

    public:

        Live(ros::NodeHandle* _n)
        {
            this->n = *_n;
            this->sub = this->n.subscribe("/camera/depth_registered/points", 1, &Live::callBack, this);

        }

        void callBack(const sensor_msgs::PointCloud2Ptr& msg)
        {
            std::cout << msg->width << std::endl;
            this->pub = this->n.advertise<sensor_msgs::PointCloud2>("/camera/Owl/PhilMod", 10);

            this->pub.publish(msg);

            ros::spinOnce();

        }
    protected:

        ros::Subscriber sub;
        ros::NodeHandle n;
        ros::Publisher pub;

};
*/
