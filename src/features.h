#ifndef feature_H
#define feature_H

#include <iostream>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>

// Declaring Header type stuff //
class feature
{
public:
	virtual void extractFeatures(){std::cout << "Feature base class.";}

    virtual void kmeans()
    {
      // perform kmean on feature
    }

    virtual void comparason()
    {

    }

	cv::Mat data;
};

#endif
