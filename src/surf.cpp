#include "features.h"
#include "surf.h"

sensor_msgs::Image conversions(cv::Mat mat)
{
    sensor_msgs::Image sensorImg;

    sensorImg.height = mat.rows;
    sensorImg.width = mat.cols;
    sensorImg.encoding = sensor_msgs::image_encodings::BGR8;
    sensorImg.is_bigendian = false;
    sensorImg.step = mat.step;
    size_t size = mat.step * mat.rows;
    sensorImg.data.resize(size);
    memcpy((char*)(&sensorImg.data[0]), mat.data, size);

    return sensorImg;
}

cv::Mat conversions(const sensor_msgs::ImageConstPtr& image)
{
    cv::Mat x;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::BGR8);
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception returning Null image: %s",  e.what());
        return x;
    }
    return cv_ptr->image;
}


surfStruct surf(const sensor_msgs::ImageConstPtr& msg, int hessian)
{
    cv::Mat image(conversions(msg));
    surfStruct surfObj;

    if(msg->encoding != "bgr8" ){
        ROS_ERROR("Unsupported image encoding:");
        return surfObj;  // Return Null image
    }

    cv::Size s = image.size();
    if(s.height < 1)return surfObj;

    cv::SurfFeatureDetector detector(hessian);

    detector.detect(image, surfObj.keypoints);

	//-- Step 2: Calculate descriptors (feature vectors)
	cv::SurfDescriptorExtractor extractor;
    extractor.compute( image, surfObj.keypoints, surfObj.descriptors);
    std::cout << "Surf:" <<surfObj.descriptors.size() << "\n";

    if(s.height > 1)
    {
        cv::Mat imageSurfOverlayed;
     // cv::drawKeypoints(image, surfObj.descriptors, imageSurfOverlayed, cv::Scalar( 0, 0, 255), cv::DrawMatchesFlags::DEFAULT);
     // cv::imshow("Good Matches", imageSurfOverlayed);
    }
    return surfObj;
}




void DMatch(surfStruct s1, cv::Mat i1, surfStruct s2, cv::Mat i2)
{
    cv::Size t1 = i1.size();
    cv::Size t2 = i2.size();

    if(t1.height < 1 && t2.height < 1)return;
    try
    {
        cv::FlannBasedMatcher matcher;
        std::vector< cv::DMatch > matches;
        matcher.match( s1.descriptors, s2.descriptors, matches );

        double max_dist = 0; double min_dist = 0.9;

          //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < s1.descriptors.rows; i++ )
        {
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }

        printf("-- Max dist : %f \n", max_dist );
        printf("-- Min dist : %f \n", min_dist );

        std::vector< cv::DMatch > good_matches;

        for( int i = 0; i < s1.descriptors.rows; i++ )
        { if( matches[i].distance < 2*min_dist )
            {
                good_matches.push_back( matches[i]);
            }
        }

        std::sort(good_matches.begin(), good_matches.end());

        std::vector< cv::DMatch > less;
        for(int i = 0; i < 10; i ++)
        {
            less.push_back(good_matches[i]);
        }

        cv::Mat img_matches;
          cv::drawMatches( i1, s1.keypoints, i2, s2.keypoints,
                   good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                   std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

          cv::imshow("DMatches", img_matches);
          cv::waitKey(10);
          /*
          //-- Localize the object
            std::vector<cv::Point2f> obj;
            std::vector<cv::Point2f> scene;

            for( int i = 0; i < good_matches.size(); i++ )
            {
              //-- Get the keypoints from the good matches
              obj.push_back( s1.keypoints[ good_matches[i].queryIdx ].pt );
              scene.push_back( s2.keypoints[ good_matches[i].trainIdx ].pt );
            }

            cv::Mat H = findHomography( obj, scene, CV_RANSAC );

            //-- Get the corners from the image_1 ( the object to be "detected" )
            std::vector<cv::Point2f> obj_corners(4);
            obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( i1.cols, 0 );
            obj_corners[2] = cvPoint( i1.cols, i1.rows ); obj_corners[3] = cvPoint( 0, i1.rows );
            std::vector<cv::Point2f> scene_corners(4);

            perspectiveTransform( obj_corners, scene_corners, H);

            //-- Draw lines between the corners (the mapped object in the scene - image_2 )
            line( img_matches, scene_corners[0] + cv::Point2f( i1.cols, 0), scene_corners[1] + cv::Point2f( i1.cols, 0), cv::Scalar(0, 255, 0), 4 );
            line( img_matches, scene_corners[1] + cv::Point2f( i1.cols, 0), scene_corners[2] + cv::Point2f( i1.cols, 0), cv::Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2] + cv::Point2f( i1.cols, 0), scene_corners[3] + cv::Point2f( i1.cols, 0), cv::Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3] + cv::Point2f( i1.cols, 0), scene_corners[0] + cv::Point2f( i1.cols, 0), cv::Scalar( 0, 255, 0), 4 );

            //-- Show detected matches
            imshow( "Good Matches & Object detection", img_matches );

            cv::abswaitKey(0);
            */
        /*
        cv::FlannBasedMatcher matcher;
        std::vector< cv::DMatch > matches;
        matcher.match( s1.descriptors, s2.descriptors, matches );

        double max_dist = 50; double min_dist = 100;

        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < s1.descriptors.rows; i++ ){
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
          }

          //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
          //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
          //-- small)
          //-- PS.- radiusMatch can also be used here.
          std::vector< cv::DMatch > good_matches;

          for( int i = 0; i < s1.descriptors.rows; i++ ){
              if( matches[i].distance <= std::max(2*min_dist, 0.005) ){
                  good_matches.push_back( matches[i]);
              }
          }

          //-- Draw only "good" matches
          cv::Mat img_matches;
          drawMatches( i1, s1.keypoints, i2, s2.keypoints,
                       good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                       std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

          //-- Show detected matches
          imshow("DMatched Pictures", img_matches);
          cv::waitKey(100);
*/
    }
    catch (const std::exception &exc)
    {
        // catch anything thrown within try block that derives from std::exception
        std::cerr << exc.what();
    }
}

cv::Mat kmeans(cv::Mat image)
{
    assert(image.type() == CV_32F);
    int clusters = 100;

    cv::BOWKMeansTrainer kmean(clusters);
    kmean.add(image);
    cv::Mat l = kmean.cluster();
    std::cout << std::endl << l.size() << "Postclustered?" << std::endl;

 // perform change detection!

    return l;
}

/*
//
// Dirty Image conversion in Groovy Distro
//
FILE* file = fopen( "temp.ppm", "w" );

fprintf( file, "P3\n" );
fprintf( file, "%i %i\n", msg->width, msg->height );
fprintf( file, "255\n" );

for ( uint32_t y = 0; y < msg->height; y++ )
{
  for ( uint32_t x = 0; x < msg->width; x++ )
    {
        // Get indices for the pixel components
    uint32_t redByteIdx = y*msg->step + 3*x;
    uint32_t greenByteIdx = redByteIdx + 1;
    uint32_t blueByteIdx = redByteIdx + 2;

    fprintf( file, "%i %i %i ",
    msg->data[ redByteIdx ],
    msg->data[ greenByteIdx ],
    msg->data[ blueByteIdx ] );
    }
    fprintf( file, "\n" );
}
cv::Mat image = cv::imread("temp.ppm", CV_LOAD_IMAGE_COLOR);   // Read the file
fclose(file);
*/  // Extracted the Features returning to sensor_msgs::image format

/*
//Dirty Return to SENSOR_
G::IMAGE
ros_image.height = descriptors.rows;
ros_image.width = descriptors.cols;
ros_image.encoding = sensor_msgs::image_encodings::BGR8;
ros_image.is_bigendian = false;
ros_image.step = descriptors.step;
size_t size = descriptors.step * descriptors.rows;
ros_image.data.resize(size);
memcpy((char*)(&ros_image.data[0]), descriptors.data, size);
*/
