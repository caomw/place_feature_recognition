#include "visualise.h"

void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}

void visualisation::visualise(narfStruct x)
{
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
    pcl::RangeImage& range_image = x.rangeImg;

    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
      viewer.setBackgroundColor (1, 1, 1);
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
      viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
      //viewer.addCoordinateSystem (1.0f, "global");
      //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
      //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
      viewer.initCameraParameters ();

    setViewerPose (viewer, range_image.getTransformationToWorldSystem ());

    pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    range_image_widget.showRangeImage (range_image);

    // Sleep
    unsigned int microseconds = 5000000;// set sleep 1000000 = 1second
    usleep(microseconds);               // SLEEP
}

void visualisation::visualise(std::vector<cv::KeyPoint> features,cv::Mat image)
{
    cv::Size s = image.size();
    if(s.height > 1)
    {
      cv::Mat imageSurfOverlayed;
      cv::drawKeypoints(image, features, imageSurfOverlayed, cv::Scalar( 0, 0, 255), cv::DrawMatchesFlags::DEFAULT);
      cv::imshow("Good Matches", imageSurfOverlayed);
    }
}






