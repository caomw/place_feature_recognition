#include "narf.h"

float angular_resolution = pcl::deg2rad(0.5);
float support_size = 0.2f; // 0.1f
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;
bool rotation_invariant = true;
float noise_level = 0.0;
float min_range = 0.0f;
int border_size = 1;
Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());

narfStruct narf(const sensor_msgs::PointCloud2::Ptr p)
{
	//sensor_msgs::PointCloud2::Ptr s = msg.instantiate<sensor_msgs::PointCloud2>();
	pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg (*p, point_cloud);

	pcl::RangeImage range_image;
	point_cloud.width = (int) point_cloud.points.size ();  point_cloud.height = 1;
	range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
				   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

	// Extract NARF keypoints
	pcl::RangeImageBorderExtractor range_image_border_extractor;
	pcl::NarfKeypoint narf_keypoint_detector;
	narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
	narf_keypoint_detector.setRangeImage (&range_image);
	narf_keypoint_detector.getParameters ().support_size = support_size;
		
	pcl::PointCloud<int> keypoint_indices;
	narf_keypoint_detector.compute (keypoint_indices);

	// Extract NARF descriptors for interest points
	std::vector<int> keypoint_indices2;
	keypoint_indices2.resize (keypoint_indices.points.size ());
	for (unsigned int i=0; i<keypoint_indices.size (); ++i){ // This step is necessary to get the right vector type
        keypoint_indices2[i]=keypoint_indices.points[i];
    }

	pcl::NarfDescriptor narf_descriptor (&range_image, &keypoint_indices2);
	narf_descriptor.getParameters ().support_size = support_size;
	narf_descriptor.getParameters ().rotation_invariant = rotation_invariant;
    narfStruct ns;
    //pcl::PointCloud<pcl::Narf36> narf_descriptors;
	
    narf_descriptor.compute (ns.narf_descriptors);
    std::cout << "\nNarf:[36 x " <<ns.narf_descriptors.size ()<< "]\t";

    ns.rangeImg = range_image;

    return ns;
}

void extractPointXYZ(const sensor_msgs::PointCloud2::Ptr p)
{
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg (*p, point_cloud);

}
