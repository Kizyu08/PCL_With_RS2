#pragma once
#include <pcl/point_types.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>

namespace convertutil {
    typedef pcl::PointXYZ PointT;
    using pcl_ptr = pcl::PointCloud<PointT>::Ptr;

    pcl_ptr points_to_pcl(const rs2::points&);
    cv::Mat frame_to_mat(const rs2::frame&);
}