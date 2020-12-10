#pragma once
#include <pcl/point_types.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <pcl/point_cloud.h>

namespace convertutil {
    typedef pcl::PointXYZ PointT;
    using pcl_ptr = pcl::PointCloud<PointT>::Ptr;

    pcl_ptr points_to_pcl(const rs2::points&);
}