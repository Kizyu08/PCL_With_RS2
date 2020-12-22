#pragma once
#include <vector>
#include <iostream>
#include <random>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <Windows.h>
#include <opencv2/opencv.hpp>
#include "convert_util.hpp"
#include "cv_dnn.hpp"

typedef pcl::PointXYZ PointT;
using namespace convertutil;
using namespace libstest;

class points_position_detector
{
private:
    //
    //realsense
    //
    
    
    
    

    /*static rs2::decimation_filter dec_filter;
    static rs2::spatial_filter spat_filter;
    static rs2::temporal_filter temp_filter;
    static rs2::hole_filling_filter hf_filter;*/

    //
    //pcl
    //

    //
    //OpenCV
    //
    const std::string window_name = "Display Image";

public:
    points_position_detector();
    void get_targets_position();
    void get_pointcloud(pcl_ptr, cv::Rect&, rs2::video_frame&, rs2::depth_frame&, const rs2_intrinsics*);
};

