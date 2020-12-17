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
#include "rabv.hpp"
#include "convert_util.hpp"

namespace libstest {

    // ビューワー起動時の一回だけ呼ばれる
    void viewerOneOff(pcl::visualization::PCLVisualizer&);

    // ビューワー起動中の毎フレーム実行される
    void viewerPsycho(pcl::visualization::PCLVisualizer&);

    void rs2pcl();

    void rs2pcd(int = 300, int = 300, int = 600, int = 600);

    void rs2pcd_rab();

    int rs2cvSample();

    int rabSample();
}

