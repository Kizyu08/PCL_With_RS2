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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
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

class myBox
{
public:
    //íÜêS
    float x = 0;
    float y = 0;
    float z = 0;

    //ï”ÇÃí∑Ç≥
    float xLength = 0;
    float yLength = 0;
    float zLength = 0;
};

class points_position_detector
{
private:

    cv_dnn cv_dnn_instance;

    rs2::pipeline* pipeline;

    rs2::points points;
    rs2::pointcloud pointcloud;
    rs2::colorizer color_map;

    rs2::pipeline_profile profile;

    std::mutex imageMutex;
    //std::mutex boxesMutex;

    bool imageAllived = false;
    cv::Mat image;
    cv::Mat image_ext;//åˆäJóp
    cv::Mat editedImage;

    std::vector<myBox> boxes;

    bool run = false;
    std::thread detectorTheread;

    const std::string window_name = "Display Image";

    void rs2_frame_to_mat(rs2::frame& src, cv::Mat& dst);
    void save_image_and_pointclouds(cv::Mat& image, std::vector<pcl_ptr>& clouds);
    void get_pointclouds(
        std::vector<pcl_ptr>& cloud,
        std::vector<cv::Rect>& rects,
        rs2::video_frame& color_frame,
        rs2::depth_frame& depth_frame,
        const rs2_intrinsics* intrinsics
    );
    void get_pointcloud(pcl_ptr, cv::Rect&, rs2::video_frame&, rs2::depth_frame&, const rs2_intrinsics*);
    void Remove_outliers(pcl_ptr cloud, pcl_ptr out);
    void GetBox(pcl_ptr cloud, myBox& box);

public:
    points_position_detector();
    ~points_position_detector();

    void start_detector_thread(std::string&);
    void stop_detector_thread();
    void detector();
    void get_texture(cv::Mat&);
    void get_boxes(std::vector<myBox>&);
};
