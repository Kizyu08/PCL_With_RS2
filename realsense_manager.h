#pragma once
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <thread>
#include <mutex>
#include <stdexcept>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <atomic>

class realsense_manager
{
private:
    std::mutex mutex;

    //Realsense


    const int CAPACITY = 5; // allow max latency of 5 frames

    rs2::frameset frames;
    rs2::frameset aligned_frames;

    rs2::pipeline_profile profile;


    std::thread frame_updater_thread;


    void update_frame();
    rs2::frameset get_frames();

    void rs2_frame_to_mat(rs2::frame& src, cv::Mat& dst);

public:
    std::atomic<bool> pipeline_isinit = false;
    std::atomic<bool> sensor_isrunning = false;
    rs2_intrinsics depth_intr;

    rs2::frame_queue queue;

    realsense_manager();
    ~realsense_manager();
    bool init_sensor();
    void start_sensor();
    void stop_sensor();
    void get_aligned_RGBimage(cv::Mat&);
    //void get_aligned_frames(rs2::video_frame*, rs2::depth_frame*);

};

