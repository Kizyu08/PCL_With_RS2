#include "realsense_manager.h"

realsense_manager::realsense_manager()
{
}
realsense_manager::~realsense_manager()
{
    if (sensor_isrunning)stop_sensor();
}

bool realsense_manager::init_sensor()
{
        //Realsense初期化処理
        //const rs2::context ctx;
        //pipeline_ptr = new rs2::pipeline(ctx);
        //profile = pipeline_ptr->start();

        //frames = pipeline_ptr->wait_for_frames();
        //frames = pipe.wait_for_frames();
        //update_frame_once();

        //内部パラメータ取得
        //depth_intr = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();




    return true;
}

void realsense_manager::start_sensor()
{
    sensor_isrunning = true;
    //フレーム更新スレッド作成
    frame_updater_thread = std::thread(std::bind(&realsense_manager::update_frame, this));
}

void realsense_manager::stop_sensor()
{
    sensor_isrunning = false;
    frame_updater_thread.join();
}

void realsense_manager::update_frame()
{
    rs2::pipeline pipe;
    profile = pipe.start();
    depth_intr = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
    queue = rs2::frame_queue(CAPACITY, true);
    auto align = rs2::align(RS2_STREAM_COLOR);

    //1回目
    frames = pipe.wait_for_frames();
    aligned_frames = align.process(frames);
    queue.enqueue(aligned_frames);
    
    pipeline_isinit = true;

    while (pipeline_isinit && sensor_isrunning) {
        //std::lock_guard<std::mutex> lock(mutex);
        //frames = pipeline_ptr->wait_for_frames();
        frames = pipe.wait_for_frames();
        aligned_frames = align.process(frames);

        queue.enqueue(aligned_frames);
    }
}

rs2::frameset realsense_manager::get_frames()
{
    std::lock_guard<std::mutex> lock(mutex);
    return frames;
}

void realsense_manager::get_aligned_RGBimage(cv::Mat& out)
{
    if (pipeline_isinit) {
        rs2::frameset frame;
        frame = queue.wait_for_frame();
        //queue.poll_for_frame(&frame);
        rs2::video_frame vframe = frame.first(RS2_STREAM_COLOR);
        rs2_frame_to_mat(vframe, out);

    }
}

//void realsense_manager::get_aligned_frames(rs2::video_frame* color, rs2::depth_frame* depth)
//{
//    std::lock_guard<std::mutex> lock(mutex);
//    color = aligned_color_frame_ptr;
//    depth = aligned_depth_frame_ptr;
//}

void realsense_manager::rs2_frame_to_mat(
    rs2::frame& src,
    cv::Mat& dst)
{
    //rs2::frame frame4cv = src;
    
    const int w = src.as<rs2::video_frame>().get_width();
    const int h = src.as<rs2::video_frame>().get_height();

    dst = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)src.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(dst, dst, cv::COLOR_BGR2RGB);
}