#include "points_position_detector.h"

points_position_detector::points_position_detector() 
{
    //unity落ちる
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
}

points_position_detector::~points_position_detector()
{
    //スレッド残留防止
    stop_detector_thread();
}

void points_position_detector::start_detector_thread()
{
    run = true;
    //バインドを作らなきゃならないらしい
    //インスタンスよりスレッドが長生きしないよう注意
    detectorTheread = std::thread(std::bind(&points_position_detector::detector, this));


}

void points_position_detector::stop_detector_thread()
{
    if (run) {
        run = false;
        //スレッド終了待ち
        detectorTheread.join();
    }
    cv::destroyAllWindows();
}

void points_position_detector::detector() 
{
    const rs2::context ctx;
    pipeline = new rs2::pipeline(ctx);
    profile = pipeline->start();

    //YOLO

    rs2::align align(RS2_STREAM_COLOR);

    //realsense
    /*dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3);
    spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    spat_filter.set_option(RS2_OPTION_HOLES_FILL, 0);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    temp_filter.set_option(RS2_OPTION_HOLES_FILL, 3);
    hf_filter.set_option(RS2_OPTION_HOLES_FILL, 1);*/

    //strat rs pipeline

    //pcl
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PassThrough<PointT> pass;
    
    //opencv


    const auto intrinsics = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
    std::cout << "fx:" << intrinsics.fx << " fy:" << intrinsics.fy << std::endl;



    //while (cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
    while (run)
    {   
        {
            std::lock_guard<std::mutex> lock(imageMutex);
            boxes.clear();
        }
        //
        // Realsense
        //
        auto frames = pipeline->wait_for_frames();

        //位置合わせ
        auto aligned_frames = align.process(frames);
        rs2::video_frame aligned_color_frame = aligned_frames.first(RS2_STREAM_COLOR);
        rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();

        //
        // OpenCV
        //
        {
            std::lock_guard<std::mutex> lock(imageMutex);
            rs2_frame_to_mat(aligned_color_frame, image);
        }
        imshow(window_name, image);


        //dev
        std::vector<cv::Rect> rects;

        // YOLO
        std::vector<int> classIds;
        cv_dnn_instance.exec(image, editedImage, rects, classIds, imageMutex);

        //切り出し
        if (rects.size() > 0) {
            std::vector<pcl_ptr> clouds; 
            points_position_detector::get_pointclouds(clouds, rects, boxes, aligned_color_frame, aligned_depth_frame, &intrinsics);

            //
            // PCL
            //

            std::vector<pcl_ptr> clouds_filterd;

            //各点群に対して処理
            for (int i = 0; i < clouds.size(); i++)
            {
                std::cout << i << std::endl;
                if (classIds[i] == 39 || classIds[i] == 40 || classIds[i] == 41 || classIds[i] == 64) {
                    std::cerr << "PointCloud before filtering has: " << clouds[i]->size() << " data points." << std::endl;
                    // Clear the view
                    viewer->removeAllShapes();
                    viewer->removeAllPointClouds();

                    // Build a passthrough filter to remove spurious NaNs
                    pass.setInputCloud(clouds[i]);
                    pass.setFilterFieldName("z");
                    pass.setFilterLimits(0.00000001, 2.0);
                    pass.filter(*cloud_filtered);
                    std::cerr << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl;

                    clouds_filterd.push_back(cloud_filtered);

                    // viewerに追加
                    viewer->addPointCloud<PointT>(cloud_filtered, "coloud" + i);
                }
            }

            
            if (clouds_filterd.size() > 0) {
                if (GetAsyncKeyState(VK_SPACE) & 1) {
                    // スペースが押されている時の処理
                    std::lock_guard<std::mutex> lock(imageMutex);
                    save_image_and_pointclouds(image, clouds_filterd);
                }

                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "coloud");
                viewer->addCoordinateSystem();
                viewer->spinOnce(10);
            }
        }
    }
}

void points_position_detector::get_texture(cv::Mat& out)
{
    cv::Mat src;
    {
        std::lock_guard<std::mutex> lock(imageMutex);
        out = image.clone();
    }
    if (out.cols > 0) {
        cv::cvtColor(out, out, cv::COLOR_BGR2RGBA);
    }
}

void points_position_detector::get_boxes(std::vector<myBox>& boxes)
{
    std::lock_guard<std::mutex> lock(imageMutex);
    boxes = this->boxes;
}


void points_position_detector::rs2_frame_to_mat(
    rs2::frame& src, 
    cv::Mat& dst)
{
    rs2::frame frame4cv = src;

    const int w = frame4cv.as<rs2::video_frame>().get_width();
    const int h = frame4cv.as<rs2::video_frame>().get_height();

    dst = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)frame4cv.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(dst, dst, cv::COLOR_BGR2RGB);
}

void points_position_detector::save_image_and_pointclouds(
    cv::Mat& image, 
    std::vector<pcl_ptr>& clouds)
{
    // 作成したPointCloudをPCD形式で保存する
    cout << "savePCDFileASCII" << endl;
    
    // テキスト形式で保存する
    for (int i = 0; i < clouds.size(); i++) {
        pcl::io::savePCDFileASCII("p_cloud_ascii_" + std::to_string(i) + ".pcd", *clouds[i]);
    }

    // 画像保存
    cv::imwrite("image.png", image);
}

/// <summary>
/// 
/// </summary>
/// <param name="clouds">出力</param>
/// <param name="rects">画像上の枠</param>
/// <param name="aligned_color_frame">rsのvideo_frame</param>
/// <param name="aligned_depth_frame">rsのdepth_frame</param>
/// <param name="intrinsics"></param>
void points_position_detector::get_pointclouds(
    std::vector<pcl_ptr>& clouds,
    std::vector<cv::Rect>& rects,
    std::vector<myBox>& boxes,
    rs2::video_frame& aligned_color_frame,
    rs2::depth_frame& aligned_depth_frame,
    const rs2_intrinsics* intrinsics)
{
    int frameWidth = aligned_color_frame.get_width();
    int frameHeight = aligned_color_frame.get_height();

    for (cv::Rect& rect : rects) {
        if (rect.x + rect.width > frameWidth)
            rect.width = frameWidth - rect.x;
        if (rect.y + rect.height > frameHeight)
            rect.height = frameHeight - rect.y;
        if (rect.x < 0)
            rect.x = 0;
        if (rect.y < 0)
            rect.y = 0;

        pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        myBox box;
        get_pointcloud(cloud, rect, box, aligned_color_frame, aligned_depth_frame, intrinsics);
        std::cerr << "PointCloud cutting has: " << cloud->size() << " data points." << std::endl;
        clouds.push_back(cloud);        
        {
            std::lock_guard<std::mutex> lock(imageMutex);
            boxes.push_back(box);
        }
    }
}

void points_position_detector::get_pointcloud(
    pcl_ptr cloud,
    cv::Rect& rect,
    myBox& box,
    rs2::video_frame& color_frame,
    rs2::depth_frame& depth_frame,
    const rs2_intrinsics* intrinsics
) {
    float pixel[2];
    float point[3];

    pcl::PointXYZ min, max;

    cloud->is_dense = false;

    // vertical
    for (int i = rect.y; i < rect.y + rect.height; i++) {
        // horizontal
        for (int j = rect.x; j < rect.x + rect.width; j++) {
            pixel[0] = j;
            pixel[1] = i;

            //画像の位置→3D点
            auto depth = depth_frame.get_distance(j, i);
            rs2_deproject_pixel_to_point(point, intrinsics, pixel, depth);

            pcl::PointXYZ p(point[0], point[1], point[2]);
            cloud->points.push_back(p);
            

            // std::cerr << "x: " << p.x << "  y: " << p.y << "  z: " << p.z << std::endl;
        }
    }

    //pclのminMax3Dが何故か使えない&遅いらしいのでベタ
    float min_x = cloud->points[0].x, 
        min_y = cloud->points[0].y, 
        min_z = cloud->points[0].z, 
        max_x = cloud->points[0].x, 
        max_y = cloud->points[0].y, 
        max_z = cloud->points[0].z;
    for (size_t i = 1; i < cloud->points.size(); ++i) {
        if (cloud->points[i].x <= min_x)
            min_x = cloud->points[i].x;
        else if (cloud->points[i].y <= min_y)
            min_y = cloud->points[i].y;
        else if (cloud->points[i].z <= min_z)
            min_z = cloud->points[i].z;
        else if (cloud->points[i].x >= max_x)
            max_x = cloud->points[i].x;
        else if (cloud->points[i].y >= max_y)
            max_y = cloud->points[i].y;
        else if (cloud->points[i].z >= max_z)
            max_z = cloud->points[i].z;
    }

    box.xLength = max_x - min_x;
    box.yLength = max_y - min_y;
    box.zLength = max_z - min_z;
    box.x = min_x + (box.xLength / 2);
    box.y = min_y + (box.yLength / 2);
    box.z = min_z + (box.zLength / 2);
    std::cout << "box x: " << box.x << "  y: " << box.y << "  z: " << box.z << std::endl;

}
