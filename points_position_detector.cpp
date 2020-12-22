#include "points_position_detector.h"

#define PPD points_position_detector


PPD::points_position_detector() {
}

void PPD::get_targets_position() {
    cv_dnn cv_dnn_instance;

    rs2::pipeline pipeline;
    rs2::points points;
    rs2::pointcloud pointcloud;
    rs2::colorizer color_map;

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
    rs2::pipeline_profile profile;
    profile = pipeline.start();


    //rs2
    rs2::align align(RS2_STREAM_COLOR);

    //pcl
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PassThrough<PointT> pass;
    
    //opencv
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);


    const auto intrinsics = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
    std::cout << "fx:" << intrinsics.fx << " fy:" << intrinsics.fy << std::endl;

    /*auto sensor = profile.get_device().first<rs2::depth_sensor>();
    auto scale = sensor.get_depth_scale();
    std::cout << "depth scale: " << scale << std::endl;*/


    while (cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
    {
        //
        // Realsense
        //
        auto frames = pipeline.wait_for_frames();

        //位置合わせ
        auto aligned_frames = align.process(frames);
        rs2::video_frame aligned_color_frame = aligned_frames.first(RS2_STREAM_COLOR);
        rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();

        //
        // OpenCV
        //
        //rs2::frame frame4cv = frames.get_depth_frame().apply_filter(color_map);
        rs2::frame frame4cv = aligned_color_frame;

        const int w = frame4cv.as<rs2::video_frame>().get_width();
        const int h = frame4cv.as<rs2::video_frame>().get_height();

        cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)frame4cv.get_data(), cv::Mat::AUTO_STEP);
        cvtColor(image, image, cv::COLOR_BGR2RGB);

        //この辺にyolo

        //dev
        std::vector<cv::Rect> rects;
        //cv::Rect rect(300, 300, 300, 300);
        //rects.push_back(rect);

        cv_dnn_instance.exec(image, image, rects);

        
        //切り出し
        if (rects.size() > 0) {
            std::vector<pcl_ptr> clouds;
            for (cv::Rect& rect : rects) {
                if (rect.x + rect.width < aligned_color_frame.get_width()
                    && rect.y + rect.height < aligned_color_frame.get_height()
                    && rect.x >= 0 
                    && rect.y >= 0) {
                    //std::cerr << "x: " << rect.x << "  y: " << rect.y << "  width: " << rect.width << "  height: " << rect.height <<  std::endl;
                    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    get_pointcloud(cloud, rect, aligned_color_frame, aligned_depth_frame, &intrinsics);
                    std::cerr << "PointCloud cutting has: " << cloud->size() << " data points." << std::endl;
                    clouds.push_back(cloud);

                }
                //std::cerr << "x: " << cloud->points[460800].x << "  y: " << cloud->points[460800].y << "  z: " << cloud->points[460800].z << std::endl;

                //cv::rectangle(image, rect, cv::Scalar(0, 0, 255), 4, cv::LINE_4);
            }
            //
            // PCL
            //

            // Clear the view
            viewer->removeAllShapes();
            viewer->removeAllPointClouds();

            //各点群に対して処理
            for (int i = 0; i < clouds.size(); i++)
            {
                // points = pointcloud.calculate(aligned_depth_frame);
                // auto pcl_points = points_to_pcl(points);
                auto pcl_points = clouds[i];

                // Build a passthrough filter to remove spurious NaNs
                pass.setInputCloud(pcl_points);
                pass.setFilterFieldName("z");
                pass.setFilterLimits(0.00000001, 2.0);
                pass.filter(*cloud_filtered);
                std::cerr << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl;

                // viewerに追加
                viewer->addPointCloud<PointT>(cloud_filtered, "coloud" + i);

                if (GetAsyncKeyState(VK_SPACE) & 1) {
                    // スペースが押されている時の処理
                    // 作成したPointCloudをPCD形式で保存する
                    cout << "savePCDFileASCII" << endl;
                    // テキスト形式で保存する
                    pcl::io::savePCDFileASCII("p_cloud_ascii.pcd", *cloud_filtered);

                    cv::imwrite("image.png", image);
                }

            }

            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "coloud");
            viewer->addCoordinateSystem();
            viewer->spinOnce(10);
        }
        imshow(window_name, image);

    }
}

void PPD::get_pointcloud(
    pcl_ptr cloud, 
    cv::Rect& rect, 
    rs2::video_frame& color_frame, 
    rs2::depth_frame& depth_frame, 
    const rs2_intrinsics* intrinsics
) {
    float pixel[2];
    float point[3];

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
}
