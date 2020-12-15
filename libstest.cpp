#include "libstest.hpp"

namespace libstest {
    typedef pcl::PointXYZ PointT;
    using pcl_ptr = pcl::PointCloud<PointT>::Ptr;
    using namespace convertutil;
    //int user_data;

    //// ビューワー起動時の一回だけ呼ばれる
    //void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
    //{
    //    viewer.setBackgroundColor(1.0, 0.5, 1.0);
    //    pcl::PointXYZ o;
    //    o.x = 1.0;
    //    o.y = 0;
    //    o.z = 0;
    //    viewer.addSphere(o, 0.25, "sphere", 0);
    //    std::cout << "i only run once" << std::endl;
    //}

    //// ビューワー起動中の毎フレーム実行される
    //void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
    //{
    //    static unsigned count = 0;
    //    std::stringstream ss;
    //    ss << "Once per viewer loop: " << count++;
    //    viewer.removeShape("text", 0);
    //    viewer.addText(ss.str(), 200, 300, "text", 0);
    //    user_data++;
    //}

    cv::Rect box;
    bool drawing_box = false;
    bool complete_region = false;

    /*************************************************
     * void draw_box(Mat* img, Rect rect)
     *
     * Mat* img : 矩形を描く画像
     * Rect rect : 矩形の座標(x, y, width, height)
     *
     * 機能 : 画像に矩形を描く
     *
     * return :　なし
     *************************************************/
    void draw_box(cv::Mat* img, cv::Rect rect) {
        rectangle(*img, cv::Point2d(box.x, box.y),
            cv::Point2d(box.x + box.width, box.y + box.height),
            cv::Scalar(255, 255, 255));
    }

    /*************************************************
     * void my_mouse_callback(int event, int x, int y, int flags, void* param)
     *
     * int event : マウスイベントの種類
     * int x : クリックし始め座標
     * int y : クリックし始め座標
     * int flags :
     *
     * 機能 : マウスイベントコールバック関数
     *
     * return :　なし
     *************************************************/
    void my_mouse_callback(int event, int x, int y, int flags, void* param) {
        cv::Mat* image = static_cast<cv::Mat*>(param);
        switch (event) {
        case cv::EVENT_MOUSEMOVE:       // マウス動作検知
            if (drawing_box) {         // 左クリック押されている時
                box.width = x - box.x;  // 選択範囲の横幅を計算
                box.height = y - box.y; // 選択範囲の高さを計算
            }
            break;

        case cv::EVENT_LBUTTONDOWN:     // マウス左クリック押されたら
            drawing_box = true;       // ボタンが押されたら、選択範囲の横幅と高さを計算するif文に入る
            box = cv::Rect(x, y, 0, 0);   // クリックされ始めの場所を選択範囲の左上座標として保存
            break;

        case cv::EVENT_LBUTTONUP:       // マウス左クリック離されたら
            drawing_box = false;      // 選択範囲の横幅と高さを計算するif文から抜ける
            complete_region = true;   // 範囲選択を完了

            if (box.width < 0) {       // クリック終わりの座標がクリックし始めた場所より左にある場合
                box.x += box.width;
                box.width *= -1;
            }

            if (box.height < 0) {      // クリックが終わりの座標がクリックし始めた場所より上にある場合
                box.y += box.height;
                box.height *= -1;
            }

            draw_box(image, box);     // 矩形を描く関数へ
            break;
        }
    }

    void rs2pcl()
    {
        //realsense
        rs2::pipeline pipeline;
        rs2::points points;
        rs2::pointcloud pointcloud;
        pipeline.start();

        // All the objects needed
        pcl::PCDReader reader;
        pcl::PassThrough<PointT> pass;
        pcl::VoxelGrid<PointT> voxel_filter;
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        pcl::PCDWriter writer;
        pcl::ExtractIndices<PointT> extract;
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

        // Datasets
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_voxel_filtered(new pcl::PointCloud<PointT>());
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
        pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
        pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
        pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);


        pcl::visualization::CloudViewer viewer("Cloud Viewer");
        //メインループ
        while (!viewer.wasStopped())
        {
            auto frames = pipeline.wait_for_frames();
            auto depths = frames.get_depth_frame();
            auto rgb = frames.get_color_frame();

            points = pointcloud.calculate(depths);
            auto pcl_points = points_to_pcl(points);
            //viewer.showCloud(pcl_points);

            cloud = pcl_points;
            //viewer.showCloud(cloud);
            // Read in the cloud data
            //reader.read("table_scene_mug_stereo_textured.pcd", *cloud);
            std::cerr << "PointCloud has: " << cloud->size() << " data points." << std::endl;


            // Build a passthrough filter to remove spurious NaNs
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0, 1.5);
            pass.filter(*cloud_filtered);
            std::cerr << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl;

            //ボクセルフィルタ
            voxel_filter.setInputCloud(cloud_filtered);
            {
                float leafsize = 0.05f;
                voxel_filter.setLeafSize(leafsize, leafsize, leafsize);
                voxel_filter.filter(*cloud_voxel_filtered);
            }
            if (cloud_voxel_filtered->size() > 100000)continue;
            std::cerr << "PointCloud after voxel filtering has: " << cloud_voxel_filtered->size() << " data points." << std::endl;
            viewer.showCloud(cloud_voxel_filtered);

            //法線計算
            ne.setSearchMethod(tree);
            ne.setInputCloud(cloud_voxel_filtered);
            ne.setKSearch(50);
            ne.compute(*cloud_normals);

            // Create the segmentation object for the planar model and set all the parameters
            seg.setInputCloud(cloud_voxel_filtered);
            {
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
                seg.setNormalDistanceWeight(0.1);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setMaxIterations(100);
                seg.setDistanceThreshold(0.03);
                seg.setInputNormals(cloud_normals);
            }
            // Obtain the plane inliers and coefficients
            seg.segment(*inliers_plane, *coefficients_plane);
            std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

            // Extract the planar inliers from the input cloud
            extract.setInputCloud(cloud_voxel_filtered);
            extract.setIndices(inliers_plane);
            extract.setNegative(false);

            // Write the planar inliers to disk
            pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
            extract.filter(*cloud_plane);
            std::cerr << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;
            //writer.write("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

            // Remove the planar inliers, extract the rest
            {
                extract.setNegative(true);
                extract.filter(*cloud_filtered2);
                extract_normals.setNegative(true);
                extract_normals.setInputCloud(cloud_normals);
                extract_normals.setIndices(inliers_plane);
                extract_normals.filter(*cloud_normals2);

            }

            // Create the segmentation object for cylinder segmentation and set all the parameters
            {
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_CYLINDER);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setNormalDistanceWeight(0.1);
                seg.setMaxIterations(10000);
                seg.setDistanceThreshold(0.05);
                seg.setRadiusLimits(0, 0.1);
                seg.setInputCloud(cloud_filtered2);
                seg.setInputNormals(cloud_normals2);
            }

            // Obtain the cylinder inliers and coefficients
            seg.segment(*inliers_cylinder, *coefficients_cylinder);
            //std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

            // Write the cylinder inliers to disk
            extract.setInputCloud(cloud_filtered2);
            extract.setIndices(inliers_cylinder);
            extract.setNegative(false);
            pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
            extract.filter(*cloud_cylinder);
            if (cloud_cylinder->points.empty())
                std::cerr << "Can't find the cylindrical component." << std::endl;
            else
            {
                //viewer.showCloud(cloud_cylinder);

                std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->size() << " data points." << std::endl;
                //writer.write("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
            }

        }
    }

    void rs2pcd() {

        //realsense
        rs2::pipeline pipeline;
        rs2::points points;
        rs2::pointcloud pointcloud;
        rs2::colorizer color_map;
        rs2::align align(RS2_STREAM_COLOR);

        rs2::decimation_filter dec_filter;
        dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3);
        rs2::disparity_transform depth_to_disparity(true);
        rs2::disparity_transform disparity_to_depth(false);

        rs2::spatial_filter spat_filter;
        spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
        spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);
        spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
        spat_filter.set_option(RS2_OPTION_HOLES_FILL, 0);

        rs2::temporal_filter temp_filter;
        temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4);
        temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
        temp_filter.set_option(RS2_OPTION_HOLES_FILL, 3);

        rs2::hole_filling_filter hf_filter;
        hf_filter.set_option(RS2_OPTION_HOLES_FILL, 1);


        //pcl
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        pcl::PassThrough<PointT> pass;
        pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

        //opencv
        const auto window_name = "Display Image";
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);


        //strat
        rs2::pipeline_profile profile = pipeline.start();
        
        auto intrinsics = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
        std::cout << "fx:" << intrinsics.fx << " fy:" << intrinsics.fy << std::endl;

        auto sensor = profile.get_device().first<rs2::depth_sensor>();
        auto scale = sensor.get_depth_scale();
        std::cout << "depth scale: " << scale << std::endl;
        

        while (cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
        {
            auto frames = pipeline.wait_for_frames();
            auto depths = frames.get_depth_frame();
            auto rgb = frames.get_color_frame();

            //位置合わせ
            auto aligned_frames = align.process(frames);
            rs2::video_frame aligned_color_frame = aligned_frames.first(RS2_STREAM_COLOR);
            rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();

            //pcl
            // Clear the view
            viewer->removeAllShapes();
            viewer->removeAllPointClouds();

            points = pointcloud.calculate(aligned_depth_frame);
            auto pcl_points = points_to_pcl(points);


            // Build a passthrough filter to remove spurious NaNs
            pass.setInputCloud(pcl_points);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0.00000001, 1.5);
            pass.filter(*cloud_filtered);
            std::cerr << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl;

            // The pointcloud
            viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "coloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "coloud");
            viewer->addCoordinateSystem();
            viewer->spinOnce(10);

            if (GetAsyncKeyState(VK_SPACE) & 1) {
                // スペースが押されている時の処理
                // 作成したPointCloudをPCD形式で保存する
                cout << "savePCDFileASCII" << endl;
                pcl::io::savePCDFileASCII("p_cloud_ascii.pcd", *cloud_filtered); // テキスト形式で保存する
            }


            //opencv
            {
                using namespace cv;
                //rs2::frame frame4cv = frames.get_depth_frame().apply_filter(color_map);
                rs2::frame frame4cv = rgb;

                const int w = frame4cv.as<rs2::video_frame>().get_width();
                const int h = frame4cv.as<rs2::video_frame>().get_height();

                Mat image(Size(w, h), CV_8UC3, (void*)frame4cv.get_data(), Mat::AUTO_STEP);

                imshow(window_name, image);
            }
        }
    }

    void rs2pcd_rab() {

        ////realsense
        rs2::pipeline pipeline;
        rs2::points rs2_points;
        rs2::pointcloud pointcloud;
        rs2::colorizer color_map;

        //pcl
        //pcl::visualization::CloudViewer viewer("Cloud Viewer");
        pcl::PassThrough<PointT> pass;
        pcl_ptr pcl_points(new pcl::PointCloud<PointT>);
        pcl_ptr cloud_filtered(new pcl::PointCloud<PointT>);

        // Make an instance of rabv::Rab
        auto rab = rabv::Rab::create();

        // Add a pointcloud
        rab->addCloud(
            "sample1",   // Unique name of the pointcloud
            cloud_filtered // Pointcloud
        ); 

        rab->addCoordinateSystem(
            "world", // 座標系名，"world"を指定するとワールド座標系
            0.3      // 矢印のスケール
        );

        // Visualze the rab data
        const auto& viewer1 = rabv::Viewer::create("Viewer1", rab);


        //opencv
        const auto window_name = "Display Image";
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

        pipeline.start();

        while (!viewer1->wasStopped())
        // while (!viewer1->wasStopped() &&cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
        {
            auto frames = pipeline.wait_for_frames();
            auto depths = frames.get_depth_frame();
            auto rgb = frames.get_color_frame();

            ////pcl

            rs2_points = pointcloud.calculate(depths);
            pcl_points = points_to_pcl(rs2_points);

            // Build a passthrough filter to remove spurious NaNs
            pass.setInputCloud(pcl_points);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0.00000001, 1.5);
            pass.filter(*cloud_filtered);
            std::cerr << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl;

            //viewer.showCloud(cloud_filtered);

            if (GetAsyncKeyState(VK_SPACE) & 1) {
                // スペースが押されている時の処理
                // 作成したPointCloudをPCD形式で保存する
                cout << "savePCDFileASCII" << endl;
                pcl::io::savePCDFileASCII("p_cloud_ascii.pcd", *cloud_filtered); // テキスト形式で保存する
            }
            rab->removeCloud("sample1");
            rab->addCloud(
                "sample1",   // Unique name of the pointcloud
                cloud_filtered // Pointcloud
            );
            viewer1->setRab(rab);
            viewer1->spin();

            //opencv
            {
                using namespace cv;
                rs2::frame frame4cv = frames.get_depth_frame().apply_filter(color_map);
                //rs2::frame frame4cv = rgb;

                const int w = frame4cv.as<rs2::video_frame>().get_width();
                const int h = frame4cv.as<rs2::video_frame>().get_height();

                Mat image(Size(w, h), CV_8UC3, (void*)frame4cv.get_data(), Mat::AUTO_STEP);

                imshow(window_name, image);
                // コールバック関数宣言
                //cv::setMouseCallback(window_name, my_mouse_callback, (void*)&image);

            }
        }
    }

    int rs2cvSample() try
    {
        // Declare depth colorizer for pretty visualization of depth data
        rs2::colorizer color_map;

        // Declare RealSense pipeline, encapsulating the actual device and sensors
        rs2::pipeline pipe;
        // Start streaming with default recommended configuration
        pipe.start();

        using namespace cv;
        const auto window_name = "Display Image";
        namedWindow(window_name, WINDOW_AUTOSIZE);

        while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
        {
            rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
            rs2::frame depth = data.get_depth_frame().apply_filter(color_map);

            // Query frame size (width and height)
            const int w = depth.as<rs2::video_frame>().get_width();
            const int h = depth.as<rs2::video_frame>().get_height();

            // Create OpenCV matrix of size (w,h) from the colorized depth data
            Mat image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);

            // Update the window with new data
            imshow(window_name, image);
        }

        return EXIT_SUCCESS;
    }
    catch (const rs2::error& e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    int rabSample()
    {
        //
        // Generate sample data
        //

        const int sample_point_num = 100;
        const int sample_corr_num = 5;

        std::mt19937 mt(20150403);
        std::normal_distribution<float> nd(0.0, 0.2);
        std::uniform_int_distribution<int> ud(0, sample_point_num);

        pcl::PointCloud<pcl::PointXYZ>::Ptr sample_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (int i = 0; i < sample_point_num; ++i)
        {
            sample_cloud->push_back(pcl::PointXYZ(nd(mt), nd(mt), nd(mt)));
        }

        //pcl::PointCloud<pcl::PointNormal>::Ptr sample_point_normal(new pcl::PointCloud<pcl::PointNormal>());
        //pcl::PointCloud<pcl::Normal>::Ptr sample_normal(new pcl::PointCloud<pcl::Normal>());
        //for (int i = 0; i < sample_point_num; ++i)
        //{
        //    pcl::PointNormal pn;
        //    pn.x = nd(mt);
        //    pn.y = nd(mt);
        //    pn.z = nd(mt);
        //    const auto normal = Eigen::Vector3f(nd(mt), nd(mt), nd(mt)).normalized();
        //    pn.normal_x = normal.x();
        //    pn.normal_y = normal.y();
        //    pn.normal_z = normal.z();
        //    sample_normal->push_back(pcl::Normal(normal.x(), normal.y(), normal.z()));
        //    sample_point_normal->push_back(pn);
        //}

        //pcl::Correspondences corrs1;
        //pcl::CorrespondencesPtr corrs2(new pcl::Correspondences());
        //std::vector<std::pair<int, int>> corrs3;
        //for (int i = 0; i < sample_corr_num; i++)
        //{
        //    int idx = ud(mt);
        //    pcl::Correspondence corr1(idx, idx, 2.0);
        //    corrs1.push_back(corr1);
        //    idx = ud(mt);
        //    pcl::Correspondence corr2(idx, idx, 2.0);
        //    corrs2->push_back(corr2);
        //    idx = ud(mt);
        //    corrs3.push_back(std::make_pair(idx, idx));
        //}



        //
        // Rab Class Usage
        //

        // 1. Make an instance of rabv::Rab
        auto rab = rabv::Rab::create();


        // 2.1. Add a pointcloud
        rab->addCloud(
            "sample1",   // Unique name of the pointcloud
            sample_cloud // Pointcloud
        );

        //rab->addCloud(
        //    "sample2",                    // Unique name
        //    sample_cloud,                 // Pointcloud
        //    1,                            // Point size
        //    rabv::Color(255, 255, 128),   // Color (r, g, b)
        //    rabv::Point(2.0, 0.0, 0.0),   // Offset of tlanslation (x, y, z)
        //    rabv::Rotation(0.5, 0.0, 0.0) // Rotation (X-axis, Y-axis, Z-axis)
        //);

        //// 2.2. Add a pointcloud and normal together
        //rab->addCloudNormal(
        //    "sample3",                  // Unique name
        //    sample_point_normal,        // Pointcloud and normals
        //    1,                          // Point size
        //    rabv::Color(255, 0, 128),   // Color (r, g, b)
        //    rabv::Point(3.0, 0.0, 0.0), // Offset of tlanslation (x, y, z)
        //    rabv::Rotation(),           // Rotation (none)
        //    1,                          // Density of normals
        //    0.05,                       // Length of normals
        //    1,                          // Width of normals
        //    rabv::Color(0, 255, 255)    // Color of normals
        //);

        //// 2.3. Add a normal after "addPointcloud"
        //rab->addNormal(
        //    "sample1",                  // Unique name of the corresponding pointcloud
        //    sample_normal,              // Pointcloud
        //    1,                          // Density of normals
        //    0.05,                       // Length of normals
        //    2,                          // Width of normals
        //    rabv::Color(255, 0, 128)    // Color of normals
        //);


        // 3.1. Add a coordinate system as world coordinate system
        rab->addCoordinateSystem(
            "world", // Unique name of the pointcloud or "world"
            0.3      // Scale factor
        );

        //// 3.2. Add a coordinate system as pointcloud coordinate system
        //rab->addCoordinateSystem(
        //    "sample2", // Unique name of the pointcloud or "world"
        //    0.2        // Scale factor
        //);


        //// 4. Add correspondence lines between two pointclouds
        //rab->addCorrespondence(
        //    "corr1",   // Unique name of the correspondence
        //    "sample1", // Unique name of the pointcloud (from)
        //    "sample2", // Unique name of the pointcloud (to)
        //    corrs1     // Correspondence (pcl::Correspondences)
        //);

        //rab->addCorrespondence(
        //    "corr2",                   // Unique name of the correspondence
        //    "sample1",                 // Unique name of the pointcloud (from)
        //    "sample2",                 // Unique name of the pointcloud (to)
        //    corrs2,                    // Correspondence (pcl::CorrespondencesPtr)
        //    2,                         // Line width
        //    rabv::Color(255, 255, 128) // Color (r, g, b)
        //);

        //rab->addCorrespondence(
        //    "corr3",                   // Unique name of the correspondence
        //    "sample1",                 // Unique name of the pointcloud (from)
        //    "sample2",                 // Unique name of the pointcloud (to)
        //    corrs3,                    // Correspondence (std::vector<std::pair<int, int>>)
        //    3,                         // Line width
        //    rabv::Color(255, 128, 128) // Color (r, g, b)
        //);


        //// 5.1. Add lines (Make rabv::Lines first)
        //rabv::Lines line1;

        //rabv::Lines line2(
        //    3.0,                       // Line width
        //    rabv::Color(128, 128, 255) // Color (r, g, b)
        //);

        //line1.addLine(
        //    rabv::Point(1.0, 0.0, 0.0), // Point (x, y, z) for "from"
        //    rabv::Point(1.0, 0.0, 1.0)  // Point (x, y, z) for "to"
        //);

        //line2.addLine(
        //    pcl::PointXYZ(1.0, 1.0, 1.0), // Point (x, y, z) for "from"
        //    pcl::PointXYZ(1.0, 1.0, 0.0)  // Point (x, y, z) for "to"
        //);

        //line2.addLine(rabv::Line(
        //    { 1.0, 1.0, 0.0 },  // Point (x, y, z) for "from"
        //    { 1.0, 0.0, 0.0 }   // Point (x, y, z) for "to"
        //));

        //rab->addLines(
        //    "line1", // Unique name of the lines
        //    line1    // rabv::Lines
        //);
        //rab->addLines(
        //    "line2", // Unique name of the lines
        //    line2    // rabv::Lines
        //);

        //// 5.2 Add lines (into rab, directly)
        //rab->addLine(
        //    "line3",                    // Unique name of the lines
        //    rabv::Point(0.0, 0.0, 0.0), // Point (x, y, z) for "from"
        //    rabv::Point(2.0, 0.0, 0.0)  // Point (x, y, z) for "to"
        //);

        //rab->addLine(
        //    "line3",                      // Unique name of the lines
        //    pcl::PointXYZ(2.0, 1.0, 0.0), // Point (x, y, z) for "from"
        //    pcl::PointXYZ(0.0, 1.0, 0.0)  // Point (x, y, z) for "to"
        //);


        //// 6.1 Add 2D text
        //rab->addText(
        //    "Text1"	// Text
        //);

        //rab->addText(
        //    "Text2",               // Text
        //    300, 0,                // Position (x, y)
        //    30,                    // Font size
        //    rabv::Color(0, 255, 0) // Color (r, g, b)
        //);

        //// 6.2 Add 3D text (can see them always)
        //rab->addText3D(
        //    "Text3" // Text
        //);
        //rab->addText3D(
        //    "Text4",                    // Text
        //    rabv::Point(2.0, 0.5, 0.1), // Position (x, y, z)
        //    0.1,                        // Font size
        //    rabv::Color(0, 255, 0)      // Color (r, g, b)
        //);

        //// 6.3 Add Flat 3D text (can see them from only several viewpoint)
        //rab->addFlatText3D(
        //    "Text5",                   // Text
        //    rabv::Point(1.0, 0.5, 0.0) // Position (x, y, z)
        //);
        //rab->addFlatText3D(
        //    "Text6",                      // Text
        //    rabv::Point(1.0, 0.5, 0.1),   // Position (x, y, z)
        //    0.1,                          // Font size
        //    rabv::Color(0, 255, 0),       // Color (r, g, b)
        //    rabv::Rotation(0.5, 0.0, 0.0) // Rotation (X-axis, Y-axis, Z-axis)
        //);


        //// 7. Add a cube
        //rab->addCube("cube1", rabv::Point(-1.0, -1.0, -1.0), rabv::Point(-0.5, -0.5, -0.5));

        //// 8. Generate good colors to visualize
        //const auto& colors = rabv::Color::divideColors(3 /*The number of color*/);

        //rab->addText("Color1", 0, 50, 30, colors[0]);
        //rab->addText("Color2", 100, 50, 30, colors[1]);
        //rab->addText("Color3", 200, 50, 30, colors[2]);



        //
        // Viewer Class Usage
        //

        // 9. Visualze the rab data while the window is closed
        const auto& viewer1 = rabv::Viewer::create(
            "Viewer1",	// Title of viewer
            rab			// Rab data
        );

        while (!viewer1->wasStopped()) {
            viewer1->spin();
        }

        return 0;
    }
}

