#include "points_position_detector.h"

points_position_detector::points_position_detector() 
{
    //unity������
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
}

points_position_detector::~points_position_detector()
{
    //�X���b�h�c���h�~
    stop_detector_thread();

    rs_man.stop_sensor();
}

void points_position_detector::start_detector_thread(std::string& path)
{
    run = true;
    //ini�ǂݍ���
    cv_dnn_instance.setConfig(path);
    
    rs_man.init_sensor();
    rs_man.start_sensor();

    //�o�C���h�����Ȃ���Ȃ�Ȃ��炵��
    //�C���X�^���X���X���b�h�����������Ȃ��悤����
    detector_theread = std::thread(std::bind(&points_position_detector::detector, this));
}

void points_position_detector::stop_detector_thread()
{
    if (run) {
        run = false;
        //�X���b�h�I���҂�
        detector_theread.join();
    }
    cv::destroyAllWindows();

    rs_man.stop_sensor();
}

/// <summary>
/// ���̌��o�񂷃}��
/// </summary>
void points_position_detector::detector() 
{

    while (!rs_man.pipeline_isinit) {}


    //PCL
    //����
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    
    
    //�œ_����px
    //std::cout << "fx:" << depth_intr.fx << " fy:" << depth_intr.fy << std::endl;

    //FOV�v�Z�i�s�v�j
    //auto wfov = 2 * std::atan(depth_intr.width / (2 * depth_intr.fx));
    //auto hfov = 2 * std::atan(depth_intr.height / (2 * depth_intr.fy));
    //std::cout << "W_FOV: " << wfov << "  H_FOV: " << hfov << std::endl;


    const auto depth_intr = rs_man.depth_intr;

    //���C�����[�v
    while (run)
    {   
        {
            std::lock_guard<std::mutex> lock(boxes_mutex);
            boxes.clear();
        }

        //
        // Realsense
        //
        rs2::frameset frame = rs_man.queue.wait_for_frame();
        rs2::video_frame aligned_color_frame = frame.first(RS2_STREAM_COLOR);
        rs2::depth_frame aligned_depth_frame = frame.get_depth_frame();
        //�ʒu���킹

        //��v���Ă���ۏ؂͂Ȃ�
        //�܂Ƃ߂������悢�H
        rs_man.get_aligned_RGBimage(image);

        //
        // OpenCV
        //

        //����

        std::vector<cv::Rect> rects;

        // YOLO
        std::vector<int> classIds;
        cv_dnn_instance.exec(image, editedImage, rects, classIds);
        //image_ext = image.clone();

        //imshow(window_name, editedImage);

        //�؂�o��
        if (rects.size() > 0) {
            std::vector<pcl_ptr> clouds; 


            //yolo�̌���rects�͈͓̔��̓_�Q��[�x�摜���狁�߂�
            points_position_detector::get_pointclouds(clouds, rects, aligned_color_frame, aligned_depth_frame, &depth_intr);

            //
            // PCL
            //

            std::vector<pcl_ptr> clouds_filterd;

            // Clear the view
            viewer->removeAllShapes();
            viewer->removeAllPointClouds();

            //�e�_�Q�ɑ΂��ď���
            for (int i = 0; i < clouds.size(); i++)
            {

                //�t�B���^���������_�Q�ۑ��p
                pcl_ptr cloud_down_sampled(new pcl::PointCloud<PointT>);
                pcl_ptr cloud_pass_filtered(new pcl::PointCloud<PointT>);
                pcl_ptr cloud_sor_filtered(new pcl::PointCloud<PointT>);

                //std::cout << i << std::endl;
                //id�Ńt�B���^
                //39 bottle
                //40 wine glass
                //41 cup
                //64 mouse
                if (classIds[i] == 39 || classIds[i] == 40 || classIds[i] == 41 || classIds[i] == 64) {
                    std::cerr << "PointCloud before filtering has: " << clouds[i]->size() << " data points." << std::endl;

                    //
                    down_sampling(clouds[i], cloud_down_sampled, 0.01f);
                    std::cerr << "PointCloud after down sampling has: " << cloud_down_sampled->size() << " data points." << std::endl;


                    //�p�X�X���[�t�B���^
                    //�w��͈͓��̂��̂����c��
                    passthrough_filter(cloud_down_sampled, cloud_pass_filtered);
                    std::cerr << "PointCloud after passfilter has: " << cloud_pass_filtered->size() << " data points." << std::endl;

                    

                    //���v�g���ĊO��l������
                    //�d��
                    //���̑O�Ɍ��炵�Ă�������
                    remove_outliers(cloud_pass_filtered, cloud_sor_filtered);
                    std::cerr << "PointCloud after sorfilter has: " << cloud_sor_filtered->size() << " data points." << std::endl;


                    //clouds_filterd.push_back(cloud_sor_filtered);
                    //if (cloud_sor_filtered->size() > 0)clouds_filterd.push_back(clouds[i]);
                    if (cloud_sor_filtered->size() > 0) {
                        //�L�^
                        clouds_filterd.push_back(cloud_sor_filtered);

                        // viewer�ɒǉ�
                        viewer->addPointCloud<PointT>(clouds_filterd.back() , "coloud" + i);
                    };

                    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "coloud" + i);
                }
            }
            {
                std::lock_guard<std::mutex> lock(boxes_mutex);
                boxes.clear();

                for (int i = 0; i < clouds_filterd.size(); i++)
                {
                    myBox box;
                    get_box(clouds_filterd[i], box);
                    boxes.push_back(box);

                    //�����p
                    auto min_x = box.x - (box.xLength / 2);
                    auto min_y = box.y - (box.yLength / 2);
                    auto min_z = box.z - (box.zLength / 2);
                    auto max_x = box.x + (box.xLength / 2);
                    auto max_y = box.y + (box.yLength / 2);
                    auto max_z = box.z + (box.zLength / 2);
                    viewer->addCube(min_x, max_x, min_y, max_y, min_z, max_z, 1, 1, 1, "cube" + i);
                }
            }

            
            if (clouds_filterd.size() > 0) {
                if (GetAsyncKeyState(VK_SPACE) & 1) {
                    // �X�y�[�X��������Ă��鎞�̏���
                    save_image_and_pointclouds(image, editedImage, clouds_filterd);
                }

                viewer->addCoordinateSystem();
                viewer->setRepresentationToWireframeForAllActors();
                viewer->spinOnce(10);
            }
        }
    }
}

void points_position_detector::get_texture(cv::Mat& out)
{
    if (rs_man.sensor_isrunning) {
        rs_man.get_aligned_RGBimage(out);
        if (out.cols > 0) {
            //cv::flip(out, out, 0);
            //Unity�p
            cv::cvtColor(out, out, cv::COLOR_RGB2BGRA);
        }
    }
}

void points_position_detector::get_boxes(std::vector<myBox>& boxes)
{
    std::lock_guard<std::mutex> lock(boxes_mutex);
    boxes = this->boxes;
}



void points_position_detector::save_image_and_pointclouds(
    cv::Mat& image, 
    cv::Mat& detected_image,
    std::vector<pcl_ptr>& clouds)
{
    // �쐬����PointCloud��PCD�`���ŕۑ�����
    cout << "savePCDFileASCII" << endl;
    
    // �e�L�X�g�`���ŕۑ�����
    for (int i = 0; i < clouds.size(); i++) {
        pcl::io::savePCDFileASCII("p_cloud_ascii_" + std::to_string(i) + ".pcd", *clouds[i]);
    }

    // �摜�ۑ�
    cv::imwrite("image.png", image);
    cv::imwrite("detected_image.png", detected_image);
}

/// <summary>
/// �摜��̍��W�Ŏ����������̋�`���Ɋ܂܂��_�Q��
/// �[�x�摜�E�Z���T�̓����p�����[�^��p���Čv�Z
/// </summary>
/// <param name="clouds">�o��</param>
/// <param name="rects">�摜��̘g</param>
/// <param name="aligned_color_frame">rs��video_frame</param>
/// <param name="aligned_depth_frame">rs��depth_frame</param>
/// <param name="intrinsics"></param>
void points_position_detector::get_pointclouds(
    std::vector<pcl_ptr>& clouds,
    std::vector<cv::Rect>& rects,
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
        get_pointcloud(cloud, rect, aligned_color_frame, aligned_depth_frame, intrinsics);
        std::cerr << "PointCloud cutting has: " << cloud->size() << " data points." << std::endl;

        clouds.push_back(cloud);       
    }
}

void points_position_detector::get_pointcloud(
    pcl_ptr cloud,
    cv::Rect& rect,
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

            //�摜�̈ʒu��3D�_
            auto depth = depth_frame.get_distance(j, i);
            rs2_deproject_pixel_to_point(point, intrinsics, pixel, depth);

            if (point[0] != 0 && point[1] != 0 && point[2] != 0) {
                pcl::PointXYZ p(point[0], point[1], point[2]);
                cloud->points.push_back(p);
            }
            

            // std::cerr << "x: " << p.x << "  y: " << p.y << "  z: " << p.z << std::endl;
        }
    }
}
void points_position_detector::passthrough_filter(pcl_ptr cloud, pcl_ptr out)
{
    pcl::PassThrough<PointT> pass;
    // Build a passthrough filter to remove spurious NaNs
    //���s�����t�B���^
    pass.setInputCloud(cloud);
    //Z��
    pass.setFilterFieldName("z");
    //10cm-1m
    pass.setFilterLimits(0.01, 1);
    pass.filter(*out);
}
void points_position_detector::remove_outliers(pcl_ptr cloud, pcl_ptr out)
{
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);//�O��l����������_�Q�����
    sor.setMeanK(50);//MeanK��ݒ�
    sor.setStddevMulThresh(0.1);
    sor.setNegative(false);//�O��l���o�͂���ꍇ��true�ɂ���
    sor.filter(*out);//�o��  
}

void points_position_detector::down_sampling(pcl_ptr cloud, pcl_ptr out, float leafsize)
{
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leafsize, leafsize, leafsize);
    vg.filter(*out);
}

void points_position_detector::get_box(pcl_ptr cloud, myBox& box) 
{
    //pcl��minMax3D�����̂��g���Ȃ�&�x���炵���̂Ńx�^
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
    std::cout << "xl: " << box.xLength << "  yl: " << box.yLength << "  zl: " << box.zLength << std::endl;
}