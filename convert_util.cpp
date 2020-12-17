#include "convert_util.hpp"

namespace convertutil {

    pcl_ptr points_to_pcl(const rs2::points& points)
    {
        pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        auto sp = points.get_profile().as<rs2::video_stream_profile>();
        cloud->width = sp.width();
        cloud->height = sp.height();
        cloud->is_dense = false;
        cloud->points.resize(points.size());
        auto ptr = points.get_vertices();
        for (auto& p : cloud->points)
        {
            p.x = ptr->x;
            p.y = ptr->y;
            p.z = ptr->z;
            ptr++;
        }

        return cloud;
    }

    cv::Mat frame_to_mat(const rs2::frame& f)
    {
        using namespace cv;
        using namespace rs2;

        auto vf = f.as<video_frame>();
        const int w = vf.get_width();
        const int h = vf.get_height();

        if (f.get_profile().format() == RS2_FORMAT_BGR8)
        {
            return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        }
        else if (f.get_profile().format() == RS2_FORMAT_RGB8)
        {
            auto r = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
            cvtColor(r, r, COLOR_RGB2BGR);
            return r;
        }
        else if (f.get_profile().format() == RS2_FORMAT_Z16)
        {
            return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
        }
        else if (f.get_profile().format() == RS2_FORMAT_Y8)
        {
            return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
        }

        throw std::runtime_error("Frame format is not supported yet!");
    }
}