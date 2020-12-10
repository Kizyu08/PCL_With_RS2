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
}