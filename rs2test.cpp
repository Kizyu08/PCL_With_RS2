#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

int user_data;

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere(o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;

}

void
viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0);
    user_data++;
}


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

int main()
{
    rs2::pipeline pipeline;
    rs2::points points;
    rs2::pointcloud pointcloud;

    pipeline.start();




    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    while (!viewer.wasStopped())
    {
        auto frames = pipeline.wait_for_frames();
        auto depths = frames.get_depth_frame();

        points = pointcloud.calculate(depths);
        auto pcl_points = points_to_pcl(points);
        viewer.showCloud(pcl_points);
    }
    return 0;
}