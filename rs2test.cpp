#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>


typedef pcl::PointXYZ PointT;

int user_data;

using pcl_ptr = pcl::PointCloud<PointT>::Ptr;

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
    pcl::PointCloud<PointT>::Ptr cloud_voxel_filtered(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);


    pcl::visualization::CloudViewer viewer("Cloud Viewer");
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

        //voxel filter
        voxel_filter.setInputCloud(cloud);
        float leafsize = 0.05f;
        voxel_filter.setLeafSize(leafsize, leafsize, leafsize);
        voxel_filter.filter(*cloud_voxel_filtered);
        viewer.showCloud(cloud_voxel_filtered);
        std::cerr << "PointCloud after voxel filtering has: " << cloud_voxel_filtered->size() << " data points." << std::endl;
        if (cloud_voxel_filtered->size() > 100000)continue;
        //viewer.showCloud(cloud_filtered);

        // Estimate point normals
        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud_voxel_filtered);
        ne.setKSearch(50);
        ne.compute(*cloud_normals);

        // Create the segmentation object for the planar model and set all the parameters
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setNormalDistanceWeight(0.1);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.03);
        seg.setInputCloud(cloud_voxel_filtered);
        seg.setInputNormals(cloud_normals);
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
        extract.setNegative(true);
        extract.filter(*cloud_filtered2);
        extract_normals.setNegative(true);
        extract_normals.setInputCloud(cloud_normals);
        extract_normals.setIndices(inliers_plane);
        extract_normals.filter(*cloud_normals2);

        // Create the segmentation object for cylinder segmentation and set all the parameters
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.1);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.05);
        seg.setRadiusLimits(0, 0.1);
        seg.setInputCloud(cloud_filtered2);
        seg.setInputNormals(cloud_normals2);

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
    return 0;
}