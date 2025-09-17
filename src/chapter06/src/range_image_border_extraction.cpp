#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZ PointType;

// Parameters
float angular_resolution = 0.5f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;

void printUsage(const char* progName)
{
    std::cout << "\n\nUsage: " << progName << " [options] <scene.pcd>\n\n"
              << "Options:\n"
              << "-------------------------------------------\n"
              << "-r <float>   angular resolution in degrees (default " << angular_resolution << ")\n"
              << "-c <int>     coordinate frame (default " << (int)coordinate_frame << ")\n"
              << "-m           Treat all unseen points to max range\n"
              << "-h           this help\n"
              << "\n\n";
}

int main(int argc, char** argv)
{
    if (pcl::console::find_argument(argc, argv, "-h") >= 0)
    {
        printUsage(argv[0]);
        return 0;
    }
    if (pcl::console::find_argument(argc, argv, "-m") >= 0)
    {
        setUnseenToMaxRange = true;
        std::cout << "Setting unseen values in range image to maximum range readings.\n";
    }
    int tmp_coordinate_frame;
    if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0)
    {
        coordinate_frame = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
        std::cout << "Using coordinate frame " << (int)coordinate_frame << ".\n";
    }
    if (pcl::console::parse(argc, argv, "-r", angular_resolution) >= 0)
        std::cout << "Setting angular resolution to " << angular_resolution << "deg.\n";
    angular_resolution = pcl::deg2rad(angular_resolution);

    pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
    Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
    std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");
    if (!pcd_filename_indices.empty())
    {
        std::string filename = argv[pcd_filename_indices[0]];
        if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
        {
            std::cout << "Was not able to open file \"" << filename << "\".\n";
            printUsage(argv[0]);
            return 0;
        }
        scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
                                                               point_cloud.sensor_origin_[1],
                                                               point_cloud.sensor_origin_[2])) *
                            Eigen::Affine3f(point_cloud.sensor_orientation_);
    }
    else
    {
        std::cout << "\nNo *.pcd file given => Generating example point cloud.\n\n";
        for (float x = -0.5f; x <= 0.5f; x += 0.01f)
        {
            for (float y = -0.5f; y <= 0.5f; y += 0.01f)
            {
                PointType point;
                point.x = x;
                point.y = y;
                point.z = 2.0f - y;
                point_cloud.points.push_back(point);
            }
        }
        point_cloud.width = point_cloud.points.size();
        point_cloud.height = 1;
    }

    // Create RangeImage from the PointCloud
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;
    range_image.createFromPointCloud(point_cloud, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                     scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    if (setUnseenToMaxRange)
        range_image.setUnseenToMaxRange();

    // Check if range image was created successfully
    if (range_image.points.empty())
    {
        std::cerr << "Range image creation failed or resulted in an empty range image.\n";
        return -1;
    }

    // Extract borders
    pcl::RangeImageBorderExtractor border_extractor(&range_image);
    pcl::PointCloud<pcl::BorderDescription> border_descriptions;
    border_extractor.compute(border_descriptions);

    // Create 3D viewer
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(1, 1, 1);
    viewer.addCoordinateSystem(1.0f);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler(point_cloud_ptr, 0, 0, 0);
    viewer.addPointCloud(point_cloud_ptr, point_cloud_color_handler, "original point cloud");

    // Extract border points
    pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
                                              veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
                                              shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
    for (int y = 0; y < range_image.height; ++y)
    {
        for (int x = 0; x < range_image.width; ++x)
        {
            if (border_descriptions.points[y * range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
                border_points_ptr->points.push_back(range_image.points[y * range_image.width + x]);
            if (border_descriptions.points[y * range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
                veil_points_ptr->points.push_back(range_image.points[y * range_image.width + x]);
            if (border_descriptions.points[y * range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
                shadow_points_ptr->points.push_back(range_image.points[y * range_image.width + x]);
        }
    }

    // Add border points to viewer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler(border_points_ptr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointWithRange>(border_points_ptr, border_points_color_handler, "border points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler(veil_points_ptr, 255, 0, 0);
    viewer.addPointCloud<pcl::PointWithRange>(veil_points_ptr, veil_points_color_handler, "veil points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler(shadow_points_ptr, 0, 255, 255);
    viewer.addPointCloud<pcl::PointWithRange>(shadow_points_ptr, shadow_points_color_handler, "shadow points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");

    // Main loop
    while (!viewer.wasStopped())
    {
        viewer.spin();
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }

    return 0;
}