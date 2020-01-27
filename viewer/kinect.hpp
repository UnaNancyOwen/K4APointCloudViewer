#ifndef __KINECT__
#define __KINECT__

#include <k4a/k4a.hpp>
#include <pcl/visualization/pcl_visualizer.h>

class kinect
{
private:
    // Kinect
    k4a::device device;
    k4a::capture capture;
    k4a::calibration calibration;
    k4a::transformation transformation;
    k4a_device_configuration_t device_configuration;
    uint32_t device_index;

    // Depth
    k4a::image depth_image;

    // Point Cloud
    k4a::image xyz_image;

    // PCL
    pcl::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZ>::Ptr handler;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

public:
    // Constructor
    kinect( const uint32_t index = K4A_DEVICE_DEFAULT );

    // Destructor
    ~kinect();

    // Run
    void run();

    // Update
    void update();

    // Draw
    void draw();

    // Show
    void show();

private:
    // Initialize
    void initialize();

    // Initialize Sensor
    void initialize_sensor();

    // Initialize Viewer
    void initialize_viewer();

    // Keyboard Callback
    void keyboard_callback( const pcl::visualization::KeyboardEvent& event, void* cookie );

    // Finalize
    void finalize();

    // Update Frame
    void update_frame();

    // Update Depth
    void update_depth();

    // Update Point Cloud
    void update_point_cloud();

    // Draw Point Cloud
    void draw_point_cloud();

    // Show Point Cloud
    void show_point_cloud();
};

#endif // __KINECT__
