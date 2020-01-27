#include "kinect.hpp"

#include <chrono>
#include <sstream>
#include <iomanip>

#include <pcl/io/pcd_io.h>

// Constructor
kinect::kinect( const uint32_t index )
    : device_index( index )
{
    // Initialize
    initialize();
}

kinect::~kinect()
{
    // Finalize
    finalize();
}

// Initialize
void kinect::initialize()
{
    // Initialize Sensor
    initialize_sensor();

    // Initialize Viewer
    initialize_viewer();
}

// Initialize Sensor
inline void kinect::initialize_sensor()
{
    // Get Connected Devices
    const int32_t device_count = k4a::device::get_installed_count();
    if( device_count == 0 ){
        throw k4a::error( "Failed to found device!" );
    }

    // Open Default Device
    device = k4a::device::open( device_index );

    // Start Cameras with Configuration
    device_configuration = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    device_configuration.color_format             = k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32;
    device_configuration.color_resolution         = k4a_color_resolution_t::K4A_COLOR_RESOLUTION_720P;
    device_configuration.depth_mode               = k4a_depth_mode_t::K4A_DEPTH_MODE_NFOV_UNBINNED;
    device_configuration.synchronized_images_only = true;
    device_configuration.wired_sync_mode          = k4a_wired_sync_mode_t::K4A_WIRED_SYNC_MODE_STANDALONE;
    device.start_cameras( &device_configuration );

    // Get Calibration
    calibration = device.get_calibration( device_configuration.depth_mode, device_configuration.color_resolution );

    // Create Transformation
    transformation = k4a::transformation( calibration );
}

// Initialize Viewer
inline void kinect::initialize_viewer()
{
    // Create Point Cloud
    cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->width  = static_cast<uint32_t>( calibration.depth_camera_calibration.resolution_width );
    cloud->height = static_cast<uint32_t>( calibration.depth_camera_calibration.resolution_height );
    cloud->points.resize( cloud->height * cloud->width );
    cloud->is_dense = false;

    // Create PCLVisualizer
    viewer = pcl::make_shared<pcl::visualization::PCLVisualizer>( "Point Cloud Viewer" );

    // Register Keyboard Callback
    viewer->registerKeyboardCallback( &kinect::keyboard_callback, *this );

    // Color Handler
    std::vector<double> color = { 255.0, 255.0, 255.0 };
    handler = pcl::make_shared<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>>( color[0], color[1], color[2] );
}

// Keyboard Callback
void kinect::keyboard_callback( const pcl::visualization::KeyboardEvent& event, void* cookie )
{
    if( event.getKeyCode() == 's' && event.keyDown() ){
        static int32_t i = 0;
        std::ostringstream oss;
        oss << std::setfill( '0' ) << std::setw( 3 ) << i++;
        pcl::io::savePCDFile( oss.str() + ".pcd", *cloud );
    }
}

// Finalize
void kinect::finalize()
{
    // Destroy Transformation
    transformation.destroy();

    // Stop Cameras
    device.stop_cameras();

    // Close Device
    device.close();
}

// Run
void kinect::run()
{
    // Main Loop
    while( !viewer->wasStopped() ){
        // Update
        update();

        // Draw
        draw();

        // Show
        show();
    }
}

// Update
void kinect::update()
{
    // Update Frame
    update_frame();

    // Update Depth
    update_depth();

    // Update Point Cloud
    update_point_cloud();

    // Release Capture Handle
    capture.reset();
}

// Update Frame
inline void kinect::update_frame()
{
    // Get Capture Frame
    constexpr std::chrono::milliseconds time_out( K4A_WAIT_INFINITE );
    const bool result = device.get_capture( &capture, time_out );
    if( !result ){
        this->~kinect();
    }
}

// Update Depth
inline void kinect::update_depth()
{
    // Get Depth Image
    depth_image = capture.get_depth_image();
}

// Update Point Cloud
inline void kinect::update_point_cloud()
{
    if( !depth_image.handle() ){
        return;
    }

    // Transform Depth Image to Point Cloud
    xyz_image = transformation.depth_image_to_point_cloud( depth_image, K4A_CALIBRATION_TYPE_DEPTH );

    // Reset Point Cloud
    cloud->clear();
    cloud->width  = static_cast<uint32_t>( xyz_image.get_width_pixels()  );
    cloud->height = static_cast<uint32_t>( xyz_image.get_height_pixels() );
    cloud->points.resize( cloud->height * cloud->width );

    // Convert pcl::PointCloud
    const int16_t* buffer = reinterpret_cast<int16_t*>( xyz_image.get_buffer() );
    for( int32_t index = 0; index < cloud->size(); index += 3 ){
        pcl::PointXYZ point;
        point.x = buffer[index + 0];
        point.y = buffer[index + 1];
        point.z = buffer[index + 2];
        cloud->push_back( point );
    }

    // Release Depth Imaage and Point Cloud Image Handle
    depth_image.reset();
    xyz_image.reset();
}

// Draw
void kinect::draw()
{
    // Draw Point Cloud
    draw_point_cloud();
}

// Draw Point Cloud
inline void kinect::draw_point_cloud()
{
    // Update Point Cloud
    if( !viewer->updatePointCloud( cloud, *handler, "cloud" ) ){
        viewer->addPointCloud( cloud, *handler, "cloud" );
    }
}

// Show
void kinect::show()
{
    // Show Point Cloud
    show_point_cloud();
}

// Show Point Cloud
inline void kinect::show_point_cloud()
{
    // Update Viwer
    viewer->spinOnce();
}
