// From ROS packages
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

// From Non-ROS libraries
#include <boost/filesystem.hpp>
#include <memory>

namespace sintel_stereo_publisher
{
  class SintelStereoPublisher 
  {
  private:
    /**
     * @brief Frame id of left camera
     */
    const std::string _FRAME_ID_LEFT = "sintel_left";
    /**
     * @brief Frame id of right camera
     */
    const std::string _FRAME_ID_RIGHT = "sintel_right";
    /**
     * @brief Width of images
     */
    const int _WIDTH = 1024;
    /**
     * @brief Height of images
     */
    const int _HEIGHT = 436;
    /**
     * @brief Maximum frame number of each sequence
     */
    const int _MAX_FRAME_NUMBER = 50;

    /**
     * @brief ROS node handle of private namespace
     */
    ros::NodeHandle _private_node_handle;
    
    /**
     * @brief Transport to publish images
     */
    std::shared_ptr<image_transport::ImageTransport> _image_transport;
    /**
     * @brief Publisher for left image and camera info
     */
    image_transport::CameraPublisher _camera_publisher_left;
    /**
     * @brief Publisher for right image and camera info
     */
    image_transport::CameraPublisher _camera_publisher_right;
    /**
     * @brief Publisher for information of current sequence and frame number of image
     */
    ros::Publisher _sintel_info_publisher;

    /**
     * @brief Root directory of MPI Sintel Stereo Training Data, set by ROS param. 
     * 
     * Under the root directory, there should be `training` and `sdk` directories.
     */
    boost::filesystem::path _dataset_directory;
    /**
     * @brief Name of sequence directory which contains stereo images
     */
    std::string _sequence_name;

    /**
     * @brief Render pass of images 
     * 
     * Valid value is clean and final.
     */
    std::string _render_pass;

    /**
     * @brief CameraInfo for published left images
     */
    sensor_msgs::CameraInfo _camera_info_left;
    /**
     * @brief CameraInfo for published right images
     */
    sensor_msgs::CameraInfo _camera_info_right;

    /**
     * @brief Used to load left image file and convert to ROS msg
     */
    cv_bridge::CvImage _cv_image_left;
    /**
     * @brief Used to load right image file and convert to ROS msg
     */
    cv_bridge::CvImage _cv_image_right;

    /**
     * @brief Published frames per second
     */
    double _publish_rate;

    /**
     * @brief If true, publish of all frames in a sequence is finished then back to start and continue publishing
     */
    bool _loop;

    /**
     * @brief Load intrinsic camera parameters from a file contained in MPI Sintel Depth Training Data
     */
    void loadCameraParameters(std::string camera_param_path);
    /**
     * @brief Load ROS params
     */
    void loadROSParameters();
    /**
     * @brief Load stereo sequence and publish
     */
    void publishSequence();
    
  public:
    SintelStereoPublisher();
  };
}