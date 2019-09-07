#include "sintel_stereo_publisher.h"

#include <sensor_msgs/Image.h>
#include <sintel_stereo_publisher/SintelInfo.h>

#include <cstdlib>
#include <fstream>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include <boost/format.hpp>

namespace sintel_stereo_publisher
{
  SintelStereoPublisher::SintelStereoPublisher()
  {
    _private_node_handle = ros::NodeHandle("~");

    loadROSParameters();

    _image_transport.reset(new image_transport::ImageTransport(_private_node_handle));
    _camera_publisher_left = _image_transport->advertiseCamera("left/image_rect_color", 1);
    _camera_publisher_right = _image_transport->advertiseCamera("right/image_rect_color", 1);

    _sintel_info_publisher = _private_node_handle.advertise<sintel_stereo_publisher::SintelInfo>("sintel_info", 1);

    _start_publish_server = _private_node_handle.advertiseService("start_publish", &SintelStereoPublisher::startPublishCallback, this);

    publishSequence();
  }

  void SintelStereoPublisher::loadCameraParameters(std::string camera_param_path)
  {
    std::ifstream camera_param_file(camera_param_path, std::ios_base::binary);
    if (!camera_param_file.is_open())
    {
      ROS_FATAL_STREAM("Can't open camera parameter file: " << camera_param_path);
      ros::shutdown();
      std::exit(EXIT_FAILURE);
    }
    camera_param_file.seekg(4); // Move to intrinsic matrix position
    camera_param_file.read(reinterpret_cast<char*>(_camera_info_left.K.data()), 3*3*sizeof(double));
    camera_param_file.close();

    // Set params to left camera info
    _camera_info_left.header.frame_id = _FRAME_ID_LEFT;
    _camera_info_left.height = _HEIGHT;
    _camera_info_left.width = _WIDTH;
    _camera_info_left.P[0] = _camera_info_left.K[0]; // fx
    _camera_info_left.P[2] = _camera_info_left.K[2]; // cx
    _camera_info_left.P[5] = _camera_info_left.K[4]; // fy
    _camera_info_left.P[6] = _camera_info_left.K[5]; // cy
    _camera_info_left.P[10] = 1.0;

    //Set params to right camera info
    _camera_info_right = _camera_info_left;
    _camera_info_right.header.frame_id = _FRAME_ID_RIGHT;
    _camera_info_right.P[3] = -1.0 * _camera_info_right.P[0] * 0.10; // Tx = -fx * baseline
  }

  void SintelStereoPublisher::loadROSParameters()
  {
    // Set dataset directory
    std::string dataset_directory_string;
    if (!_private_node_handle.getParam("dataset_directory", dataset_directory_string))
    {
      ROS_FATAL("Path of MPI Sintel Stereo Training Dataset is not specified by ROS param!");
      ros::shutdown();
      std::exit(EXIT_FAILURE);
    }
    _dataset_directory = boost::filesystem::path(dataset_directory_string);

    // Set camera param file
    std::string camera_param_file;
    if (!_private_node_handle.getParam("camera_param_file", camera_param_file))
    {
      ROS_FATAL("Path of MPI Sintel Stereo Training Dataset is not specified by ROS param!");
      ros::shutdown();
      std::exit(EXIT_FAILURE);
    }
    loadCameraParameters(camera_param_file);

    _render_pass = _private_node_handle.param<std::string>("render_pass", "clean");
    if (_render_pass != "clean" && _render_pass != "final")
    {
      ROS_FATAL("Path of MPI Sintel Stereo Training Dataset is not specified by ROS param!");
    }

    // Set publish rate
    _publish_rate = _private_node_handle.param<double>("publish_rate", 24.0);

    // Load other ROS parameters
    _loop = _private_node_handle.param<bool>("loop", false);
    _pause = _private_node_handle.param<bool>("pause", false);
    _sequence_name = _private_node_handle.param<std::string>("sequence_name", "alley_1");
  }

  void SintelStereoPublisher::publishSequence()
  {
    // Set sequence path
    boost::filesystem::path image_dir_left = _dataset_directory;
    image_dir_left /= "training/" + _render_pass + "_left/" + _sequence_name;
    boost::filesystem::path image_dir_right = _dataset_directory;
    image_dir_right /= "training/" + _render_pass + "_right/" + _sequence_name;

    // Set parameter shared in all frames to CvImage
    _cv_image_left.header.frame_id = _FRAME_ID_LEFT;
    _cv_image_left.encoding = "bgr8";
    _cv_image_right.header.frame_id = _FRAME_ID_RIGHT;
    _cv_image_right.encoding = "bgr8";

    // Set parameter shared in all frames to SintelInfo 
    sintel_stereo_publisher::SintelInfo sintel_info;
    sintel_info.header.frame_id = _FRAME_ID_LEFT;
    sintel_info.publish_rate = _publish_rate;
    sintel_info.render_pass = _render_pass;
    sintel_info.sequence_name = _sequence_name;

    ros::Rate publish_rate_keeper(_publish_rate);
    ros::Rate service_rate_keeper(10.0);

    while (ros::ok() && _pause)
    {
      ros::spinOnce();
      service_rate_keeper.sleep();
    }

    for (int frame_number = 1; frame_number <= _MAX_FRAME_NUMBER; frame_number++)
    {
      if (!ros::ok())
        break;

      // Load left image from a file
      boost::filesystem::path image_file_left = image_dir_left;
      image_file_left /= (boost::format("frame_%1$04d.png") % frame_number).str();
      _cv_image_left.image = cv::imread(image_file_left.string());
      if (_cv_image_left.image.empty())
      {
        ROS_FATAL_STREAM("Can't read a left image: " << image_file_left.string());
        ros::shutdown();
        std::exit(EXIT_FAILURE);
      }

      // Load right image from a file
      boost::filesystem::path image_file_right = image_dir_right;
      image_file_right /= (boost::format("frame_%1$04d.png") % frame_number).str();
      _cv_image_right.image = cv::imread(image_file_right.string());
      if (_cv_image_right.image.empty())
      {
        ROS_FATAL_STREAM("Can't read a right image: " << image_file_right.string());
        ros::shutdown();
        std::exit(EXIT_FAILURE);
      }

      sintel_info.frame_number = frame_number;

      publish_rate_keeper.sleep();

      // Publish
      ros::Time time_stamp = ros::Time::now();
      _camera_publisher_left.publish(*(_cv_image_left.toImageMsg()), _camera_info_left, time_stamp);
      _camera_publisher_right.publish(*(_cv_image_right.toImageMsg()), _camera_info_right, time_stamp);
      sintel_info.header.stamp = time_stamp;
      _sintel_info_publisher.publish(sintel_info);

      if (_loop && frame_number == _MAX_FRAME_NUMBER)
      {
        ROS_INFO("Publish of all frames is finished and back to first frame.");
        frame_number = 0; // frame_number is 1 at next loop
      }
    }

    ROS_INFO("Publish of all frames is finished");
    ROS_INFO("Exit by Ctrl-C");

    ros::spin();
  }

  bool SintelStereoPublisher::startPublishCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
  {
    ROS_INFO("Start publishing");
    _pause = false;
    return true;
  }
}
