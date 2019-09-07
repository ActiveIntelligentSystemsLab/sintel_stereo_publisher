# sintel_stereo_publisher

ROS package to publish stereo images from [MPI Sintel Stereo Training Data](http://sintel.is.tue.mpg.de/stereo).

## Requirement

* Docker
* Docker Compose

## Run with Docker

```shell
$ git clone https://github.com/ActiveIntelligentSystemsLab/sintel_stereo_publisher
$ cd sintel_stereo_publisher
$ xhost +local:root
$ sudo docker-compose up
```

Then a image viewer window opens.

## Node: sintel_stereo_publisher

Publish stereo images from MPI Sintel Stereo Training Data.

### Published topics

* `~left/image_rect_color` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
* `~left/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))
* `~right/image_rect_color` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
* `~right/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))
* `~sintel_info` ([sintel_stereo_publisher/SintelInfo](msg/SintelInfo.msg))

  Used to obtain frame number which is associated to image by timestamp.

  Also it contains information of published sequence.

### Parameters

* `~dataset_directory` (string, default: None)

  Root directory of MPI Sintel Stereo Training Data. 
  Under the root directory, there should be `training` and `sdk` directories.

  If this is not set, the node just exits with error.

* `~publish_rate` (double, default: 24.0)

  Published frames per second.

* `~sequence_name` (string, default: "alley_1")

  Name of sequence directory which you want to publish.

  There are sequence directories under `_dataset_directory/training/`.

* `~camera_param_file` (string, default: None)

  Path of a file which contain camera parameters.
  Used to publish [CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html).

  The file is contained in [MPI Sintel Depth Training Data](http://sintel.is.tue.mpg.de/depth) and has `.cam` extension.

  If this is not set, the node just exits with error.

* `~loop` (bool, default: false)

  If publish of all frames in a sequence is finished, back to start and continue publishing.

* `~render_pass` (string, default: "clean")

  Choose render pass from "clean" and "final".

  See [MPI Sintel Dataset's paper](http://sintel.is.tue.mpg.de/) to know about "render pass".

## ROS Message Types

* [SintelInfo](msg/SintelInfo.msg)
