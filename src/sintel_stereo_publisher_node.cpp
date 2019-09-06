#include "sintel_stereo_publisher.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sintel_stereo_publisher");

  sintel_stereo_publisher::SintelStereoPublisher sintel_stereo_publisher;

  return 0;
}