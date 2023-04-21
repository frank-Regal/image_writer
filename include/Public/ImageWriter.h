#ifndef _IMAGE_WRITER_H_
#define _IMAGE_WRITER_H_

#include <string>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"

class ImageWriter {

public:
    ImageWriter(std::string output_path, int fps); // constructor
    ~ImageWriter(); // deconstructor

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg); // callback
    void emptyCallback(const std_msgs::Empty::ConstPtr& msg); // stop callback

private:
    std::string output_path_;
    int fps_;
    cv::VideoWriter* video_writer_;
};

#endif