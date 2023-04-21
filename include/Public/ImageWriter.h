#ifndef _IMAGE_WRITER_H_
#define _IMAGE_WRITER_H_

#include <string>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class ImageWriter {

public:
    ImageWriter(std::string output_path, int fps); // constructor
    ~ImageWriter(); // deconstructor
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg); // callback

private:
    std::string output_path_;
    int fps_;
    cv::VideoWriter* video_writer_;
};

#endif