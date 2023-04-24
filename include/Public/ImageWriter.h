#ifndef _IMAGE_WRITER_H_
#define _IMAGE_WRITER_H_

#include <string>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"

class ImageWriter {

public:
    // constructor & destructor
    ImageWriter(std::string output_path, int fps); 
    ImageWriter(std::string output_path, int fps, bool process_multi_images);
    ~ImageWriter();

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg); // callback
    void emptyCallback(const std_msgs::Empty::ConstPtr& msg); // stop callback

private:
    void getTimeStamp(std::string& time_stamp_out);
    void updateOutputPath();
    std::string output_path_;
    std::string orig_output_path_;
    int fps_;
    cv::VideoWriter* video_writer_;
    bool save_multi_stream_in_sequence_;
};

#endif