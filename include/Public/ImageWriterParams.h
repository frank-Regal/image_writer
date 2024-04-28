#pragma once

#include "ImageWriter.h"

class ImageWriterParams : public ImageWriter 
{
public:
    ImageWriterParams(
        const std::string& output_path, 
        const int& fps, 
        const bool& save_multi_stream_in_sequence, 
        const ros::NodeHandle& nh,
        const std::string& node_ns);

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg); // callback
    void emptyCallback(const std_msgs::Empty::ConstPtr& msg); // stop callback
protected:
    void updateOutputPath();

private:
    ros::NodeHandle nh_;
    std::string node_ns_;
};