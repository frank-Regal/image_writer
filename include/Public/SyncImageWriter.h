#ifndef _DATASET_IMAGE_WRITER_H_
#define _DATASET_IMAGE_WRITER_H_

#include "ImageWriter.h"

class SyncImageWriter : public ImageWriter 
{
public:
    // constructors
    SyncImageWriter(std::string output_path, int fps, bool save_multi_stream_in_sequence, ros::NodeHandle nh);
    ~SyncImageWriter();

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void emptyCallback(const std_msgs::Empty::ConstPtr& msg);

protected:
    void updateOutputPath();

private:
    ros::NodeHandle nh_;
};

#endif