#include "Public/SyncImageWriter.h"

SyncImageWriter::SyncImageWriter(std::string output_path, int fps, bool save_multi_stream_in_sequence, ros::NodeHandle nh)
    : ImageWriter{output_path, fps, save_multi_stream_in_sequence}
    , nh_{nh}
{

}

SyncImageWriter::~SyncImageWriter(){}

void SyncImageWriter::updateOutputPath(){
    // get last "/" before output file name
    size_t slashPos = orig_output_path_.find_last_of("/");
    if (slashPos == std::string::npos) {
        std::cerr << "Error: '/' not found in string for file location path." << std::endl;
        return;
    }

    // update output path variable with a time stamp pre-fixed to video file.
    std::string path_to_file = orig_output_path_.substr(0, slashPos);
    std::string filename = orig_output_path_.substr(slashPos + 1);
    output_path_ = path_to_file + "/" + "myNameIsFrank.avi";
}

void SyncImageWriter::imageCallback(const sensor_msgs::Image::ConstPtr& msg) 
{
    //writeDataToImage(msg);
}

void SyncImageWriter::emptyCallback(const std_msgs::Empty::ConstPtr& msg)
{
    //resetImageWriter();
}