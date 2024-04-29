#include "Public/ImageWriterParams.h"

ImageWriterParams::ImageWriterParams(
    const std::string& output_path, 
    const int& fps, 
    const bool& save_multi_stream_in_sequence, 
    const ros::NodeHandle& nh,
    const std::string& node_ns)
    : ImageWriter::ImageWriter(output_path, fps, save_multi_stream_in_sequence)
    , nh_(nh)
    , node_ns_(node_ns) {

}

void ImageWriterParams::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    auto func = [this](){this->updateOutputPath();};
    writeDataToImage(msg, func);
}
void ImageWriterParams::emptyCallback(const std_msgs::Empty::ConstPtr& msg) {
    resetImageWriter();
}

void ImageWriterParams::updateOutputPath() {
    
    // get last "/" before output file name
    size_t slashPos = orig_output_path_.find_last_of("/");
    if (slashPos == std::string::npos) {
        std::cerr << "Error: '/' not found in string for file location path." << std::endl;
        return;
    }

    // update output path variable with a time stamp pre-fixed to video file.
    std::string path_to_file = orig_output_path_.substr(0, slashPos);
    
    // set the file name
    std::string filename {""};
    std::string filename_param {node_ns_ + "/filename"};
    if (!nh_.getParam(filename_param, filename)) {ROS_ERROR("Filename not set");}

    // new file path
    output_path_ = path_to_file + "/" + filename + ".avi";
}