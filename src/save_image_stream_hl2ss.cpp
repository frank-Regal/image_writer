#include <string>
#include <filesystem>
#include <cassert>

#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "Public/ImageWriter.h"
#include "Public/ImageWriterParams.h"
#include "hl2ss.h"


int main(int argc, char** argv) {

    // init defaults
    ros::init(argc, argv, "save_raw_image_stream");
    ros::NodeHandle nh;
    std::string filepath {ros::package::getPath("image_writer")};
    std::string filename {"video"};
    std::string topicname_image {"/image_raw"};
    std::string topicname_empty {"/stop_image_write"};
    bool save_multi_stream_in_sequence {false};
    bool use_param_server {false};
    int fps {20};
    std::string device_id {""};

    // Get the namespace of the node
    std::string node_ns = nh.getNamespace();

    nh.param<std::string>("filepath", filepath, filepath);
    nh.param<std::string>("filename", filename, filename);
    nh.param<std::string>("topicname_image", topicname_image, topicname_image);
    nh.param<std::string>("topicname_empty", topicname_empty, topicname_empty);
    nh.param<bool>("save_multi_stream_in_sequence", save_multi_stream_in_sequence, save_multi_stream_in_sequence);
    nh.param<bool>("use_param_server", use_param_server, use_param_server);
    nh.param<int>("fps", fps, fps);

    // std::string output_path {filepath + filename + ".avi"};
    // std::cout << "Output Path: " << output_path << std::endl;

    // create directories if they do not exist
    std::filesystem::create_directories(filepath);

    // init subscribers
    ros::Subscriber sub_image;
    ros::Subscriber sub_empty;

    // init objectsß
    ImageWriterParams image_writer_params(filepath, fps, save_multi_stream_in_sequence, nh, node_ns);
    ImageWriter image_writer(filepath, fps, save_multi_stream_in_sequence);

    // setup subscribers
    if (use_param_server){
        sub_image = nh.subscribe<sensor_msgs::Image>(topicname_image, 100, &ImageWriterParams::imageCallback, &image_writer_params);
        sub_empty = nh.subscribe<std_msgs::Empty>(topicname_empty, 1, &ImageWriterParams::emptyCallback, &image_writer_params);
        ROS_WARN("'%s' node configured for param server mode", node_ns.c_str());
    } else {
        sub_image = nh.subscribe<sensor_msgs::Image>(topicname_image, 100, &ImageWriter::imageCallback, &image_writer);
        sub_empty = nh.subscribe<std_msgs::Empty>(topicname_empty, 1, &ImageWriter::emptyCallback, &image_writer);
        ROS_WARN("'%s' node configured for single node", node_ns.c_str());
    }

    ros::spin();
    return 0;
}
