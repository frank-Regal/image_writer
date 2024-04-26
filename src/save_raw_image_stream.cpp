#include <string>

#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "Public/ImageWriter.h"
#include "Public/SyncImageWriter.h"


int main(int argc, char** argv) {

    // init defaults
    ros::init(argc, argv, "save_raw_image_stream");
    ros::NodeHandle nh;
    std::string filepath {ros::package::getPath("image_writer")};
    std::string filename {"video"};
    std::string topicname_image {"/image_raw"};
    std::string topicname_empty {"/stop_image_write"};
    bool save_multi_stream_in_sequence {false};
    int fps {20};
    
    nh.param<std::string>("filepath", filepath, filepath);
    nh.param<std::string>("filename", filename, filename);
    nh.param<std::string>("topicname_image", topicname_image, topicname_image);
    nh.param<std::string>("topicname_empty", topicname_empty, topicname_empty);
    nh.param<bool>("save_multi_stream_in_sequence", save_multi_stream_in_sequence, save_multi_stream_in_sequence);
    nh.param<int>("fps", fps, fps);

    std::string output_path {filepath + filename + ".avi"};
    std::cout << output_path << std::endl;
    
    // create image writer object
    std::string timestamp;
    // Check if the parameter exists
    if (nh.hasParam("config_for_dataset")) {
        // Get the parameter
        nh.param<std::string>("timestamp", timestamp, "default_value");
        ROS_INFO("Timestamp parameter: %s", timestamp.c_str());
    } else {
        ROS_WARN("Parameter 'timestamp' is not set. Using default value.");
        timestamp = "default_value";  // Set default or handle the error
    }

    SyncImageWriter param_image_writer(output_path, fps, save_multi_stream_in_sequence, nh);

    // listen and write to video
    // ros::Subscriber sub_image = nh.subscribe<sensor_msgs::Image>(topicname_image, 100, &SyncImageWriter::imageCallback, &param_image_writer);

    // // listen for stop write message
    // ros::Subscriber sub_empty = nh.subscribe<std_msgs::Empty>(topicname_empty, 1, &SyncImageWriter::emptyCallback, &param_image_writer);

    ros::spin();
    return 0;
}
