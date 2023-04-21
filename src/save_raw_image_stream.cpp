#include <string>

#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "Public/ImageWriter.h"

int main(int argc, char** argv) {

    // init
    ros::init(argc, argv, "save_raw_image_stream");
    ros::NodeHandle nh;
    std::string filepath {ros::package::getPath("image_writer")};
    std::string filename {"/output/example_video.avi"};
    std::string topicname_image {"/image_raw"};
    std::string topicname_empty {"/stop_image_write"};
    bool isFilenameStamped {false};
    int fps {30};
    
    // get params - edit in params/params.yaml
    nh.param<std::string>("filepath", filepath, filepath);
    nh.param<std::string>("filename", filename, filename);
    nh.param<std::string>("topicname_image", topicname_image, topicname_image);
    nh.param<std::string>("topicname_empty", topicname_empty, topicname_empty);
    nh.param<bool>("isFilenameStamped", isFilenameStamped, isFilenameStamped);
    nh.param<int>("fps", fps, fps);

    // setup output file
    std::string output_path {filepath + filename};
    if (isFilenameStamped) { output_path = filepath + "test" + filename;}
    std::cout << "\n\nSaving File To:\n" << output_path << std::endl;
    
    // create image writer object
    ImageWriter image_writer(output_path, fps);

    // listen and write to video
    ros::Subscriber sub_image = nh.subscribe<sensor_msgs::Image>(topicname_image, 100, &ImageWriter::imageCallback, &image_writer);

    // listen for stop write message
    ros::Subscriber sub_empty = nh.subscribe<std_msgs::Empty>(topicname_empty, 1, &ImageWriter::emptyCallback, &image_writer);

    ros::spin();
    return 0;
}
