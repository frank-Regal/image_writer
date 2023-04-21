#include <string>
#include <ros/console.h>

#include "Public/ImageWriter.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "save_raw_image_stream");
    ros::NodeHandle nh;

    // init and get params
    std::string filepath {ros::package::getPath("image_writer")};
    std::string filename {"/data/video.avi"};
    bool isFilenameStamped {false};
    int fps {30};
    
    nh.param<std::string>("filepath", filepath, filepath);
    nh.param<std::string>("filename", filename, filename);
    nh.param<bool>("isFilenameStamped", isFilenameStamped, isFilenameStamped);
    nh.param<int>("fps", fps, fps);

    std::string output_path = filepath + filename;
    std::cout << "Saving File To:\n" << output_path << std::endl;
    
    // create image saver class
    ImageWriter image_saver(output_path, fps);

    // listen and write to video
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>("/hololens/RIGHT_RIGHT/image", 1, &ImageWriter::imageCallback, &image_saver);

    ros::spin();

    return 0;
}
