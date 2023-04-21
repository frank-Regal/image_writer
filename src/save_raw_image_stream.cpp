#include <string>

#include "Public/ImageWriter.h"

int main(int argc, char** argv) {

    // init
    ros::init(argc, argv, "save_raw_image_stream");
    ros::NodeHandle nh;
    std::string filepath {ros::package::getPath("image_writer")};
    std::string filename {"/output/example_video.avi"};
    std::string output_path {filepath + filename};
    bool isFilenameStamped {false};
    int fps {30};
    
    // get params - edit in params/params.yaml
    nh.param<std::string>("filepath", filepath, filepath);
    nh.param<std::string>("filename", filename, filename);
    nh.param<bool>("isFilenameStamped", isFilenameStamped, isFilenameStamped);
    nh.param<int>("fps", fps, fps);

    // setup output file
    if (isFilenameStamped) {
        output_path = filepath + "test" + filename;
    }

    std::cout << "\n\nSaving File To:\n" << output_path << std::endl;
    
    // create image writer object
    ImageWriter image_saver(output_path, fps);

    // listen and write to video
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>("/hololens/RIGHT_RIGHT/image", 1, &ImageWriter::imageCallback, &image_saver);

    ros::spin();
    return 0;
}
