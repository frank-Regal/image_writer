#include <string>
#include <filesystem>
#include <cassert>
#include <iostream>
#include <opencv2/highgui.hpp>

#include "Public/ImageWriter.h"
#include "Public/ImageWriterParams.h"
#include "hl2ss_lnm.h"
#include "hl2ss_mt.h"


int main(int argc, char** argv) {

    // init defaults
    ros::init(argc, argv, "save_image_stream_hl2ss");
    ros::NodeHandle nh;

    // init parameters
    std::string filepath {ros::package::getPath("image_writer")};
    std::string filename {"video.mp4"};
    bool save_multi_stream_in_sequence {false};
    bool use_param_server {false};
    int fps {30};
    std::string device_id {""};

    std::string output_path {filepath + "/output/" + filename};

    // Get the namespace of the node
    std::string node_ns = nh.getNamespace();

    nh.param<std::string>("filepath", filepath, filepath);
    nh.param<std::string>("filename", filename, filename);
    nh.param<bool>("save_multi_stream_in_sequence", save_multi_stream_in_sequence, save_multi_stream_in_sequence);
    nh.param<bool>("use_param_server", use_param_server, use_param_server);
    nh.param<int>("fps", fps, fps);

    // create directories if they do not exist
    std::filesystem::create_directories(filepath + "/output/");

    // init objectsß
    ImageWriterParams image_writer_params(filepath, fps, save_multi_stream_in_sequence, nh, node_ns);
    ImageWriter image_writer(filepath, fps, save_multi_stream_in_sequence);

    hl2ss::client::initialize();

    // init hl2ss (TODO: make this a parameter)
    char const* host {"192.168.50.33"};
    const uint16_t port {hl2ss::stream_port::RM_VLC_LEFTFRONT};

    // Create client
    std::unique_ptr<hl2ss::rx_rm_vlc> client = hl2ss::lnm::rx_rm_vlc(host, port,
        hl2ss::chunk_size::RM_VLC,        // chunk size
        hl2ss::stream_mode::MODE_0,       // Streaming mode (Video Only)
        1,                                // divisor (1 = full framerate - 30 FPS) 
        hl2ss::video_profile::H264_BASE,  // Video encoding profile
        hl2ss::h26x_level::H264_3,        // H.264 Level 3.0
        2*1024*1024                       // bitrate (2 Mbps)
    );

    int rotation = 0;
    switch (port) {
        case hl2ss::stream_port::RM_VLC_LEFTFRONT:
            rotation = cv::ROTATE_90_CLOCKWISE;
            break;
        case hl2ss::stream_port::RM_VLC_LEFTLEFT:
            rotation = cv::ROTATE_90_COUNTERCLOCKWISE;
            break;
        case hl2ss::stream_port::RM_VLC_RIGHTFRONT:
            rotation = cv::ROTATE_90_COUNTERCLOCKWISE;
            break;
        case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:
            rotation = cv::ROTATE_90_CLOCKWISE;
            break;
        default:
            std::cout << "Invalid port" << std::endl;
            return 0;
    }

    std::string port_name = hl2ss::get_port_name(port);

    // Rotated height and width because of later image rotation
    cv::Size frame_size(hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::WIDTH);
     std::shared_ptr<cv::VideoWriter> video_writer = std::make_shared<cv::VideoWriter>(
        output_path, 
        cv::VideoWriter::fourcc('M','J','P','G'), 
        hl2ss::parameters_rm_vlc::FPS, 
        frame_size, 
        false); // for grayscale
    
    // Check if video writer is opened
    if (!video_writer->isOpened()) {
        std::cout << "Error: Could not open video writer" << std::endl;
        return 0;
    }

    // Create Mat objects once, outside the loop
    cv::Mat mat_image(hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::WIDTH, CV_8UC1);
    cv::Mat mat_image_rotated;

    client->open();
    
    // Add ROS shutdown handler
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {
        std::shared_ptr<hl2ss::packet> data = client->get_next_packet();
        hl2ss::map_rm_vlc region = hl2ss::unpack_rm_vlc(data->payload.get());

        // Reuse mat_image with new data pointer
        mat_image.data = region.image;
        
        // Reuse mat_image_rotated for rotation
        cv::rotate(mat_image, mat_image_rotated, rotation);
        
        // Write frame to video file
        video_writer->write(mat_image_rotated);
        
        cv::imshow(port_name, mat_image_rotated);
        if ((cv::waitKey(1) & 0xFF) == 27) { break; }
    }
    
    video_writer->release();
    client->close();
    cv::destroyAllWindows();

    return 0;
}
