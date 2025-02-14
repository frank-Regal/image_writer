#include <string>
#include <filesystem>
#include <cassert>
#include <iostream>
#include <opencv2/highgui.hpp>

#include "Public/ImageWriter.h"
#include "Public/ImageWriterParams.h"
#include "hl2ss_lnm.h"
#include "hl2ss_mt.h"


int GetRotation(uint16_t port) 
{
    switch (port) {
        case hl2ss::stream_port::RM_VLC_LEFTFRONT:
            return cv::ROTATE_90_CLOCKWISE;
        case hl2ss::stream_port::RM_VLC_LEFTLEFT:
            return cv::ROTATE_90_COUNTERCLOCKWISE;
        case hl2ss::stream_port::RM_VLC_RIGHTFRONT:
            return cv::ROTATE_90_COUNTERCLOCKWISE;
        case hl2ss::stream_port::RM_VLC_RIGHTRIGHT:
            return cv::ROTATE_90_CLOCKWISE;
        default:
            std::cout << "Invalid port" << std::endl;
            return 0;
    }
}

int main(int argc, char** argv) {

    // init defaults
    ros::init(argc, argv, "save_image_stream_hl2ss");
    ros::NodeHandle nh;

    // init parameters
    std::string filepath {ros::package::getPath("image_writer")};
    std::string filename {"video.mp4"};
    bool save_multi_stream_in_sequence {false};
    bool use_param_server {false};
    int fps {hl2ss::parameters_rm_vlc::FPS};
    std::string device_id {""};

    std::string output_path_lf {filepath + "/output/lf/" + filename};
    std::string output_path_rf {filepath + "/output/rf/" + filename};

    // Get the namespace of the node
    std::string node_ns = nh.getNamespace();

    nh.param<std::string>("filepath", filepath, filepath);
    nh.param<std::string>("filename", filename, filename);
    nh.param<bool>("save_multi_stream_in_sequence", save_multi_stream_in_sequence, save_multi_stream_in_sequence);
    nh.param<bool>("use_param_server", use_param_server, use_param_server);
    nh.param<int>("fps", fps, fps);

    // create directories if they do not exist
    std::filesystem::create_directories(filepath + "/output/lf/");
    std::filesystem::create_directories(filepath + "/output/rf/");

    // init objectsß
    ImageWriterParams image_writer_params(filepath, fps, save_multi_stream_in_sequence, nh, node_ns);
    ImageWriter image_writer(filepath, fps, save_multi_stream_in_sequence);

    hl2ss::client::initialize();

    // init hl2ss (TODO: make this a parameter)
    char const* host {"192.168.50.33"};
    const uint16_t port_vlc_lf {hl2ss::stream_port::RM_VLC_LEFTFRONT};
    const uint16_t port_vlc_rf {hl2ss::stream_port::RM_VLC_RIGHTFRONT};

    std::string vlc_lf_name = hl2ss::get_port_name(hl2ss::stream_port::RM_VLC_LEFTFRONT);
    std::string vlc_rf_name = hl2ss::get_port_name(hl2ss::stream_port::RM_VLC_RIGHTFRONT);

    cv::namedWindow(vlc_lf_name);
    cv::namedWindow(vlc_rf_name);
    
    // Create client
    std::unique_ptr<hl2ss::rx_rm_vlc> client_vlc_lf = hl2ss::lnm::rx_rm_vlc(host, port_vlc_lf,
        hl2ss::chunk_size::RM_VLC,        // chunk size
        hl2ss::stream_mode::MODE_0,       // Streaming mode (Video Only)
        1,                                // divisor (1 = full framerate - 30 FPS) 
        hl2ss::video_profile::H264_BASE,  // Video encoding profile
        hl2ss::h26x_level::H264_3,        // H.264 Level 3.0
        2*1024*1024                       // bitrate (2 Mbps)
    );

    std::unique_ptr<hl2ss::rx_rm_vlc> client_vlc_rf = hl2ss::lnm::rx_rm_vlc(host, port_vlc_rf,
        hl2ss::chunk_size::RM_VLC,        // chunk size
        hl2ss::stream_mode::MODE_0,       // Streaming mode (Video Only)
        1,                                // divisor (1 = full framerate - 30 FPS) 
        hl2ss::video_profile::H264_BASE,  // Video encoding profile
        hl2ss::h26x_level::H264_3,        // H.264 Level 3.0
        2*1024*1024                       // bitrate (2 Mbps)
    );

        
    // Buffer size in seconds
    uint64_t buffer_size = 10;

    std::unique_ptr<hl2ss::mt::source> source_vlc_lf = std::make_unique<hl2ss::mt::source>(buffer_size*fps, std::move(client_vlc_lf));
    std::unique_ptr<hl2ss::mt::source> source_vlc_rf = std::make_unique<hl2ss::mt::source>(buffer_size*fps, std::move(client_vlc_rf));

    // Open the sources
    source_vlc_lf->start();
    source_vlc_rf->start();

    // Get the rotation for the left and right streams
    int rotation_lf = GetRotation(port_vlc_lf);
    int rotation_rf = GetRotation(port_vlc_rf);

    // // Create the video writers
    // std::string port_name = hl2ss::get_port_name(port);

    // Rotated height and width because of later image rotation
    cv::Size frame_size(hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::WIDTH);
    std::shared_ptr<cv::VideoWriter> video_writer_lf = std::make_shared<cv::VideoWriter>(
        output_path_lf, 
        cv::VideoWriter::fourcc('M','J','P','G'), 
        fps, 
        frame_size, 
        false); // for grayscale
    
    // Check if video writer is opened
    if (!video_writer_lf->isOpened()) {
        std::cout << "Error: Could not open lf video writer" << std::endl;
        return 0;
    }

    std::shared_ptr<cv::VideoWriter> video_writer_rf = std::make_shared<cv::VideoWriter>(
        output_path_rf, 
        cv::VideoWriter::fourcc('M','P','4','V'), 
        30, 
        frame_size, 
        false); // for grayscale

    // Check if video writer is opened
    if (!video_writer_rf->isOpened()) {
        std::cout << "Error: Could not open rf video writer" << std::endl;
        return 0;
    }
    // // Create Mat objects once, outside the loop
    cv::Mat vlc_lf_mat_image(hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::WIDTH, CV_8UC1);
    cv::Mat vlc_lf_mat_image_rotated;

    cv::Mat vlc_rf_mat_image(hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::WIDTH, CV_8UC1);
    cv::Mat vlc_rf_mat_image_rotated;

    // client->open();
    
    // Add ROS shutdown handler
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // int64_t vlc_lf_frame_index = -1;
    // int32_t vlc_rf_frame_index = 0;
    int64_t vlc_lf_frame_index{-1};
    int64_t vlc_rf_frame_index{-1};
    int64_t current_vlc_lf_index{-1};
    int64_t current_vlc_rf_index{-1};

    while (ros::ok())
    
    {

        int wait_key_ms = 1;

        std::exception error;
        if (!source_vlc_lf->status(error)) { throw error; }
        vlc_lf_frame_index = -1;
        if (!source_vlc_rf->status(error)) { throw error; }
        vlc_rf_frame_index = -1;

        // Get PV frame by index
        // Return value: 0 if frame retrieved successfully
        int32_t vlc_lf_status;
        int32_t vlc_rf_status;

        // Alternatively pass index -1 for most recent frame, -2 for second most recent frame, etc., will repeat/drop frames if necessary
        std::shared_ptr<hl2ss::packet> data_vlc_lf = source_vlc_lf->get_packet(vlc_lf_frame_index, vlc_lf_status);
        // std::shared_ptr<hl2ss::packet> data_vlc_rf = source_vlc_rf->get_packet(vlc_rf_frame_index, vlc_rf_status);

        // std::shared_ptr<hl2ss::packet> data = client->get_next_packet();
        // hl2ss::map_rm_vlc region = hl2ss::unpack_rm_vlc(data->payload.get());

        // // Reuse mat_image with new data pointer
        // mat_image.data = region.image;
        
        // // Reuse mat_image_rotated for rotation
        // cv::rotate(mat_image, mat_image_rotated, rotation);
        
        // // Write frame to video file
        // video_writer->write(mat_image_rotated);
        
        // cv::imshow(port_name, mat_image_rotated);
        // if ((cv::waitKey(1) & 0xFF) == 27) { break; }

        if (vlc_lf_status < 0) 
        {
            // Requested frame is too old and has been dropped from the buffer (data_pv is null)
            // Advance to next frame
            // vlc_lf_frame_index++;
            std::cout << "STATUS IS NEGATIVE: " << vlc_lf_status << " lf frame index: " << vlc_lf_frame_index << std::endl;
        }
        else if (vlc_lf_status == 0)
        {
            std::cout << "vlc_lf_status: " << vlc_lf_status << " lf frame index: " << vlc_lf_frame_index << " timestamp: " << data_vlc_lf->timestamp << std::endl;
            if (current_vlc_lf_index != vlc_lf_frame_index){
                // Frame retrieved successfully
                hl2ss::map_rm_vlc region_vlc_lf = hl2ss::unpack_rm_vlc(data_vlc_lf->payload.get());
                vlc_lf_mat_image.data = region_vlc_lf.image;
                cv::rotate(vlc_lf_mat_image, vlc_lf_mat_image_rotated, rotation_lf);
                video_writer_lf->write(vlc_lf_mat_image_rotated);
                std::cout << "Wrote Frame to video" << std::endl;
            }
            current_vlc_lf_index = vlc_lf_frame_index;
            cv::imshow(vlc_lf_name, vlc_lf_mat_image_rotated);
            if ((cv::waitKey(1) & 0xFF) == 27) { break; }

            int32_t search_mode = hl2ss::mt::time_preference::PREFER_NEAREST; 

            // Choose frame with timestamp > data->timestamp if search mode is NEAREST and the two nearest frames are at the distance
            bool tiebreak_right = false;

            // Return value: frame_index of the returned frame
            vlc_rf_frame_index = -1;

            // Get depth frame
            std::shared_ptr<hl2ss::packet> data_vlc_rf = source_vlc_rf->get_packet(data_vlc_lf->timestamp, search_mode, tiebreak_right, vlc_rf_frame_index, vlc_rf_status);

            if (data_vlc_rf)
            {
                if (current_vlc_rf_index != vlc_rf_frame_index){
                    // Frame retrieved successfully
                    hl2ss::map_rm_vlc region_vlc_rf = hl2ss::unpack_rm_vlc(data_vlc_rf->payload.get());
                    vlc_rf_mat_image.data = region_vlc_rf.image;
                    cv::rotate(vlc_rf_mat_image, vlc_rf_mat_image_rotated, rotation_rf);
                    video_writer_rf->write(vlc_rf_mat_image_rotated);
                    std::cout << "Wrote Frame to video" << std::endl;
                }
                current_vlc_rf_index = vlc_rf_frame_index;
                cv::imshow(vlc_rf_name, vlc_rf_mat_image_rotated);
                if ((cv::waitKey(1) & 0xFF) == 27) { break; }
            }
        }

        // if (vlc_rf_status < 0) 
        // {
        //     // Requested frame is too old and has been dropped from the buffer (data_pv is null)
        //     // Advance to next frame
        //     vlc_rf_frame_index++;
        // }
        // else if (vlc_rf_status == 0)
        // {   
        //     // Frame retrieved successfully
        //     hl2ss::map_rm_vlc region_vlc_rf = hl2ss::unpack_rm_vlc(data_vlc_rf->payload.get());
        //     vlc_rf_mat_image.data = region_vlc_rf.image;
        //     cv::rotate(vlc_rf_mat_image, vlc_rf_mat_image_rotated, rotation_rf);
        //     video_writer->write(vlc_rf_mat_image_rotated);
        //     cv::imshow(vlc_rf_name, vlc_rf_mat_image_rotated);
        //     if ((cv::waitKey(1) & 0xFF) == 27) { break; }
        // }

        else // pv_status > 0 
        {
            // Requested frame has not been received from the server yet (data_pv is null)
            // Do not advance to next frame
            // Wait 1 frame in ms
            wait_key_ms = 1000 / fps;
        }
    }
    
    video_writer_lf->release();
    // video_writer_rf->release();
    source_vlc_lf->stop();
    // source_vlc_rf->stop();
    cv::destroyAllWindows();

    return 0;
}
