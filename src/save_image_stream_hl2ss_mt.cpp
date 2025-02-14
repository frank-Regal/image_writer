#include <string>
#include <filesystem>
#include <cassert>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <vector>
#include <memory>

#include "Public/ImageWriter.h"
#include "Public/ImageWriterParams.h"
#include "hl2ss_lnm.h"
#include "hl2ss_mt.h"

struct CameraStream {
    uint16_t port;
    std::string name;
    std::string output_path;
    std::unique_ptr<hl2ss::mt::source> source;
    std::shared_ptr<cv::VideoWriter> video_writer;
    cv::Mat mat_image;
    cv::Mat mat_image_rotated;
    int64_t frame_index{-1};
    int64_t current_index{-1};
    int rotation;
};

/**
 * Gets the rotation angle for a given camera port
 * @param port The port number of the camera
 * @return The OpenCV rotation constant for the camera
 */
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

std::unique_ptr<hl2ss::rx_rm_vlc> create_client(const char* host, uint16_t port) {
    return hl2ss::lnm::rx_rm_vlc(host, port,
        hl2ss::chunk_size::RM_VLC,
        hl2ss::stream_mode::MODE_0,
        1,
        hl2ss::video_profile::H264_BASE,
        hl2ss::h26x_level::H264_3,
        2*1024*1024
    );
}

void setup_camera_stream(CameraStream& stream, const char* host, const std::string& filepath, 
                        const std::string& filename, int fps, uint64_t buffer_size) {
    stream.name = hl2ss::get_port_name(stream.port);
    stream.output_path = filepath + "/output/" + stream.name + "/" + filename;
    stream.rotation = GetRotation(stream.port);
    
    std::filesystem::create_directories(filepath + "/output/" + stream.name + "/");
    cv::namedWindow(stream.name);
    
    auto client = create_client(host, stream.port);
    stream.source = std::make_unique<hl2ss::mt::source>(buffer_size*fps, std::move(client));
    stream.source->start();
    
    cv::Size frame_size(hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::WIDTH);
    stream.video_writer = std::make_shared<cv::VideoWriter>(
        stream.output_path,
        cv::VideoWriter::fourcc('M','J','P','G'),
        fps,
        frame_size,
        false
    );
    
    if (!stream.video_writer->isOpened()) {
        std::cout << "Error: Could not open " << stream.name << " video writer" << std::endl;
        throw std::runtime_error("Failed to open video writer");
    }
    
    stream.mat_image = cv::Mat(hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::WIDTH, CV_8UC1);
}

void process_frame(CameraStream& stream, std::shared_ptr<hl2ss::packet> data) {
    if (stream.current_index != stream.frame_index) {
        hl2ss::map_rm_vlc region = hl2ss::unpack_rm_vlc(data->payload.get());
        stream.mat_image.data = region.image;
        cv::rotate(stream.mat_image, stream.mat_image_rotated, stream.rotation);
        stream.video_writer->write(stream.mat_image_rotated);
        // std::cout << "Wrote Frame to video for " << stream.name << std::endl;
    }
    stream.current_index = stream.frame_index;
    cv::imshow(stream.name, stream.mat_image_rotated);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "save_image_stream_hl2ss");
    ros::NodeHandle nh;

    std::string filepath {ros::package::getPath("image_writer")};
    std::string filename {"video.mp4"};
    bool save_multi_stream_in_sequence {false};
    bool use_param_server {false};
    int fps {hl2ss::parameters_rm_vlc::FPS};
    std::string device_id {""};
    
    std::string node_ns = nh.getNamespace();

    nh.param<std::string>("filepath", filepath, filepath);
    nh.param<std::string>("filename", filename, filename);
    nh.param<bool>("save_multi_stream_in_sequence", save_multi_stream_in_sequence, save_multi_stream_in_sequence);
    nh.param<bool>("use_param_server", use_param_server, use_param_server);
    nh.param<int>("fps", fps, fps);

    ImageWriterParams image_writer_params(filepath, fps, save_multi_stream_in_sequence, nh, node_ns);
    ImageWriter image_writer(filepath, fps, save_multi_stream_in_sequence);

    hl2ss::client::initialize();
    const char* host {"192.168.50.33"};
    uint64_t buffer_size = 10;

    std::vector<CameraStream> streams;
    
    // Initialize all camera streams
    CameraStream lf_stream{hl2ss::stream_port::RM_VLC_LEFTFRONT};
    CameraStream rf_stream{hl2ss::stream_port::RM_VLC_RIGHTFRONT};
    CameraStream ll_stream{hl2ss::stream_port::RM_VLC_LEFTLEFT};
    CameraStream rr_stream{hl2ss::stream_port::RM_VLC_RIGHTRIGHT};
    setup_camera_stream(lf_stream, host, filepath, filename, fps, buffer_size);
    setup_camera_stream(rf_stream, host, filepath, filename, fps, buffer_size);
    setup_camera_stream(ll_stream, host, filepath, filename, fps, buffer_size);
    setup_camera_stream(rr_stream, host, filepath, filename, fps, buffer_size);
    streams.push_back(std::move(lf_stream));
    streams.push_back(std::move(rf_stream));
    streams.push_back(std::move(ll_stream));
    streams.push_back(std::move(rr_stream));

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok()) {
        int wait_key_ms = 1;
        std::exception error;

        // Check all stream statuses
        for (auto& stream : streams) {
            if (!stream.source->status(error)) { throw error; }
            stream.frame_index = -1;
        }

        int32_t status;
        auto& primary_stream = streams[0]; // Use first stream as primary

        std::shared_ptr<hl2ss::packet> primary_data = primary_stream.source->get_packet(primary_stream.frame_index, status);

        if (status < 0) {
             // Requested frame is too old and has been dropped from the buffer (data_pv is null)
        }
        else if (status == 0) {
            process_frame(primary_stream, primary_data);
            if ((cv::waitKey(1) & 0xFF) == 27) { break; }

            // Process other streams
            for (size_t i = 1; i < streams.size(); i++) {
                auto& stream = streams[i];
                int32_t search_mode = hl2ss::mt::time_preference::PREFER_NEAREST;
                bool tiebreak_right = false;
                stream.frame_index = -1;

                std::shared_ptr<hl2ss::packet> data = stream.source->get_packet(
                    primary_data->timestamp, search_mode, tiebreak_right, stream.frame_index, status);

                if (data) {
                    process_frame(stream, data);
                    if ((cv::waitKey(1) & 0xFF) == 27) { break; }
                }
            }
        }
        else {
            wait_key_ms = 1000 / fps;
        }
    }

    // Cleanup
    for (auto& stream : streams) {
        stream.video_writer->release();
        stream.source->stop();
    }
    cv::destroyAllWindows();

    return 0;
}
