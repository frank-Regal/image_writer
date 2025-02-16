// Standard library includes
#include <string>        
#include <filesystem>    
#include <cassert>       
#include <iostream>      
#include <opencv2/highgui.hpp>  
#include <vector>        
#include <memory>        
#include <chrono>
#include <iomanip>
#include <sstream>

// Project-specific includes
#include "Public/ImageWriter.h"         
#include "Public/ImageWriterParams.h"   
#include "hl2ss_lnm.h"                 
#include "hl2ss_mt.h"                  
#include "std_msgs/Empty.h"

/**
 * Structure to hold all necessary data for a single camera stream
 */
struct CameraStream {
    uint16_t port;                                    // Port number for the camera
    std::string name;                                 // Name of the camera stream
    std::string output_path;                          // Path where video will be saved
    std::unique_ptr<hl2ss::mt::source> source;        // Source for receiving camera data
    std::shared_ptr<cv::VideoWriter> video_writer;    // OpenCV video writer
    cv::Mat mat_image;                                // Original image matrix
    cv::Mat mat_image_rotated;                        // Rotated image matrix
    int64_t frame_index{-1};                          // Current frame index
    int64_t current_index{-1};                        // Last processed frame index
    int rotation;                                     // Rotation angle for this camera
    bool is_recording{false};                         // Flag to control recording state
};

class StreamManager {
public:
    StreamManager(ros::NodeHandle& nh) : nh_(nh) {
        initializeParameters();
        setupStreams();
        setupSubscribers();
    }

    void run() {
        ros::AsyncSpinner spinner(1);
        spinner.start();

        bool running = true;
        while (running && ros::ok()) {
            processStreams(running);
        }
    }

private:
    ros::NodeHandle& nh_;
    std::vector<CameraStream> streams_;
    std::string filepath_;
    std::string filename_;
    bool save_multi_stream_in_sequence_;
    bool use_param_server_;
    bool show_streams_;
    int fps_;
    std::string device_id_;
    const char* host_{"192.168.0.22"};
    uint64_t buffer_size_{10};

    void initializeParameters() {
        filepath_ = ros::package::getPath("image_writer");
        filename_ = "video.mp4";
        save_multi_stream_in_sequence_ = false;
        use_param_server_ = false;
        show_streams_ = true;
        fps_ = hl2ss::parameters_rm_vlc::FPS;
        device_id_ = "";

        std::string node_ns = nh_.getNamespace();

        nh_.param<std::string>("filepath", filepath_, filepath_);
        nh_.param<std::string>("filename", filename_, filename_);
        nh_.param<bool>("save_multi_stream_in_sequence", save_multi_stream_in_sequence_, save_multi_stream_in_sequence_);
        nh_.param<bool>("use_param_server", use_param_server_, use_param_server_);
        nh_.param<bool>("show_streams", show_streams_, show_streams_);
        nh_.param<int>("fps", fps_, fps_);
    }

    void setupStreams() {
        hl2ss::client::initialize();

        CameraStream lf_stream{hl2ss::stream_port::RM_VLC_LEFTFRONT};
        CameraStream rf_stream{hl2ss::stream_port::RM_VLC_RIGHTFRONT};
        CameraStream ll_stream{hl2ss::stream_port::RM_VLC_LEFTLEFT};
        CameraStream rr_stream{hl2ss::stream_port::RM_VLC_RIGHTRIGHT};

        setupCameraStream(lf_stream);
        setupCameraStream(rf_stream);
        setupCameraStream(ll_stream);
        setupCameraStream(rr_stream);

        streams_.push_back(std::move(lf_stream));
        streams_.push_back(std::move(rf_stream));
        streams_.push_back(std::move(ll_stream));
        streams_.push_back(std::move(rr_stream));
    }

    void setupSubscribers() {
        nh_.subscribe<std_msgs::Empty>("/hri_cacti/dataset_capture/start", 1, 
            &StreamManager::startRecordingCallback, this);
        nh_.subscribe<std_msgs::Empty>("/hri_cacti/dataset_capture/stop", 1,
            &StreamManager::stopRecordingCallback, this);
    }

    std::string getTimestampedFilename(const std::string& base, const std::string& ext) {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << base << "_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") << ext;
        return ss.str();
    }

    int GetRotation(uint16_t port) {
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

    std::unique_ptr<hl2ss::rx_rm_vlc> createClient(uint16_t port) {
        return hl2ss::lnm::rx_rm_vlc(host_, port,
            hl2ss::chunk_size::RM_VLC,
            hl2ss::stream_mode::MODE_0,
            1,
            hl2ss::video_profile::H264_BASE,
            hl2ss::h26x_level::H264_3,
            2*1024*1024
        );
    }

    void setupCameraStream(CameraStream& stream) {
        stream.name = hl2ss::get_port_name(stream.port);
        std::string base_filename = filename_.substr(0, filename_.find_last_of("."));
        std::string extension = filename_.substr(filename_.find_last_of("."));
        std::string timestamped_filename = getTimestampedFilename(base_filename, extension);
        
        stream.output_path = filepath_ + "/output/" + stream.name + "/" + timestamped_filename;
        stream.rotation = GetRotation(stream.port);
        
        std::filesystem::create_directories(filepath_ + "/output/" + stream.name + "/");
        if (show_streams_) {
            cv::namedWindow(stream.name);
        }
        
        auto client = createClient(stream.port);
        stream.source = std::make_unique<hl2ss::mt::source>(buffer_size_*fps_, std::move(client));
        stream.source->start();
        
        stream.video_writer = std::make_shared<cv::VideoWriter>();
    }

    void startRecording(CameraStream& stream) {
        if (!stream.is_recording) {
            cv::Size frame_size(hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::WIDTH);
            stream.video_writer->open(
                stream.output_path,
                cv::VideoWriter::fourcc('M','J','P','G'),
                fps_,
                frame_size,
                false  // Grayscale video
            );
            stream.is_recording = true;
        }
    }

    void stopRecording(CameraStream& stream) {
        if (stream.is_recording) {
            stream.video_writer->release();
            stream.is_recording = false;
        }
    }

    void startRecordingCallback(const std_msgs::Empty::ConstPtr& msg) {
        for (auto& stream : streams_) {
            startRecording(stream);
        }
    }

    void stopRecordingCallback(const std_msgs::Empty::ConstPtr& msg) {
        for (auto& stream : streams_) {
            stopRecording(stream);
        }
    }
    
    void processFrame(CameraStream& stream, std::shared_ptr<hl2ss::packet> data) {
        if (stream.current_index != stream.frame_index) {
            hl2ss::map_rm_vlc region = hl2ss::unpack_rm_vlc(data->payload.get());
            stream.mat_image.data = region.image;
            cv::rotate(stream.mat_image, stream.mat_image_rotated, stream.rotation);
            if (stream.is_recording ) {
                if (!stream.mat_image_rotated.empty()) {
                    stream.video_writer->write(stream.mat_image_rotated);
                }else{
                    std::cout << "Warning: Empty image data in frame for " << stream.name << std::endl;
                }
            }
        }
        stream.current_index = stream.frame_index;
        if (show_streams_) {
            cv::imshow(stream.name, stream.mat_image_rotated);
        }
    }

    void processStreams(bool& running) {
        int wait_key_ms = 1;
        std::exception error;

        // Check health of all streams
        for (auto& stream : streams_) {
            if (!stream.source->status(error)) { 
                running = false;
                throw error; 
            }
            stream.frame_index = -1;
        }

        int32_t status;
        auto& primary_stream = streams_[0];  // First stream is primary

        // Get frame from primary stream
        std::shared_ptr<hl2ss::packet> primary_data = primary_stream.source->get_packet(primary_stream.frame_index, status);

        if (status < 0) {
            // Frame too old, dropped from buffer
        }
        else if (status == 0) {
            // Process primary stream
            processFrame(primary_stream, primary_data);
            if (show_streams_ && (cv::waitKey(1) & 0xFF) == 27) { 
                running = false;
                return; 
            }

            // Process secondary streams
            for (size_t i = 1; i < streams_.size(); i++) {
                auto& stream = streams_[i];
                int32_t search_mode = hl2ss::mt::time_preference::PREFER_NEAREST;
                bool tiebreak_right = false;
                stream.frame_index = -1;

                // Get temporally matching frame from secondary stream
                std::shared_ptr<hl2ss::packet> data = stream.source->get_packet(
                    primary_data->timestamp, search_mode, tiebreak_right, stream.frame_index, status);

                if (data) {
                    processFrame(stream, data);
                    if (show_streams_ && (cv::waitKey(1) & 0xFF) == 27) {
                        running = false;
                        return;
                    }
                }
            }
        }
        else {
            wait_key_ms = 1000 / fps_;  // Adjust wait time based on FPS
        }

        if (show_streams_) {
            cv::waitKey(wait_key_ms);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "save_image_stream_hl2ss_mt");
    ros::NodeHandle nh;

    StreamManager manager(nh);
    manager.run();

    return 0;
}
