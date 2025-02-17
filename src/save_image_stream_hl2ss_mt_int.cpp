//------------------------------------------------------------------------------
// Standard library includes for basic functionality
//------------------------------------------------------------------------------
#include <string>        // For string handling
#include <filesystem>    // For filesystem operations
#include <cassert>       // For assertions
#include <iostream>      // For console I/O
#include <opencv2/highgui.hpp>  // For OpenCV GUI functionality
#include <vector>        // For dynamic arrays
#include <memory>        // For smart pointers

//------------------------------------------------------------------------------
// Project-specific includes
//------------------------------------------------------------------------------
#include "Public/ImageWriter.h"         // Custom image writing functionality
#include "Public/ImageWriterParams.h"   // Parameters for image writing
#include "hl2ss_lnm.h"                 // HoloLens streaming low-level networking
#include "hl2ss_mt.h"                  // HoloLens streaming multi-threading
#include "ros/ros.h"                   // ROS core functionality
#include "std_msgs/Empty.h"            // ROS empty message type

/**
 * Structure to hold all necessary data for a single camera stream from the HoloLens
 * This encapsulates all the components needed to receive, process, and save video
 * from a single camera on the HoloLens device.
 */
struct CameraStream {
    uint16_t port;                                    // Network port number for the specific camera
    std::string name;                                 // Human-readable name of the camera stream
    std::string base_path;                            // Base path for saving video files
    std::string output_path;                          // Full path where video file will be saved
    std::unique_ptr<hl2ss::mt::source> source;        // Thread-safe source for receiving camera data
    std::shared_ptr<cv::VideoWriter> video_writer;    // OpenCV writer for saving video to disk
    cv::Mat mat_image;                                // Original unprocessed image matrix
    cv::Mat mat_image_rotated;                        // Image matrix after rotation correction
    int64_t frame_index{-1};                          // Index of current frame being processed
    int64_t current_index{-1};                        // Index of last successfully processed frame
    int rotation;                                     // Required rotation angle for this camera
};


/**
 * Class to handle ROS callbacks for starting and stopping video recording
 * Provides a thread-safe interface for controlling recording state through ROS messages
 */
class SaveImageStreamListener {

public:
    // Constructor initializes ROS node handle and prints status
    SaveImageStreamListener(ros::NodeHandle& nh) : nh_(nh) 
    {
        std::cout << "SaveImageStreamListener initialized" << std::endl;
    };

    // Callback triggered when start recording message is received
    void StartRecordingCallback(const std_msgs::Empty::ConstPtr& msg) {
        record_ = true;
        std::cout << "Recording started" << std::endl;
    }

    // Callback triggered when stop recording message is received
    void StopRecordingCallback(const std_msgs::Empty::ConstPtr& msg) {
        record_ = false;
        std::cout << "Recording stopped" << std::endl;
    }

    // Thread-safe getter for current recording state
    bool GetRecordingStatus() {
        return record_;
    }
    
private:
    ros::NodeHandle nh_;          // ROS node handle for communication
    bool record_ {false};         // Current recording state
};


/**
 * Determines the correct rotation angle for a given HoloLens camera
 * Different cameras on the HoloLens are mounted at different orientations
 * and need to be rotated to appear correctly in saved videos
 * 
 * @param port      The port number identifying the specific camera
 * @return          OpenCV rotation constant for the camera
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

/**
 * Creates a new HoloLens camera client with standardized streaming parameters
 * Configures video quality, framerate, and networking parameters for reliable streaming
 * 
 * @param host      IP address of the HoloLens device
 * @param port      Port number for the specific camera
 * @return          Unique pointer to the configured streaming client
 */
std::unique_ptr<hl2ss::rx_rm_vlc> create_client(const char* host, uint16_t port) {
    return hl2ss::lnm::rx_rm_vlc(host, port,
        hl2ss::chunk_size::RM_VLC,          // Standard chunk size for VLC streams
        hl2ss::stream_mode::MODE_0,         // Video-only mode
        1,                                  // Full framerate
        hl2ss::video_profile::H265_MAIN,    // Basic H264 profile for good compatibility
        hl2ss::h26x_level::DEFAULT          // H264 level 3 for decent quality
    );
}

/**
 * Initializes all components of a camera stream
 * Creates necessary directories, sets up display windows, initializes video writer,
 * and configures streaming client
 * 
 * @param stream        Reference to CameraStream structure to initialize
 * @param host          HoloLens IP address
 * @param filepath      Base path for saving video files
 * @param filename      Base name for video files
 * @param fps           Desired frames per second
 * @param buffer_size   Size of frame buffer for streaming
 * @param show_streams  Whether to display video streams in windows
 */
void setup_camera_stream(CameraStream& stream, 
                        const char* host, 
                        const std::string& stream_name, 
                        const int fps, 
                        const uint64_t buffer_size, 
                        const bool& show_streams) {
    
    // Set the stream name if provided, otherwise use the port name
    stream.name = (!stream_name.empty()) ? stream_name : hl2ss::get_port_name(stream.port);

    // Get the rotation angle for the stream (this is camera dependent and flips the image to be correct)
    stream.rotation = GetRotation(stream.port);
    
    if (show_streams) {
        cv::namedWindow(stream.name);
    }

    // Create a video writer object for this stream
    stream.video_writer = std::make_shared<cv::VideoWriter>();
    
    // Initialize image matrix with correct dimensions
    stream.mat_image = cv::Mat(hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::WIDTH, CV_8UC1);
    
    // Set up streaming client and prepare it
    auto client = create_client(host, stream.port);
    stream.source = std::make_unique<hl2ss::mt::source>(buffer_size*fps, std::move(client));
    // stream.source->start();  // Starting is deferred until recording begins

    std::cout << "Stream '" << stream.name << "' setup." << std::endl;
}

/**
 * Processes a single frame from a camera stream
 * Handles frame unpacking, rotation, writing to video file, and display
 * Only processes each unique frame once to avoid duplicates
 * 
 * @param stream        Reference to camera stream to process
 * @param data          Packet containing the frame data
 * @param show_streams  Whether to display video streams
 */
void process_frame(CameraStream& stream, std::shared_ptr<hl2ss::packet> data, bool show_streams) {
    // Only process new frames we haven't seen before
    if (stream.current_index != stream.frame_index) {
        hl2ss::map_rm_vlc region = hl2ss::unpack_rm_vlc(data->payload.get());
        stream.mat_image.data = region.image;
        cv::rotate(stream.mat_image, stream.mat_image_rotated, stream.rotation);
        if (stream.video_writer->isOpened()) {
            stream.video_writer->write(stream.mat_image_rotated);
        }
    }
    stream.current_index = stream.frame_index;
    if (show_streams) {
        cv::imshow(stream.name, stream.mat_image_rotated);
    }
}

/**
 * Creates and initializes a new video writer for a camera stream
 * Configures video format, codec, and dimensions
 * 
 * @param stream    Reference to camera stream needing new video writer
 * @param fps       Frames per second for the video
 * @throws std::runtime_error if video writer fails to open
 */
void create_new_video_writer(CameraStream& stream, int fps) {
    stream.video_writer->open(stream.output_path, cv::VideoWriter::fourcc('m','p','4','v'), fps, cv::Size(hl2ss::parameters_rm_vlc::HEIGHT, hl2ss::parameters_rm_vlc::WIDTH), false);
    if (!stream.video_writer->isOpened()) {
        std::cout << "Error: Could not open " << stream.name << " video writer" << std::endl;
        throw std::runtime_error("Failed to open video writer");
    } else {
        std::cout << "Opened " << stream.name << " video writer" << std::endl;
    }
}

/**
 * Generates a filename with current timestamp
 * Creates a unique filename by combining base name with current date and time
 * 
 * @param base     Base filename
 * @param ext      File extension (including dot)
 * @return         Complete filename with timestamp
 */
std::string getTimestampedFilename(const std::string& base, const std::string& ext) {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << base << "_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") << ext;
    return ss.str();
}


/**
 * Main entry point for the HoloLens video streaming and recording application
 * Initializes ROS node, sets up camera streams, and manages recording lifecycle
 * 
 * @param argc     Command line argument count
 * @param argv     Command line argument values
 * @return         0 on successful execution, non-zero on error
 */
int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "save_image_stream_hl2ss_mt_int");
    ros::NodeHandle nh;

    // Initialize parameters with default values
    std::string base_filepath {ros::package::getPath("image_writer")};
    std::string subdir_filepath {"output/advance"};
    std::string base_filename {"video"};
    std::string hololens_ip {"192.168.0.22"};
    std::string sensor_id {""};
    bool use_param_server {false};
    bool show_streams {true};  // New parameter to control stream display
    int fps {hl2ss::parameters_rm_vlc::FPS};
    
    
    std::string node_ns = nh.getNamespace();

    // Load parameters from ROS parameter server
    nh.param<std::string>("base_filepath", base_filepath, base_filepath);
    nh.param<std::string>("base_filename", base_filename, base_filename);
    nh.param<std::string>("hololens_ip", hololens_ip, hololens_ip);
    nh.param<std::string>("sensor_id", sensor_id, sensor_id);
    nh.param<bool>("use_param_server", use_param_server, use_param_server);
    nh.param<bool>("show_streams", show_streams, show_streams);  // Load show_streams parameter

    // Initialize HoloLens client
    // hl2ss::client::initialize();
      // HoloLens IP address
    const char* host = hololens_ip.c_str();
    uint64_t buffer_size = 10;

    // Vector to hold all camera streams
    std::vector<CameraStream> streams;
    
    // Initialize and set up all camera streams
    std::vector<uint16_t> ports = {
        hl2ss::stream_port::RM_VLC_LEFTFRONT,
        hl2ss::stream_port::RM_VLC_RIGHTFRONT, 
        hl2ss::stream_port::RM_VLC_LEFTLEFT,
        hl2ss::stream_port::RM_VLC_RIGHTRIGHT
    };

    for (auto port : ports) {
        CameraStream stream{port};
        setup_camera_stream(stream, host, sensor_id, fps, buffer_size, show_streams);
        streams.push_back(std::move(stream));
    }
    
    // Set up each camera stream
    // for (auto& stream : streams) {
    //     stream.base_path = base_filepath + "/" + subdir_filepath + "/" + stream.name + "/";
    //     std::filesystem::create_directories(stream.base_path);
    // }
    
    // Initialize recording listener
    SaveImageStreamListener RecordingListener(nh);
    ros::Subscriber start_sub = nh.subscribe<std_msgs::Empty>("/hri_cacti/dataset_capture/start", 1, &SaveImageStreamListener::StartRecordingCallback, &RecordingListener);
    ros::Subscriber stop_sub = nh.subscribe<std_msgs::Empty>("/hri_cacti/dataset_capture/stop", 1, &SaveImageStreamListener::StopRecordingCallback, &RecordingListener);

    // Start ROS spinner for asynchronous callbacks
    ros::AsyncSpinner spinner(1);
    spinner.start();

    bool recording = false;

    // Main processing loop
    while (ros::ok()) {
        int wait_key_ms = 1;
        std::exception error;

        if (RecordingListener.GetRecordingStatus()) {

            // If first time recording, setup all streams
            if (!recording) {
                for (auto& stream : streams) {

                    // Create a new video writer with the updated output path
                    nh.param<std::string>("subdir_filepath", subdir_filepath, subdir_filepath);
                    stream.base_path = base_filepath + "/" + subdir_filepath + "/" + stream.name + "/";
                    std::filesystem::create_directories(stream.base_path);
                    stream.output_path = stream.base_path + getTimestampedFilename(base_filename, ".mp4");
                    create_new_video_writer(stream, fps);
                    
                    // Start the stream
                    stream.source->start();

                    // Print the stream status
                    std::cout << "Started recording'" << stream.name << "' stream to: '" << stream.output_path << "'" << std::endl;
                }
                
                recording = true;
            }

            // Check health of all streams
            for (auto& stream : streams) {
                if (!stream.source->status(error)) {
                    std::cout << "Error: Stream " << stream.name << " has a problem" << std::endl;
                    if (stream.video_writer->isOpened()) {
                        stream.video_writer->release();
                    }
                    stream.source->stop();
                    std::cout << "Stream " << stream.name << " stopped" << std::endl;
                    return 0;
                }
                stream.frame_index = -1;
            }

            int32_t status;
            auto& primary_stream = streams[0];  // First stream is primary

            // Get frame from primary stream
            std::shared_ptr<hl2ss::packet> primary_data = primary_stream.source->get_packet(primary_stream.frame_index, status);

            if (status < 0) {
                // Frame too old, dropped from buffer
            }
            else if (status == 0) {
                process_frame(primary_stream, primary_data, show_streams);
                if (show_streams && (cv::waitKey(1) & 0xFF) == 27) { break; }  // Exit on ESC key
                // Process secondary streams
                for (size_t i = 1; i < streams.size(); i++) {
                    auto& stream = streams[i];
                    int32_t search_mode = hl2ss::mt::time_preference::PREFER_NEAREST;
                    bool tiebreak_right = false;
                    stream.frame_index = -1;
                    // Get temporally matching frame from secondary stream
                    std::shared_ptr<hl2ss::packet> data = stream.source->get_packet(
                        primary_data->timestamp, search_mode, tiebreak_right, stream.frame_index, status);
                    if (data) {
                        process_frame(stream, data, show_streams);
                        if (show_streams && (cv::waitKey(1) & 0xFF) == 27) { break; }
                    }
                }
            }
            else {
                wait_key_ms = 1000 / fps;  // Adjust wait time based on FPS
            }
        }
        else {
            recording = false;
            for (auto& stream : streams) {
                stream.source->stop();
                stream.video_writer->release();
            }
        }
    }

    // Cleanup resources
    for (auto& stream : streams) {
        if (stream.video_writer->isOpened()) {
            stream.video_writer->release();
        }
        stream.source->stop();
    }
    if (show_streams) {
        cv::destroyAllWindows();
    }

    // hl2ss::client::close();

    return 0;
}
