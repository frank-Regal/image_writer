#include <ros/ros.h>
#include <audio_common_msgs/AudioData.h>
#include <std_msgs/Empty.h>
#include <sndfile.h>
#include <chrono>
#include <filesystem>
#include <ros/package.h>
#include "hl2ss_lnm.h"                 // HoloLens streaming low-level networking
#include "hl2ss_mt.h"                  // HoloLens streaming multi-threading

// WAV file format
const int FORMAT_FLOAT_32 = SF_FORMAT_WAV | SF_FORMAT_FLOAT;
const int FORMAT_INT_16 = SF_FORMAT_WAV | SF_FORMAT_PCM_16;
const int FORMAT_UINT8 = SF_FORMAT_WAV | SF_FORMAT_PCM_U8;

class AudioWriter {
public:

    /**
     * @brief Construct a new Audio Writer object
     * 
     * @param Nh              - ros node handle
     * @param OutputDirectory - directory where to save the audio files
     * @param PostFix         - string to post append to output file name
     * @param TopicName       - ROS topic that the audio data is being published on
     * @param QueueSize       - the queue size for the subscriber listening to the audio data
     */
    AudioWriter(ros::NodeHandle Nh, std::string& OutputDirectory, std::string& PostFix, std::string& TopicName, int& QueueSize, std::string& NodeNs):
        nh_(Nh),
        file_timestamp_("new"),
        post_fix_(PostFix),
        output_directory_(OutputDirectory),
        topic_name_(TopicName),
        queue_size_(QueueSize),
        downsample_(false),
        is_open_for_recording_(false),
        node_ns_(NodeNs)
    {
        // load all params associated with encoding and formating for audio file
        // see params/params.yaml
        if(!LoadAudioParams()){ROS_ERROR("Params not configured correctly.");}
    }

    /**
     * @brief Destroy the Audio Writer object
     * 
     */
    ~AudioWriter() {
        CloseFile();
    }

    /**
     * @brief Configure node to handle multiple batches of audio messages
     * 
     * Configure the node to have to std_msgs/Empty ROS msgs publish to start
     * and stop the node when in batch mode.
     * Info: in batch mode you can publish multiple batches of audio messages
     * and the node with record to multiple audio files.
     * 
     * @param StartTopic - std_msgs/Empty topic
     * @param StopTopic  - std_msgs/Empty topic
     */
    void StartMultiBatchWriter(const std::string& StartTopic, const std::string StopTopic)
    {
        return;
    }

    /**
     * @brief Configure node to only one batch of audio messages
     * 
     */
    void StartSingleBatchWriter()
    {
        OpenFile();
    }

    
    /**
     * @brief Write ROS message to a audio file
     * 
     * This writes to the audio file with the same format and encoding as the input
     * 
     * @param msg 
     */
    void WriteToFileRaw(const float* data, const int& size) 
    {
        // o
        if (audio_file_) {
            sf_write_float(audio_file_, data, size);
        }
    }

    // Function to write audio data to a file
    void WriteInterleavedToFileRaw(const std::vector<float>& leftChannel, const std::vector<float>& rightChannel) {
        // Ensure both channels have the same number of samples
        if (leftChannel.size() != rightChannel.size()) {
            std::cerr << "Channel size mismatch!" << std::endl;
            return;
        }

        // Interleave the planar data into a single buffer
        std::vector<float> interleavedData(leftChannel.size() * 2);
        for (size_t i = 0; i < leftChannel.size(); ++i) {
            interleavedData[2 * i] = leftChannel[i];
            interleavedData[2 * i + 1] = rightChannel[i];
        }

        // Calculate the number of bytes to write
        std::cout << "Interleaved data size: " << interleavedData.size() << std::endl;
        sf_count_t bytes = interleavedData.size() * sizeof(float);

        // Write the interleaved data to the file
        if (audio_file_) {
            sf_count_t written = sf_write_float(audio_file_, interleavedData.data(), interleavedData.size());
            if (written != interleavedData.size()) {
                std::cerr << "Error writing audio data!" << std::endl;
            }
            std::cout << "Wrote " << written << " bytes to file" << std::endl;
        }
    }

    /**
     * @brief 
     * 
     * @param msg 
     */
    void WriteToFileDownsample(const float* data, const int& size) {
        std::cout << "WriteToFileDownsample" << std::endl;
        // make sure the file is open
        if (!is_open_for_recording_) return;

        // assuming the incoming audio is 32-bit float PCM with a 96000 sample rate
        int ratio = in_sample_rate_ / out_sample_rate_; // Basic ratio for downsampling; assumes divisibility

        // convert and downsample the incoming audio data
        std::vector<float> src_data(size / sizeof(float));
        memcpy(src_data.data(), data, size);

        // fill new array
        std::vector<short> target_data(src_data.size() / ratio);
        for (size_t i = 0; i < target_data.size(); ++i) {

            // naive downsampling - just picking every nth sample
            // TODO: consider averaging or using a resampling library like `libsamplerate`
            float sample = src_data[i * ratio];

            // Scale then normalize float values to signed 16-bit PCM encoding
            // note: maximum value of a 16-bit integer is 32767
            target_data[i] = static_cast<short>(sample * 32767.0f);
        }

        // Write downsampled data to file
        if (sf_write_short(audio_file_, target_data.data(), target_data.size()) != target_data.size()) {
            ROS_ERROR("Failed to write audio data to file. File closed.");
        }
    }

    bool use_param_server_ = false;
private:

    // init class variables
    std::string file_timestamp_, output_directory_, post_fix_, topic_name_, filename_, node_ns_;
    int queue_size_, channels_, in_sample_rate_, out_sample_rate_, format_;
    bool downsample_, is_open_for_recording_ ;

    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> subscribers_;
    SNDFILE* audio_file_;
    SF_INFO audio_file_info_;


    /**
     * @brief ROS callabck to open file for recording
     * 
     * when a std_msgs/Empty message is published, create and open an audio file.
     * 
     * @param Msg - std_msgs/Empty
     */
    void StartRecordingAudio(const std_msgs::Empty::ConstPtr& Msg)
    {
        OpenFile();
    }

    /**
     * @brief ROS callback to close file for recording
     * 
     * when a std_msgs/Empty message is published, close the audio file for writing
     * 
     * @param Msg - std_msgs/Empty
     */
    void StopRecordingAudio(const std_msgs::Empty::ConstPtr& Msg)
    {
        CloseFile();
    }


    /**
     * @brief Create and open audio file
     * 
     * 1) set audio file settings
     * 2) build the file name and define output directory
     * 3) open the file for writing. Shutdown node if the file is not open.
     * 
     */
    void OpenFile()
    {
        // make sure the last file is closed.
        if(is_open_for_recording_){CloseFile();}

        // set audio output file settings
        audio_file_info_.channels = channels_;
        audio_file_info_.samplerate = out_sample_rate_;
        audio_file_info_.format = format_;

        std::string timestamp, filepath;
        //  build file name
        if (use_param_server_){
            // set the file name
            std::string filename {"audio"};
            std::string filename_param {node_ns_ + "/filename"};
            if (!nh_.getParam(filename_param, filename)) {ROS_ERROR("Filename not set");}
            filename_ = filename + ".wav";
            filepath = output_directory_ + filename_;
            std::cout << "filepath: " << filepath << std::endl;
        }else{
            GetTimeStamp(timestamp);
            filename_ = timestamp + post_fix_ + ".wav";
            filepath = output_directory_ + filename_;
        }


        // open
        audio_file_ = sf_open(filepath.c_str(), SFM_WRITE, &audio_file_info_);
        
        // check
        if (!audio_file_) {
            ROS_ERROR("Failed to open '%s' file for writing.", filename_.c_str());
            ros::shutdown();
            return;
        }

        // log
        is_open_for_recording_ = true;
        //ROS_INFO("[audio_writer] opened: '%s' Writing ...", filepath.c_str());
        std::cout << "[audio_writer] opened: '" << filepath << "' Writing ..." << std::endl;
    }

    /**
     * @brief Close the audio file once finished.
     * 
     */
    void CloseFile()
    {
        if (audio_file_) {
            sf_close(audio_file_);
            is_open_for_recording_ = false;
            ROS_INFO("'%s' closed", filename_.c_str());
        }
    }

    /**
     * @brief Set the Audio Params object
     * 
     * set these parameters in 'params/params.yaml' file
     * 
     * @return true 
     * @return false 
     */
    bool LoadAudioParams()
    {
        // init
        std::string encoding {""};

        // load audio encoding params (defined in 'params/params.yaml')
        std::string param = node_ns_ + "/input/audio_encoding";
        std::cout << param << std::endl;
        if (!LoadParam<std::string>(param, encoding) ||
            !LoadParam<int>(node_ns_ + "/input/sample_rate", in_sample_rate_) ||
            !LoadParam<int>(node_ns_ + "/output/sample_rate", out_sample_rate_) ||
            !LoadParam<int>(node_ns_ + "/input/channels", channels_)) {
            return false;
        }

        // print loaded params
        std::cout << "Audio Parameters:" << std::endl;
        std::cout << "  Input:" << std::endl;
        std::cout << "    Encoding: " << encoding << std::endl; 
        std::cout << "    Sample Rate: " << in_sample_rate_ << " Hz" << std::endl;
        std::cout << "    Channels: " << channels_ << std::endl;
        std::cout << "  Output:" << std::endl;
        std::cout << "    Sample Rate: " << out_sample_rate_ << " Hz" << std::endl;

        // determine if downsampling is required.
        if (in_sample_rate_ > out_sample_rate_) {
            downsample_ = true;
            SetFormat("pcm_16", format_);
        } 
        else {
            SetFormat(encoding, format_);
        }

        return true;
    }

    /**
     * @brief Template function to load multiple params at once.
     * 
     * @tparam T             - paramater type
     * @param Param          - paramater you would like to load
     * @param ClassVariable  - variable to assign parameter to
     * @return true          - if param was successfully loaded
     * @return false         - if param could not be loaded
     */
    template<typename T>
    bool LoadParam(const std::string& Param, T& ClassVariable)
    {
        if (nh_.getParam(Param, ClassVariable)) {
            return true;
        } 
        else {
            ROS_ERROR("Error loading '%s' param. Check 'params/params.yaml' file.", Param.c_str());
            return false;
        }
    }

    /**
     * @brief Set the Format object
     * 
     * @param Encoding 
     * @param Format 
     */
    void SetFormat(const std::string& Encoding, int& Format)
    {
        // set the proper formating used for SndFile library.
        // these are defined at top. See SndFile library for more details.
        if(Encoding == "float") {
            Format = FORMAT_FLOAT_32;
        } 
        else if (Encoding == "pcm_16") {
            Format = FORMAT_INT_16;
        }
        else if (Encoding == "uint8") {
            Format = FORMAT_UINT8;
        }
        else {
            ROS_ERROR("audio_encoding format error. Current options: ['float', 'pcm_16']");
        }
    }

    /**
     * @brief Get current timestamp
     * 
     * @param TimeStamp - outputs the current ROS time
     */
    void GetTimeStamp(std::string& TimeStamp)
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S_");
        TimeStamp = ss.str();
    }
};

void splitChannels(const float* payload, size_t numSamples, std::vector<float>& leftChannel, std::vector<float>& rightChannel) {
    // Ensure the vectors are the correct size
    leftChannel.resize(numSamples / 2);
    rightChannel.resize(numSamples / 2);

    // Split the samples into left and right channels
    for (size_t i = 0; i < numSamples / 2; ++i) {
        leftChannel[i] = payload[i]; // First half for left channel
        rightChannel[i] = payload[i + numSamples / 2]; // Second half for right channel
    }
}


int main(int argc, char **argv) {

    // init
    ros::init(argc, argv, "save_raw_audio_stream");
    ros::NodeHandle nh;

    std::string node_ns = nh.getNamespace();

    // load params required to init audio writer class
    std::string output_directory_{ros::package::getPath("image_writer")}; 
    std::string post_fix_ {"audio"};
    std::string topic_name_ {"/hl2ss/microphone"};
    int queue_size_ {1};

    output_directory_ = output_directory_ + "/output/audio/";

    // if (nh.getParam("output_directory_", output_directory) && 
    //     nh.getParam("postfix_", post_fix) && 
    //     nh.getParam("topic_name_", topic_name) &&
    //     nh.getParam("queue_size_", queue_size)) {

    //     // if all params loaded
    //     ROS_INFO(
    //     "'save_raw_audio_stream' node configurations:\n\tSubscribing to: '%s'\n\tSaving audio files to: '%s'\n\tFiles will be post fixed with: '%s'", 
    //     topic_name.c_str(), 
    //     output_directory.c_str(),
    //     post_fix.c_str()); 
    // } 
    // else {

    //     // stop node if params were not loaded
    //     ROS_ERROR("Failed to load 'output_directory' and 'post_fixed' params. Shutting down node ...");
    //     ros::shutdown();
    //     return 1;
    // }

    std::filesystem::create_directories(output_directory_);

    // create audio writer class object.
    AudioWriter audio_writer(nh, output_directory_, post_fix_, topic_name_, queue_size_, node_ns);

    // determine the node setup. If start and stop topics are configured on the param server,
    // then set up the node to start and stop recording audio messages when the empty msgs are published.
    std::string start_recording_topic, stop_recording_topic;
    nh.getParam("use_param_server", audio_writer.use_param_server_);

    // if (nh.getParam("start_recording_topic_", start_recording_topic) &&
    //     nh.getParam("stop_recording_topic_", stop_recording_topic)) {
        
    //     // if topics are configured on param server start multi batch writer
    //     audio_writer.StartMultiBatchWriter(start_recording_topic, stop_recording_topic);
    //     ROS_WARN(
    //         "'save_raw_audio_stream' configured for multi-batch writing. Listening to start recording topic ['%s'] and end recording topic ['%s']",
    //         start_recording_topic.c_str(),
    //         stop_recording_topic.c_str());
    // } 
    // else {
    //     ROS_WARN("'save_raw_audio_stream' configured for single-batch writing.");
    //     // if no start and stop topics are configured on the param server, setup single batch writer.
    //     audio_writer.StartSingleBatchWriter();
    // }

    const char* host = "192.168.50.33";
    uint16_t port = hl2ss::stream_port::MICROPHONE;
    const uint8_t profile = hl2ss::audio_profile::AAC_24000;
    const uint64_t chunk = hl2ss::chunk_size::SINGLE_TRANSFER;
    const uint8_t level = hl2ss::aac_level::L4;
    const uint16_t group_size = 4096; //hl2ss::parameters_microphone::GROUP_SIZE_AAC;
    std::unique_ptr<hl2ss::rx_microphone> client = hl2ss::lnm::rx_microphone(host, port, profile);
    std::string port_name = hl2ss::get_port_name(port);

    client->open();

    // Start ROS spinner for asynchronous callbacks
    ros::AsyncSpinner spinner(1);
    spinner.start();

    audio_writer.StartSingleBatchWriter();

    while (ros::ok()) {
        
        std::shared_ptr<hl2ss::packet> data = client->get_next_packet();
        hl2ss::map_microphone_aac region = hl2ss::unpack_microphone_aac(data->payload.get());

        // Can't directly get size of float* array, need to use data->sz_payload
        // std::cout << "region.samples data: " << region.samples << std::endl;
        // std::cout << "data->timestamp: " << data->timestamp << std::endl;
        std::cout << "data->sz_payload: " << data->sz_payload << std::endl;
        size_t float_size_bytes = sizeof(float); // number of bytes in a float
        size_t float_size_bits = float_size_bytes * 8; // 8 bits in 1 byte.
        size_t num_samples = data->sz_payload / float_size_bits; // number of samples in the audio stream
        std::cout << "number of samples: " << num_samples << std::endl;

        // Example audio data
        std::vector<float> left_channel_bits;
        std::vector<float> right_channel_bits;
        splitChannels(region.samples, num_samples, left_channel_bits, right_channel_bits);

        audio_writer.WriteInterleavedToFileRaw(left_channel_bits, right_channel_bits);
        // std::cout << "wrote to file" << std::endl;
    }

    client->close();
    
    return 0;
}