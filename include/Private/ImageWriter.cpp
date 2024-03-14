#include "Public/ImageWriter.h"

/** Constructor.
 * 
 * Image Saver class constructor
 *
 * @param output_path path on machine to save file
 * @param fps input frames per second
 */
ImageWriter::ImageWriter(std::string output_path, int fps) :
            output_path_(output_path),
            orig_output_path_(output_path),
            time_stamp_prefix_(""),
            is_timestamp_set_(false),
            fps_(fps),
            video_writer_(nullptr),
            save_multi_stream_in_sequence_(false) {}

/** Constructor overloaded.
 * 
 * Image Saver class constructor. Pass a boolean to indicate you want to process multiple images.
 *
 * @param output_path path on machine to save file
 * @param fps input frames per second
 * @param save_multi_stream_in_sequence pass true here to use multi image processing functionality
 */
ImageWriter::ImageWriter(std::string output_path, int fps, bool save_multi_stream_in_sequence) :
            output_path_(output_path),
            orig_output_path_(output_path),
            time_stamp_prefix_(""),
            is_timestamp_set_(false),
            fps_(fps),
            video_writer_(nullptr),
            save_multi_stream_in_sequence_(save_multi_stream_in_sequence) {}

/** Deconstructor.
 * 
 * Image Saver class deconstructor
 *
 */
ImageWriter::~ImageWriter()
{
    delete(video_writer_);
}

/** ROS image stream callback function.
 * 
 * This saves a ROS raw image stream to a video file.
 *
 * @param msg input sensor_msgs/Image message
 * @return void
 */
void ImageWriter::imageCallback(const sensor_msgs::Image::ConstPtr& msg) 
{
    try 
    {
        // set timestamp for this sequence of videos
        if(!is_timestamp_set_) 
        {
            getTimeStamp(time_stamp_prefix_);
            is_timestamp_set_ = true;
        }

        // convert ROS Image message to OpenCV image
        cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image;

        // check if RGB
        bool isColor = (image.type() == CV_8UC3); 

        // create video writer if it doesn't exist
        if (video_writer_ == nullptr) 
        {
            if(save_multi_stream_in_sequence_){updateOutputPath();}

            // Create instance setup and open video writer 
            video_writer_ = new cv::VideoWriter();
            int codec = cv::VideoWriter::fourcc('G','R','E','Y'); // set  codec
            video_writer_->open(output_path_, codec, fps_, image.size(), isColor);

            // check for success
            if (!video_writer_->isOpened()) {
                std::cerr << "\n[ImageWriter] ERROR: Could not create and open the file location for writing.\n" << output_path_;
                return;
            } else {
                std::cout << "\n[ImageWriter] Writing Image Stream To: " << output_path_ << std::endl;
            }
        }

        // write to video
        video_writer_->write(image);
    } 
    catch (cv_bridge::Exception& e) 
    {
        std::cerr << "\n[ImageWriter] cv_bridge exception: " << e.what() << std::endl;
        return;
    }
}

/** ROS empty message callback function.
 * 
 * This is used to release the video writer resource and reset the class to begin a new video.
 *
 * @param msg input std_msgs/Empty message
 */
void ImageWriter::emptyCallback(const std_msgs::Empty::ConstPtr& msg) 
{
     // release, delete, and reset the video writer class object
     video_writer_->release();
     delete(video_writer_);
     video_writer_ = nullptr;
     is_timestamp_set_ = false;
     std::cout << "[ImageWriter] Image writer reset. Ready to record new video stream." << std::endl;
}

/** Update output path. 
 * 
 *  Update the output path with a timestamp for the processing of multiple image streams in sequence.
 *
 */
void ImageWriter::updateOutputPath()
{ 
    // get last "/" before output file name
    size_t slashPos = orig_output_path_.find_last_of("/");
    if (slashPos == std::string::npos) {
        std::cerr << "Error: '/' not found in string for file location path." << std::endl;
        return;
    }

    // update output path variable with a time stamp pre-fixed to video file.
    std::string path_to_file = orig_output_path_.substr(0, slashPos);
    std::string filename = orig_output_path_.substr(slashPos + 1);

    // new file path
    output_path_ = path_to_file + "/" + time_stamp_prefix_ + filename;
}

/** Get current time stamp. 
 * 
 * This is used to get the current time stamp for naming files differently.
 *
 * @param time_stamp_out output variable filled with the time stamp
 */
void ImageWriter::getTimeStamp(std::string& time_stamp_out)
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S_");
    time_stamp_out = ss.str();
}