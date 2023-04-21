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
            fps_(fps),
            video_writer_(nullptr) {}

/** Deconstructor.
 * 
 * Image Saver class deconstructor
 *
 */
ImageWriter::~ImageWriter()
{
    delete(video_writer_);
}

/** ROS image callback function.
 * 
 * This saves a ROS raw image stream to a video file.
 *
 * @param msg input sensor_msgs/Image message
 * @return void
 */
void ImageWriter::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    try 
    {
        // convert ROS Image message to OpenCV image
        cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image;

        // check if RGB
        bool isColor = (image.type() == CV_8UC3); 

        // create video writer if it doesn't exist
        if (video_writer_ == nullptr) 
        {
            // Create instance setup and open video writer 
            video_writer_ = new cv::VideoWriter();
            int codec = cv::VideoWriter::fourcc('G','R','E','Y'); // set  codec
            video_writer_->open(output_path_, codec, fps_, image.size(), isColor);

            // check for success
            if (!video_writer_->isOpened()) {
                std::cerr << "Could not open the output video file for write\n";
                return;
            } else {
                std::cout << "[ImageWriter] File created and opened for writing.\n writing ..." << std::endl;
            }
        }

        // write to video
        video_writer_->write(image);
    } 
    catch (cv_bridge::Exception& e) 
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void ImageWriter::emptyCallback(const std_msgs::Empty::ConstPtr& msg) {
     std::cout << "\n stop called" << std::endl;
     video_writer_->release();
     delete(video_writer_);
     video_writer_ = nullptr;
}
