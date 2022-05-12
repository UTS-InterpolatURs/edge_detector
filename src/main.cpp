#include "ros/ros.h"
#include "image/ImageConverter.h"
#include "image/ImageProcessor.h"

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <vector>

using namespace Image;

ImageConverter converter;
ImageProcessor processor;

image_transport::Publisher imagePub;

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    // cv::Mat image = converter.convertMessageToCVImage(msg);
    cv::Mat image = cv::imread("/home/jon/Pictures/IMG_3911.jpeg");
    image = processor.apply(image, std::vector<ImageProcessor::Option>{
        ImageProcessor::Option::GREYSCALE,
        ImageProcessor::Option::GAUSSIAN,
        ImageProcessor::Option::BILATERAL_FILTER,
        // ImageProcessor::Option::CANNY_EDGE, 
        // ImageProcessor::Option::CONTOURS
    });
    sensor_msgs::Image message = converter.convertCVImageToMessage(image, sensor_msgs::image_encodings::MONO8);
    imagePub.publish(message);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "edge_detector");
    ros::NodeHandle n;

    image_transport::ImageTransport transport(n);
    image_transport::Subscriber imageSub = transport.subscribe("/camera/color/image_raw", 1, imageCallback);
    imagePub = transport.advertise("/image_convert/output_video", 1);

    ros::spin();
}