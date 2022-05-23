#include "ros/ros.h"
#include "image/ImageConverter.h"
#include "image/ImageProcessor.h"
#include "recognition/Recogniser.h"
#include "recognition/AngleRecogniser.h"

#include "edge_detector/NamedFeature.h"
#include "edge_detector/RecognisedFeatureArray.h"

#include "depth/DepthFilter.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point32.h>
#include <vector>

#include "opencv2/objdetect.hpp"

using namespace Image;

ImageConverter converter;
ImageProcessor processor;

image_transport::Publisher imagePub;
ros::Publisher centerPub;

std::vector<Recognition::Feature*> features;

int sequenceCounter = 0;

DepthFilter depthFilter;

void imageCallback(const sensor_msgs::ImageConstPtr &alignedMessage, const sensor_msgs::ImageConstPtr &colorMessage) {
    cv::Mat inputColorImage = converter.convertMessageToCVImage(colorMessage, colorMessage->encoding);
    cv::Mat inputDepthImage = converter.convertMessageToCVImage(alignedMessage, alignedMessage->encoding);
    cv::Mat filteredImage = depthFilter.removePixelsBelowDepth(inputDepthImage, inputColorImage, 70);

    cv::Mat processedImage = processor.apply(filteredImage, std::vector<ImageProcessor::Option> {
        // ImageProcessor::Option::GREYSCALE,
        ImageProcessor::Option::GAUSSIAN,
        ImageProcessor::Option::BILATERAL_FILTER,
        ImageProcessor::Option::CANNY_EDGE
    });

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierachy;

    cv::findContours(processedImage, contours, hierachy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);

    // std::vector<cv::Point> boardContour = contours.at(1);

    // std::vector<cv::RotatedRect> rects;

    cv::RotatedRect rect;

    for(auto contour : contours) {
        cv::RotatedRect localRect = cv::minAreaRect(contour);
        
        cv::Size2f size = localRect.size;

        if(rect.boundingRect().area() < localRect.boundingRect().area()) {
            rect = localRect;
        }
    }

    // ROS_INFO_STREAM("Width/Height: " << largestRect.size.width << " " << largestRect.size.height);

    cv::Point2f vertices2f[4];
    rect.points(vertices2f);

    std::vector<cv::Point> vertices(4);
    for(int i = 0; i < 4; i++) {
        vertices[i] = vertices2f[i];
    }

    cv::Point textPoint = cv::Point(vertices[0].x, vertices[0].y+10);

    std::stringstream stream;

    stream << "W:" << rect.size.width;
    stream << "/H:" << rect.size.height;
    std::string text;
    stream >> text;

    cv::putText(processedImage, text, textPoint, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.9, cv::Scalar(255,255,255));

    cv::polylines(processedImage, vertices, true, cv::Scalar(255,0,0), 3);

    cv::Point point = rect.center;

    cv::circle(processedImage, point, 20, cv::Scalar(255,255,255), 3);

    geometry_msgs::Point32 centerMessage;
    centerMessage.x = point.x;
    centerMessage.y = point.y;

    centerPub.publish(centerMessage);

    ROS_INFO_STREAM("1: " << vertices[0] << " 2: " << vertices[1] << " 3: " << vertices[2] << " 4: " << vertices[3]);

    sensor_msgs::Image output = converter.convertCVImageToMessage(processedImage, sensor_msgs::image_encodings::MONO8);
    imagePub.publish(output);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "edge_detector");
    ros::NodeHandle n;

    image_transport::ImageTransport transport(n);
    // image_transport::Subscriber imageSub = transport.subscribe("/camera/aligned_depth_to_color/image_raw", 1, imageCallback);
    imagePub = transport.advertise("/image_convert/output_video", 1);

    message_filters::Subscriber<sensor_msgs::Image> depthSub(n, "/camera/aligned_depth_to_color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> imageSub(n, "/camera/color/image_raw", 1);

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> ImageSyncPolicy;

    message_filters::Synchronizer<ImageSyncPolicy> sync(ImageSyncPolicy(10), depthSub, imageSub);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2));

    centerPub = n.advertise<geometry_msgs::Point32>("center", 10);

    Recognition::RectangularFeature *board = new Recognition::RectangularFeature("Board", 853, 1425);
    Recognition::RectangularFeature *computer = new Recognition::RectangularFeature("Computer", 306, 141);
    Recognition::RectangularFeature *keyslot = new Recognition::RectangularFeature("Keyslot", 164, 166);
    Recognition::RectangularFeature *batteryBox = new Recognition::RectangularFeature("Battery Box", 204, 409);
    Recognition::RectangularFeature *ethernetA = new Recognition::RectangularFeature("Ethernet A", 164, 166);
    Recognition::RectangularFeature *ethernetB = new Recognition::RectangularFeature("Ethernet B", 140, 140);
    Recognition::RectangularFeature *coinHolder = new Recognition::RectangularFeature("Coin Holder", 90, 271);

    features.push_back(board);
    features.push_back(computer);
    features.push_back(keyslot);
    features.push_back(batteryBox);
    features.push_back(ethernetA);
    features.push_back(ethernetB);
    features.push_back(coinHolder);

    ros::spin();
}