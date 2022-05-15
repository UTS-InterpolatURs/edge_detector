#include "ros/ros.h"
#include "image/ImageConverter.h"
#include "image/ImageProcessor.h"
#include "recognition/Recogniser.h"

#include "edge_detector/NamedFeature.h"
#include "edge_detector/RecognisedFeatureArray.h"

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <vector>

using namespace Image;

ImageConverter converter;
ImageProcessor processor;

image_transport::Publisher imagePub;
ros::Publisher featurePub;

std::vector<Recognition::Feature*> features;

int sequenceCounter = 0;

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv::Mat inputImage = converter.convertMessageToCVImage(msg);

    if(inputImage.empty()) return;

    ROS_INFO("thing");
    // cv::Mat image = cv::imread("/home/jon/Pictures/IMG_3911.jpeg");
    cv::Mat image = inputImage.clone();
    image = processor.apply(image, std::vector<ImageProcessor::Option>{
        ImageProcessor::Option::GREYSCALE,
        ImageProcessor::Option::GAUSSIAN,
        ImageProcessor::Option::BILATERAL_FILTER,
        ImageProcessor::Option::CANNY_EDGE
    });

    Recognition::Recogniser recog = Recognition::Recogniser(features);
    std::vector<Recognition::RecognisedFeature> out = recog.processImage(image);

    cv::Mat blankImage;
    cv::cvtColor(cv::Mat(image.size(), image.type()), blankImage, CV_GRAY2RGB);

    std::vector<edge_detector::NamedFeature> namedFeatures;

    // for(auto feature : out) {
    //     // if(feature.rect.area() < 8000) continue; 
    //     cv::rectangle(blankImage, feature.rect.tl(), feature.rect.br(), cv::Scalar(255,255,255), 2);
    //     cv::Point textPoint = cv::Point(feature.rect.br().x, feature.rect.br().y+10);

    //     std::stringstream stream;
    //     stream << "W" << std::to_string(feature.rect.width) << "/H" << std::to_string(feature.rect.height);
    //     std::string str;
    //     stream >> str;

    //     cv::putText(blankImage, str, textPoint, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.9, cv::Scalar(156,200, 50));
    // }

    // for(auto feature : out) {
    //     cv::rectangle(blankImage, feature.rect.tl(), feature.rect.br(), cv::Scalar(255,255,255), 2);
    //     cv::Point textPoint = cv::Point(feature.rect.br().x, feature.rect.br().y+10);
    //     cv::putText(blankImage, feature.feature->name, textPoint, cv::FONT_HERSHEY_COMPLEX, 1.2, cv::Scalar(255,0, 0));

    //     edge_detector::NamedFeature featureMsg;
    //     featureMsg.name = feature.feature->name;
    //     featureMsg.imageX = feature.rect.tl().x;
    //     featureMsg.imageY = feature.rect.tl().y;
    //     featureMsg.rectHeight = feature.rect.height;
    //     featureMsg.rectWidth = feature.rect.width;
    //     namedFeatures.push_back(featureMsg);
    // }

    // edge_detector::RecognisedFeatureArray array;
    // array.header.seq = sequenceCounter;
    // array.header.stamp = ros::Time::now();
    // array.header.frame_id = 0x01;
    // array.features = namedFeatures;

    // featurePub.publish(array);

    // cv::Mat transparent;
    // // cv::Mat background = cv::imread("/home/jon/Pictures/IMG_3911.jpeg");
    // cv::Mat background = inputImage.clone();
    // cv::inRange(blankImage, cv::Scalar(0,0,0), cv::Scalar(0,0,0), transparent);
    // blankImage.copyTo(background, 255-transparent);

    sensor_msgs::Image message = converter.convertCVImageToMessage(blankImage, sensor_msgs::image_encodings::RGB8);
    imagePub.publish(message);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "edge_detector");
    ros::NodeHandle n;

    image_transport::ImageTransport transport(n);
    // image_transport::Subscriber imageSub = transport.subscribe("/camera/color/image_raw", 1, imageCallback);
    image_transport::Subscriber imageSub = transport.subscribe("/image_publisher_1652420056196693570/image_raw", 10, &imageCallback);
    imagePub = transport.advertise("/image_convert/output_video", 1);

    featurePub = n.advertise<edge_detector::RecognisedFeatureArray>("features", 1000);

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