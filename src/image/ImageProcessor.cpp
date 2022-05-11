#include "ImageProcessor.h"

namespace Image {

    ImageProcessor::ImageProcessor() {}

    cv::Mat ImageProcessor::apply(cv::Mat image, std::vector<Option> options) {
        for(auto option : options) {

            if(option == GREYSCALE) {
                image = applyGreyscale(image);
            }else if(option == GAUSSIAN) {
                image = applyGaussian(image);
            }else if(option == CANNY_EDGE) {
                image = applyCanny(image);
            }else if(option == CONTOURS) {
                image = applyContours(image);
            }
        }

        return image;
    }

    cv::Mat ImageProcessor::applyGreyscale(cv::Mat image) {
        cv::Mat imageGrey;
        cv::cvtColor(image, imageGrey, CV_BGR2GRAY);
        return imageGrey;
    }

    cv::Mat ImageProcessor::applyGaussian(cv::Mat image) {
        cv::Mat imageBlurred;
        cv::GaussianBlur(image, imageBlurred, cv::Size(3,3), 0, 0);
        return imageBlurred;
    }

    cv::Mat ImageProcessor::applyCanny(cv::Mat image) {
        cv::Mat edgeImage;
        cv::Canny(image, edgeImage, 100, 200, 3, false);
        return edgeImage;
    }

    cv::Mat ImageProcessor::applyContours(cv::Mat image) {
        cv::Mat contourImage = image.clone();
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierachy;
        cv::findContours(image, contours, hierachy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
        cv::drawContours(contourImage, contours, -1, cv::Scalar(0, 255, 0), 2);
        return contourImage;
    }

}