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
            }else if(option == BILATERAL_FILTER) {
                image = applyBilateralFilter(image);
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

        cv::GaussianBlur(image, imageBlurred, cv::Size(5,5), 0, 0);
        return imageBlurred;
    }

    cv::Mat ImageProcessor::applyCanny(cv::Mat image) {
        cv::Mat edgeImage;
        cv::Canny(image, edgeImage, 100, 200, 3, false);
        return edgeImage;
    }

    cv::Mat ImageProcessor::applyBilateralFilter(cv::Mat image) {
        cv::Mat out;
        cv::bilateralFilter(image, out, 5, 5, 150);
        return out;
    }

    cv::Mat ImageProcessor::applyContours(cv::Mat image) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierachy;

        cv::findContours(image, contours, hierachy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
        cv::Mat drawing = cv::Mat::zeros(image.size(), CV_8UC1);

        for (size_t i = 0; i < contours.size(); i++) {
            cv::Scalar color = cv::Scalar(255, 255, 255);
            cv::drawContours(drawing, contours, (int)i, color, 2, cv::LINE_8, hierachy, 0);
        }

        return drawing;
    }

}