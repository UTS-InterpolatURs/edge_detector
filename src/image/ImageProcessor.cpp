#include "ImageProcessor.h"

namespace Image {

    ImageProcessor::ImageProcessor() {}

    cv::Mat ImageProcessor::apply(cv::Mat image, std::vector<Option> options) {
        for(auto option : options) {
            switch(option) {
                case GREYSCALE:
                    image = applyGreyscale(image);
                case GAUSSIAN:
                    image = applyGaussian(image);
                case CANNY_EDGE:
                    image = applyCanny(image);
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

}