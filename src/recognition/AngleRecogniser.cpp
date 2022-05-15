#include "AngleRecogniser.h"

namespace Recognition {
    AngleRecogniser::AngleRecogniser() { }

    float AngleRecogniser::processImage(cv::Mat image) {
        std::pair<std::vector<cv::Point>, cv::Vec4i> pair = calculateContours(image);
        std::vector<cv::Point> boardPoints = pair.first;

        cv::RotatedRect rect = minAreaRect(boardPoints);

        return rect.angle();
    }

    std::pair<std::vector<cv::Point>, cv::Vec4i> AngleRecogniser::calculateContours(cv::Mat image) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierachy;

        cv::findContours(image, contours, hierachy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

        if(contours.size() < 1 || hierachy.size() <= 1) return std::pair<std::vector<cv::Point>, cv::Vec4i>(std::vector<cv::Point>(), cv::Vec4i());

        return std::pair<std::vector<cv::Point>, cv::Vec4i>(contours.at(0), hierachy.at(0));
    }
};