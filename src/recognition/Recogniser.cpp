#include "Recogniser.h"

namespace Recognition {
    Recogniser::Recogniser(std::vector<Feature*> features) {
        this->featureList = features;
    }

    std::vector<RecognisedFeature> Recogniser::processImage(cv::Mat image) {
        std::vector<RecognisedFeature> output;
        
        auto contoursPair = calculateContours(image);
        std::vector<std::vector<cv::Point>> contours = contoursPair.first;
        std::vector<std::vector<cv::Point>> contourPoly(contours.size());

        for(size_t i = 0; i < contours.size(); i++) {
            if(i > 0) return output;
            cv::approxPolyDP(contours[i], contourPoly[i], 3, true);
            cv::RotatedRect rect = cv::minAreaRect(contourPoly[i]);

            // int index = findMatchingArea(rect);

            // if(index == -1) continue;

            RecognisedFeature feature;
            feature.rect = rect;
            feature.imageX = rect.boundingRect().x;
            feature.imageY = rect.boundingRect().y;
            // feature.feature = featureList.at(i);
            feature.contourPoints = contours.at(i);

            output.push_back(feature);
        }

        return output;
    }

    std::pair<std::vector<std::vector<cv::Point>>, std::vector<cv::Vec4i>> Recogniser::calculateContours(cv::Mat image) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierachy;

        cv::findContours(image, contours, hierachy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

        return std::pair<std::vector<std::vector<cv::Point>>, std::vector<cv::Vec4i>>(contours, hierachy);
    }

    int Recogniser::findMatchingArea(cv::Rect rect) {
        float area = rect.area();

        for(size_t i = 0; i < featureList.size(); i++) {
            auto feature = featureList.at(i);
            RectangularFeature *rectangular = static_cast<RectangularFeature*>(feature);
            float featureArea;

            if(rectangular != nullptr) {
                featureArea = rectangular->height * rectangular->width;
            } else { // Circular Feature
                // TODO: Implement
                return -1;
            }

            if(featureArea >= (area-500) && featureArea <= (area+500)) { // Gives a small margin for error 
                // ROS_INFO_STREAM("Area: " << area << " Areaf " << featureArea);
                return (int)i;
            }
        }

        return -1;
    }
};