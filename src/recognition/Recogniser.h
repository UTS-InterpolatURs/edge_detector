#ifndef RECOGNISER_H
#define RECOGNISER_H

#include "Feature.h"
#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace Recognition {

    struct RecognisedFeature {
        RecognisedFeature() {}
        Feature *feature;
        std::vector<cv::Point> contourPoints;
        cv::Rect rect;

        int imageX;
        int imageY;
    };

    class Recogniser {
        public:
            Recogniser(std::vector<Feature*> features);

            std::vector<RecognisedFeature> processImage(cv::Mat image);

        private:
            std::vector<Feature*> featureList;

            std::pair<std::vector<std::vector<cv::Point>>, std::vector<cv::Vec4i>> calculateContours(cv::Mat image);
            int findMatchingArea(cv::Rect rect);
    };
};

#endif