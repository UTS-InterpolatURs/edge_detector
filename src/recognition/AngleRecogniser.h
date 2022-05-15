#ifndef ANGLE_RECOGNISER_H
#define ANGLE_RECOGNISER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace Recognition {
    class AngleRecogniser {
        public:
            AngleRecogniser();

            float processImage(cv::Mat image);
        private:
            std::pair<std::vector<cv::Point>, cv::Vec4i> calculateContours(cv::Mat image);
    };
};

#endif