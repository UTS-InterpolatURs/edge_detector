#ifndef IMAGE_PROCESSOR_H
#define IMAGE_PROCESSOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace Image {
    class ImageProcessor {
        public:
            ImageProcessor();

            enum Option {
                GREYSCALE,
                GAUSSIAN,
                CANNY_EDGE,
                CONTOURS
            };

            cv::Mat apply(cv::Mat image, std::vector<Option> options);
        private:
            cv::Mat applyGreyscale(cv::Mat image);
            cv::Mat applyGaussian(cv::Mat image);
            cv::Mat applyCanny(cv::Mat image);
            cv::Mat applyContours(cv::Mat image);
    };
};

#endif