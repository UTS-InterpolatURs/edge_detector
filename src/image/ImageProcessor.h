#ifndef IMAGE_PROCESSOR_H
#define IMAGE_PROCESSOR_H

#include "ros/ros.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

namespace Image {
    class ImageProcessor {
        public:
            ImageProcessor();

            enum Option {
                GREYSCALE,
                GAUSSIAN,
                CANNY_EDGE
            };

            cv::Mat apply(cv::Mat image, std::vector<Option> options);
        private:
            cv::Mat applyGreyscale(cv::Mat image);
            cv::Mat applyGaussian(cv::Mat image);
            cv::Mat applyCanny(cv::Mat image);
    };
};

#endif