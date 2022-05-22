#ifndef DEPTH_FILTER_H
#define DEPTH_FILTER_H

#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace Image {
    class DepthFilter {
        public:
            DepthFilter();

            cv::Mat removePixelsBelowDepth(cv::Mat image, cv::Mat colorImage, float depthFromLowest);
        private:
            ushort depthAtPixel(cv::Mat image, cv::Point2f point);
    };
};

#endif