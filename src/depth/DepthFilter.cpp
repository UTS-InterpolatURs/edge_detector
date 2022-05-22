#include "DepthFilter.h"

namespace Image {
    DepthFilter::DepthFilter() {}
    
    // Takes in a 16UC1 image
    cv::Mat DepthFilter::removePixelsBelowDepth(cv::Mat image, cv::Mat colorImage, float depthFromLowest) {
        ushort minDepth = depthAtPixel(image, cv::Point2f(0,0));

        ROS_INFO_STREAM("Min Depth: " << minDepth);

        ROS_INFO_STREAM("Depth Rows/Colors: " << image.rows << " " << image.cols);
        ROS_INFO_STREAM("Color Rows/Colors: " << colorImage.rows << " " << colorImage.cols);

        cv::Mat newImage = cv::Mat(image.rows, image.cols, CV_8U);

        for(int y = 0; y < image.rows; y++) {
            for(int x = 0; x < image.cols; x++) {
                ushort pixelDepth = image.at<ushort>(y, x);

                if(pixelDepth > (minDepth - depthFromLowest)) { // Remove color
                    newImage.at<uchar>(y,x) = 0;
                }else {
                    newImage.at<uchar>(y,x) = 255;
                }
            }
        }

        return newImage;
    }

    ushort DepthFilter::depthAtPixel(cv::Mat image, cv::Point2f point) {
        return image.at<ushort>(point.x, point.y);
    }

}