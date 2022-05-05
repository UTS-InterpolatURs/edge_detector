#include "ImageConverter.h"

namespace Image {

    ImageConverter::ImageConverter() {}

    cv::Mat ImageConverter::convertMessageToCVImage(const sensor_msgs::ImageConstPtr &msg, std::string encoding = sensor_msgs::image_encodings::BGR8) {
        cv_bridge::CvImagePtr cvPtr;

        try {
            cvPtr = cv_bridge::toCvCopy(msg, encoding);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("CV_BRIDGE EXCEPTION: thing");
        }

        return cvPtr->image;
    }

}