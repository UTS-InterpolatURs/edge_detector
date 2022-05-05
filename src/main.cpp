#include "ros/ros.h"
#include "image/ImageConverter.h"

using namespace Image;

int main(int argc, char **argv) {
    ros::init(argc, argv, "edge_detector");
    ros::NodeHandle n;

    ros::spin();
}