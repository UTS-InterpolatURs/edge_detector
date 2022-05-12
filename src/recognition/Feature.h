#ifndef FEATURE_H
#define FEATURE_H

#include <string>

namespace Recognition {
    struct Feature {
        public:
            Feature() {}
            Feature(std::string name) { this->name = name; }
            std::string name;
    };

    struct RectangularFeature: public Feature {
        RectangularFeature(std::string name, float width, float height) : Feature(name) {
            this->width = width;
            this->height = height;
        }
        float width;
        float height;
    };

    struct CircularFeature: public Feature {
        CircularFeature(std::string name, float diameter) : Feature(name) {
            this->diameter = diameter;
        }
        float diameter;
    };
};

#endif