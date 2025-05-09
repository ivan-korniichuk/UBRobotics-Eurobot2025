#pragma once

#include "elements/element.hpp"
#include "locator.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <mutex>

using namespace cv;
using namespace std;

class Robot : public Element {
private:
    vector<Point2f> dimensions;
    Point2f position;
    int markerId;
    mutable mutex posMutex;

public:
    Locator* locator;

    Robot(int markerId, Point2f position, Scalar color, float size, float obstacleRadius = 0, string id = "Robot");

    void drawElement(Mat& image) override;
    void setPosition(Point2f position);
    Point2f getPosition() const;
    void update();
    int getMarkerId() const;
};
