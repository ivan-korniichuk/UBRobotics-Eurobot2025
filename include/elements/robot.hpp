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
    mutable mutex yawMutex;
    float yaw = -999;
    float lastYaw = -999;
    float yawThreshold = 30.0;

public:
    Locator* locator;

    Robot(int markerId, Point2f position, Point2f endPosition, Scalar color, float size, float obstacleRadius = 0, string id = "Robot");

    Point2f endPosition;

    void drawElement(Mat& image) override;
    void setPosition(Point2f position);
    Point2f getPosition() const;
    void update();
    int getMarkerId() const;
    void setMarkerId(int markerId);
    void setYaw(float yaw);
    float getYaw() const;
};
