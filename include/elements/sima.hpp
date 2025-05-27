#pragma once

#include "elements/element.hpp"
#include "locator.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>
#include <string>

using namespace cv;
using namespace std;

class Sima : public Element {
private:
    int markerId;
    Point2f position;
    vector<Point2f> dimensions;

    vector<Point2f> waypoints;
    size_t currentTargetIndex = 0;
    float proximityThreshold = 50.0f;

    mutable mutex posMutex;

public:
    Locator* locator;

    Sima(int markerId, Point2f startPosition, Scalar color, float size, float obstacleRadius = 0, string id = "Sima");

    void setPosition(Point2f pos);
    Point2f getPosition() const;

    void setMarkerId(int id);
    int getMarkerId() const;

    void drawElement(Mat& image) override;

    void setWaypoints(const vector<Point2f>& points);
    Point2f getCurrentTarget() const;
    bool isFinished() const;
    void step();
};
