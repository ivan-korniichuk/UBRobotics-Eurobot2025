#pragma once

#include "elements/element.hpp"
#include "locator.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <mutex>

using namespace cv;
using namespace std;

class Enemy : public Element {
private:
    vector<Point2f> dimensions;
    Point2f position;
    int markerId;
    mutable mutex posMutex;

public:
    Locator* locator;

    Enemy(int markerId, Point2f position, Scalar color, float size, float obstacleRadius = 0, string id = "Enemy");

    void drawElement(Mat& image) override;
    void setPosition(Point2f position);
    void update();
    int getMarkerId() const;
    void setMarkerId(int markerId);
};
