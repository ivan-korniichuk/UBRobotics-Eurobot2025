#pragma once

#include "elements/element.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

using namespace std;
using namespace cv;

class ConstructionArea : public Element {
public:
    Point2f entry;
    int rotation;
    Point2f goal;
    bool built = false;
    Point2f center;
    double priority;

    ConstructionArea(vector<Point2f> border, Scalar color, Point2f entry, int rotation, Point2f goal, float obstacleRadius = 0, string id = "-*", double priority = 0);

    void drawElement(Mat& image) override;
    Point2f getAccessPoint();
};
