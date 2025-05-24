#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

using namespace cv;
using namespace std;

class Element {
protected:
    const float navigationOffset = 45;
    vector<Point2f> border;
    vector<Point2f> obstacle;
    vector<Point2f> navigationObstacle;
    Scalar color;

    vector<Point2f> scalePolygon(const vector<Point2f>& polygon, float offset);

public:
    string id;

    Element(vector<Point2f> border, Scalar color, float obstacleRadius = 0, string id = "-");
    Element(); // empty constructor

    void updateObstacle(vector<Point2f> border, float obstacleRadius = 200);
    void setAttributes(vector<Point2f> border, Scalar color, float obstacleRadius = 0, string id = "-");
    vector<Point2f> getObstacle();
    vector<Point2f> getNavigator();
    virtual void drawElement(Mat& image);
};
