#pragma once

#include "elements/element.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

using namespace std;
using namespace cv;

class Cluster : public Element {
public:
    enum class ClusterPos {
        VERTICAL,
        HORIZONTAL
    };

    enum class ClusterStatus {
        AVAILABLE,
        TAKEN,
        UNAVAILABLE
    };

    vector<Point2f> accessPoints;
    float accessDistance = 20;
    Point2f center;
    ClusterStatus status = ClusterStatus::AVAILABLE;
    double priority;

    Cluster(vector<Point2f> border, Scalar color, ClusterPos position, float obstacleRadius = 0, string id = "*", double priority = 0);

    Point2f getAccessPoint(int index = 0);
    void setStatus(ClusterStatus status);
    void drawElement(Mat& image) override;
};
