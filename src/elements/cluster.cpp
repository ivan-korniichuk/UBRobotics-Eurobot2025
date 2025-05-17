#include "elements/cluster.hpp"
#include <iostream>

Cluster::Cluster(vector<Point2f> border, Scalar color, ClusterPos position, float obstacleRadius, string id, double priority)
    : Element(border, color, obstacleRadius, id) {
    
    this->id = id;
    if (border.size() != 4) {
        cerr << "Error: Cluster must have exactly 4 points" << endl;
        return;
    }

    center = (border[0] + border[2]) / 2;

    if (position == ClusterPos::VERTICAL) {
        Point2f diff = Point2f(obstacleRadius + accessDistance + 50, 0);
        accessPoints.push_back(center + diff);
        accessPoints.push_back(center - diff);
    } else {
        Point2f diff = Point2f(0, obstacleRadius + accessDistance + 50);
        accessPoints.push_back(center + diff);
        accessPoints.push_back(center - diff);
    }

    this->priority = priority;
}

Point2f Cluster::getAccessPoint(int index) {
    return accessPoints[index];
}

void Cluster::setStatus(ClusterStatus status) {
    this->status = status;

    if (status == ClusterStatus::TAKEN) {
        color = color * 0.1;
    } else if (status == ClusterStatus::UNAVAILABLE) {
        color = Scalar(0, 0, 255);
    }
}

void Cluster::drawElement(Mat& image) {
    Element::drawElement(image);
    circle(image, accessPoints[0], 5, color, FILLED, LINE_AA);
    circle(image, accessPoints[1], 5, color, FILLED, LINE_AA);
    putText(image, id, center - Point2f(20, -20), FONT_HERSHEY_SIMPLEX, 2, color, 5);
}