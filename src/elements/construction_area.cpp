#include "elements/construction_area.hpp"

ConstructionArea::ConstructionArea(vector<Point2f> border, Scalar color, Point2f entry, int rotation, Point2f goal, float obstacleRadius, string id, double priority)
    : Element(border, color, obstacleRadius, id) {
    this->entry = entry;
    this->rotation = rotation;
    this->goal = goal;
    this->center = (border[0] + border[2]) / 2;
    this->priority = priority;
}

void ConstructionArea::drawElement(Mat& image) {
    if (built) {
        color = Scalar(0, 0, 255);
    } else {
        color = color * 0.15;
    }
    Element::drawElement(image);
    circle(image, entry, 5, color, FILLED, LINE_AA);
    circle(image, goal, 5, color, FILLED, LINE_AA);
    putText(image, id, center - Point2f(20, -20), FONT_HERSHEY_SIMPLEX, 2, color, 5);
}

Point2f ConstructionArea::getAccessPoint() {
    return entry;
}