#include "elements/robot.hpp"
#include <iostream>

Robot::Robot(int markerId, Point2f position, Scalar color, float size, float obstacleRadius, string id) {
    vector<Point2f> square = {{0,0},{0,1},{1,1},{1,0}};
    dimensions = scalePolygon(square, size);

    vector<Point2f> border = dimensions;
    for (Point2f &point : border) {
        point.x += position.x;
        point.y += position.y;
    }

    this->position = position;
    this->markerId = markerId;

    setAttributes(border, color, obstacleRadius, id);
}

void Robot::drawElement(Mat& image) {
    Element::drawElement(image);
    circle(image, position, 5, color, FILLED, LINE_AA);
    putText(image, id, position - Point2f(90, 20), FONT_HERSHEY_SIMPLEX, 2, color, 5);
}

void Robot::setPosition(Point2f position) {
    vector<Point2f> border = dimensions;
    for (Point2f &point : border) {
        point.x += position.x;
        point.y += position.y;
    }

    this->position = position;
    updateObstacle(border);
}

Point2f Robot::getPosition() const {
    return position;
}

void Robot::update() {
    Point2f newPosition = locator->find(markerId);
    setPosition(newPosition);
}
