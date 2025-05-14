#include "elements/enemy.hpp"
#include <iostream>

Enemy::Enemy(int markerId, Point2f position, Scalar color, float size, float obstacleRadius, string id) {
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

void Enemy::drawElement(Mat& image) {
    Element::drawElement(image);
    circle(image, position, 5, color, FILLED, LINE_AA);
    putText(image, id, position - Point2f(90, 20), FONT_HERSHEY_SIMPLEX, 2, color, 5);
}

void Enemy::setPosition(Point2f position) {
    if (position == Point2f(-1, -1)) return;
    lock_guard<mutex> lock(posMutex);
    vector<Point2f> border = dimensions;
    for (Point2f &point : border) {
        point.x += position.x;
        point.y += position.y;
    }

    this->position = position;
    updateObstacle(border);
}

void Enemy::update() {
    Point2f newPosition = locator->find(markerId);
    setPosition(newPosition);
}

int Enemy::getMarkerId() const {
    return markerId;
}

void Enemy::setMarkerId(int markerId) {
    this->markerId = markerId;
}