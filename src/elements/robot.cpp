#include "elements/robot.hpp"
#include <iostream>
#include "config.hpp"

using namespace robotVars;

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

void Robot::setYaw(float newYaw) {
    if (newYaw == -999) return;

    // Normalize yaw to [0, 360)
    newYaw = fmod(newYaw + 360.0, 360.0);

    lock_guard<mutex> lock(yawMutex);

    float delta = fabs(newYaw - lastYaw);
    // Correct wraparound (e.g., from 359 to 1)
    if (delta > 180.0) delta = 360.0 - delta;

    // If the jump is too large, consider it a glitch
    if (delta > yawThreshold) {
        // Ignore this update
        // cerr << "[Yaw] Spike detected, ignoring: " << newYaw << " vs " << lastYaw << endl;
        lastYaw = newYaw;
        return;
    }
    lastYaw = newYaw;
    this->yaw = fmod(newYaw - cameraAngle + 360.0, 360.0);
}

float Robot::getYaw() const {
    std::lock_guard<std::mutex> lock(yawMutex);
    return yaw;
}

void Robot::setPosition(Point2f position) {
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

Point2f Robot::getPosition() const {
    lock_guard<mutex> lock(posMutex);
    return position;
}

void Robot::update() {
    Point2f newPosition = locator->find(markerId);
    setPosition(newPosition);
}

int Robot::getMarkerId() const {
    return markerId;
}

void Robot::setMarkerId(int markerId) {
    this->markerId = markerId;
}