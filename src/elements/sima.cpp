#include "elements/sima.hpp"
#include <cmath>

Sima::Sima(int markerId, Point2f startPosition, Scalar color, float size, float obstacleRadius, string id) {
    vector<Point2f> square = {{0,0},{0,1},{1,1},{1,0}};
    dimensions = scalePolygon(square, size);

    vector<Point2f> border = dimensions;
    for (Point2f &point : border) {
        point.x += startPosition.x;
        point.y += startPosition.y;
    }

    this->position = startPosition;
    this->markerId = markerId;

    setAttributes(border, color, obstacleRadius, id);
}

void Sima::drawElement(Mat& image) {
    Element::drawElement(image);
    circle(image, position, 3, color, FILLED, LINE_AA);
    putText(image, id, position - Point2f(30, 10), FONT_HERSHEY_SIMPLEX, 1.5, color, 3);
}

void Sima::setPosition(Point2f newPos) {
    if (newPos == Point2f(-1, -1)) return;

    lock_guard<mutex> lock(posMutex);
    vector<Point2f> border = dimensions;
    for (Point2f &point : border) {
        point.x += newPos.x;
        point.y += newPos.y;
    }

    this->position = newPos;
    updateObstacle(border);
}

Point2f Sima::getPosition() const {
    lock_guard<mutex> lock(posMutex);
    return position;
}

void Sima::setMarkerId(int id) {
    this->markerId = id;
}

int Sima::getMarkerId() const {
    return markerId;
}

void Sima::setWaypoints(const vector<Point2f>& points) {
    lock_guard<mutex> lock(posMutex);
    waypoints = points;
    currentTargetIndex = 0;
}

Point2f Sima::getCurrentTarget() const {
    if (currentTargetIndex < waypoints.size()) {
        return waypoints[currentTargetIndex];
    }
    return {-1, -1};
}

bool Sima::isFinished() const {
    return currentTargetIndex >= waypoints.size();
}

void Sima::step() {
    if (isFinished()) return;

    Point2f currentPos = getPosition();
    Point2f target = getCurrentTarget();

    float dx = target.x - currentPos.x;
    float dy = target.y - currentPos.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    if (dist < proximityThreshold) {
        currentTargetIndex++;
    }
}
