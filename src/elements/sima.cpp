#include "elements/sima.hpp"

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

    // Draw the path (waypoints)
    if (!waypoints.empty()) {
        // Draw lines between waypoints
        for (size_t i = 1; i < waypoints.size(); ++i) {
            line(image, waypoints[i-1], waypoints[i], color, 2, LINE_AA);
        }
        // Draw line from current position to next target
        if (currentTargetIndex < waypoints.size()) {
            line(image, position, waypoints[currentTargetIndex], Scalar(0, 0, 255), 2, LINE_AA);
        }
        // Draw all waypoints as circles
        for (size_t i = 0; i < waypoints.size(); ++i) {
            Scalar c = (i == currentTargetIndex) ? Scalar(0, 255, 0) : color; // green for current target
            circle(image, waypoints[i], 5, c, FILLED, LINE_AA);
        }
    }

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

void Sima::step(int simaNO) {
    if (!robotClient) return;

    if (isFinished()) {
        robotClient->sendSimasCommand(simaNO, 0x09, {0, 1, 0, 1});
        robotClient->sendSimasCommand(simaNO, 0x0A, {1});
        return;
    }

    Point2f currentPos = getPosition();
    float simaYaw = getYaw(); // degrees

    Point2f target = getCurrentTarget();
    float dx = target.x - currentPos.x;
    float dy = target.y - currentPos.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    float targetAngle = atan2(-dy, -dx) * 180.0 / CV_PI;

    float relativeYaw = targetAngle - simaYaw;
    if (relativeYaw < -90) relativeYaw += 360;

    if (relativeYaw > 180) relativeYaw -= 360;
    if (relativeYaw < -180) relativeYaw += 360;

    float speed = min(255.0f, dist/1.6f);

    uint8_t speed_byte_x;
    uint8_t speed_byte_y;

    if (relativeYaw < -3) {
        speed_byte_x = static_cast<uint8_t>(max(30.0f, speed+relativeYaw*4));
        speed_byte_y = static_cast<uint8_t>(max(40.0f, speed+relativeYaw/1.5f));
    } else if (relativeYaw > 3) {
        speed_byte_x = static_cast<uint8_t>(max(40.0f, speed-relativeYaw/1.5f));
        speed_byte_y = static_cast<uint8_t>(max(30.0f, speed-relativeYaw*4));
    } else {
        speed_byte_x = static_cast<uint8_t>(max(40.0f, speed));
        speed_byte_y = static_cast<uint8_t>(max(40.0f, speed));
    }

    if (dist < proximityThreshold) {
        currentTargetIndex++;
        robotClient->sendSimasCommand(simaNO, 0x09, {0, 1, 0, 1});
        return;
    } else {
        robotClient->sendSimasCommand(simaNO, 0x09, {speed_byte_x, 1, speed_byte_y, 1});
    }
}

void Sima::setYaw(float yaw_) { yaw = yaw_; }
float Sima::getYaw() const { return yaw; }