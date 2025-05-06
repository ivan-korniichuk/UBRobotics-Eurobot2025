#include "elements/element.hpp"
#include <iostream>
#include <cmath>

Element::Element(vector<Point2f> border, Scalar color, float obstacleRadius, string id) {
    this->border = border;
    this->color = color;
    this->obstacle = scalePolygon(border, obstacleRadius);
    this->navigationObstacle = scalePolygon(this->obstacle, navigationOffset);
    this->id = id;
}

Element::Element() {}

vector<Point2f> Element::scalePolygon(const vector<Point2f>& polygon, float offset) {
    int n = polygon.size();
    if (n < 3) {
        cerr << "Error: Polygon must have at least 3 points" << endl;
        return polygon;
    }

    vector<Point2f> scaledPolygon(n);

    for (int i = 0; i < n; i++) {
        Point2f curr = polygon[i];
        Point2f prev = polygon[(i - 1 + n) % n];
        Point2f next = polygon[(i + 1) % n];

        Point2f edge1 = curr - prev;
        Point2f edge2 = next - curr;

        Point2f normal1 = Point2f(-edge1.y, edge1.x) / norm(edge1);
        Point2f normal2 = Point2f(-edge2.y, edge2.x) / norm(edge2);

        Point2f offsetDir = (normal1 + normal2);
        float length = norm(offsetDir);
        if (length != 0) {
            offsetDir /= length;
        } else {
            offsetDir = Point2f(0, 0);
        }

        float angle = acos(normal1.dot(normal2));
        if (angle != 0) {
            offsetDir *= offset / sin(angle / 2.0f);
        }

        scaledPolygon[i] = curr + offsetDir;
    }

    return scaledPolygon;
}

void Element::updateObstacle(vector<Point2f> border, float obstacleRadius) {
    this->border = border;
    this->obstacle = scalePolygon(border, obstacleRadius);
    this->navigationObstacle = scalePolygon(this->obstacle, navigationOffset);
}

void Element::setAttributes(vector<Point2f> border, Scalar color, float obstacleRadius, string id) {
    this->border = border;
    this->color = color;
    this->obstacle = scalePolygon(border, obstacleRadius);
    this->navigationObstacle = scalePolygon(this->obstacle, navigationOffset);
    this->id = id;
}

vector<Point2f> Element::getObstacle() {
    return obstacle;
}

vector<Point2f> Element::getNavigator() {
    return navigationObstacle;
}

void Element::drawElement(Mat& image) {
    vector<Point> intBorder(border.begin(), border.end());
    if (border.size() == 0) {
        cerr << "Error: Not enough points to draw the element." << endl;
        return;
    } else if (border.size() == 1) {
        circle(image, intBorder[0], 5, color, FILLED, LINE_AA);
    } else if (border.size() == 2) {
        line(image, intBorder[0], intBorder[1], color, 5, LINE_AA);
    } else if (border.size() > 2) {
        polylines(image, intBorder, true, color, 5, LINE_AA);
    }

    vector<Point> intObstacle(obstacle.begin(), obstacle.end());
    if (obstacle.size() >= 3) {
        polylines(image, intObstacle, true, color, 1, LINE_AA);
    }
}
