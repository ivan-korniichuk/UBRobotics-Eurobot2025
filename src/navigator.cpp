#include "navigator.hpp"

bool doLineSegmentsIntersect(const Point2f& p1, const Point2f& p2,
                             const Point2f& p3, const Point2f& p4) {
    auto orientation = [](const Point2f& a, const Point2f& b, const Point2f& c) {
        float val = (b.y - a.y) * (c.x - a.x) - (b.x - a.x) * (c.y - a.y);
        if (fabs(val) < FLT_EPSILON) return 0.0f;
        return val;
    };

    float o1 = orientation(p1, p2, p3);
    float o2 = orientation(p1, p2, p4);
    float o3 = orientation(p3, p4, p1);
    float o4 = orientation(p3, p4, p2);

    if ((o1 * o2 < 0) && (o3 * o4 < 0)) {
        return true;
    }

    auto onSegment = [](const Point2f& p, const Point2f& q, const Point2f& r) {
        if (min(p.x, r.x) <= q.x && q.x <= max(p.x, r.x) &&
            min(p.y, r.y) <= q.y && q.y <= max(p.y, r.y))
            return true;
        return false;
    };

    if (fabs(o1) < FLT_EPSILON && onSegment(p1, p3, p2)) return true;
    if (fabs(o2) < FLT_EPSILON && onSegment(p1, p4, p2)) return true;
    if (fabs(o3) < FLT_EPSILON && onSegment(p3, p1, p4)) return true;
    if (fabs(o4) < FLT_EPSILON && onSegment(p3, p2, p4)) return true;

    // If none of the cases apply, no intersection
    return false;
}

bool isLineIntersectingPolygons(const vector<vector<Point2f>>& polygons,
                                const Point2f& start, const Point2f& end) {
    for (const auto& poly : polygons) {
        if (poly.size() < 3) continue;
        double resStart = pointPolygonTest(poly, start, false);
        double resEnd   = pointPolygonTest(poly, end,   false);
        if (resStart >= 0 || resEnd >= 0) {
            return true;
        }

        for (size_t i = 0; i < poly.size(); i++) {
            Point2f p1 = poly[i];
            Point2f p2 = poly[(i + 1) % poly.size()];

            if (doLineSegmentsIntersect(start, end, p1, p2)) {
                return true;
            }
        }
    }
    return false;
}

Navigator::Navigator(Rect boundary) {
    this->boundary = boundary;
}

void Navigator::setClusters(vector<Cluster*> clusters) {
    this->clusters = clusters;
}

void Navigator::setConstructionAreas(vector<ConstructionArea*> constructionAreas) {
    this->constructionAreas = constructionAreas;
}

void Navigator::insertElement(Element element) {
    stationaryElements.push_back(element);
}

void Navigator::drawPath(Mat& img, Point2f start, Point2f end) {
    vector<Point2f> path = navigate(start, end);
    for (size_t i = 1; i < path.size(); i++) {
        line(img, path[i-1], path[i], Scalar(255, 255, 255), 5, LINE_AA);
    }
}

void Navigator::showMesh(Mat& img) {
    vector<vector<Point2f>> navigators, obstacles;

    for (Element& el : stationaryElements) {
        obstacles.push_back(el.getObstacle());
        navigators.push_back(el.getNavigator());
    }

    vector<Point2f> allPoints;
    for (const auto& nav : navigators) {
        for (const auto& pt : nav) {
            if (boundary.contains(pt)) allPoints.push_back(pt);
        }
    }

    for (size_t i = 0; i < allPoints.size(); i++) {
        for (size_t j = i + 1; j < allPoints.size(); j++) {
            const Point2f& pt1 = allPoints[i];
            const Point2f& pt2 = allPoints[j];

            if (boundary.contains(pt1) && boundary.contains(pt2)) {
                if (!isLineIntersectingPolygons(obstacles, pt1, pt2)) {
                    line(img, pt1, pt2, color, 5, LINE_AA);
                }
            }
        }
    }
}

float Navigator::distanceFromPath(vector<Point2f> path) {
    float distance = 0.0f;
    if (path.empty()) return -1;

    for (size_t i = 1; i < path.size(); i++) {
        distance += norm(path[i] - path[i - 1]);
    }
    return distance;
}

vector<Point2f> Navigator::navigate(const Point2f& start, const Point2f& end) {
    vector<vector<Point2f>> navigators, obstacles;

    obstacles.push_back(enemy->getObstacle());

    for (Element& el : stationaryElements) {
        obstacles.push_back(el.getObstacle());
        navigators.push_back(el.getNavigator());
    }

    for (Cluster* cl : clusters) {
        if (cl->status == Cluster::ClusterStatus::TAKEN) continue;
        obstacles.push_back(cl->getObstacle());
        navigators.push_back(cl->getNavigator());
    }

    for (ConstructionArea* ca : constructionAreas) {
        if (ca->built) {
            obstacles.push_back(ca->getObstacle());
            navigators.push_back(ca->getNavigator());
        }
    }

    struct Node {
        Point2f point;
        double gCost = numeric_limits<double>::max();
        double hCost = 0;
        vector<Point2f> path;

        double fCost() const { return gCost + hCost; }
    };

    vector<Node> openList, closedList, allNodes;

    for (const auto& nav : navigators) {
        for (const auto& pt : nav) {
            if (boundary.contains(pt)) {
                Node node;
                node.point = pt;
                node.hCost = norm(end - pt);
                allNodes.push_back(node);
            }
        }
    }

    Node startNode;
    startNode.point = start;
    startNode.gCost = 0;
    startNode.hCost = norm(end - start);
    startNode.path.push_back(start);
    openList.push_back(startNode);

    Node endNode;
    endNode.point = end;
    allNodes.push_back(endNode);

    while (!openList.empty()) {
        auto currentIt = min_element(openList.begin(), openList.end(), [](const Node& a, const Node& b) {
            return a.fCost() < b.fCost();
        });

        Node currentNode = *currentIt;
        openList.erase(currentIt);
        closedList.push_back(currentNode);

        if (currentNode.point == end) return currentNode.path;

        for (auto& neighbor : allNodes) {
            if (any_of(closedList.begin(), closedList.end(), [&](const Node& n) { return n.point == neighbor.point; }))
                continue;

            if (isLineIntersectingPolygons(obstacles, currentNode.point, neighbor.point))
                continue;

            double tentativeG = currentNode.gCost + norm(currentNode.point - neighbor.point);

            auto it = find_if(openList.begin(), openList.end(), [&](const Node& n) { return n.point == neighbor.point; });

            if (it == openList.end() || tentativeG < neighbor.gCost) {
                neighbor.gCost = tentativeG;
                neighbor.path = currentNode.path;
                neighbor.path.push_back(neighbor.point);

                if (it == openList.end()) {
                    openList.push_back(neighbor);
                }
            }
        }
    }

    return {};
}