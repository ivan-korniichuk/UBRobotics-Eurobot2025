#pragma once

#include "elements/element.hpp"
#include "elements/cluster.hpp"
#include "elements/construction_area.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <limits>
#include "elements/enemy.hpp"

using namespace std;
using namespace cv;

class Navigator {
private:
    vector<Element> stationaryElements;
    vector<Cluster*> clusters;
    vector<ConstructionArea*> constructionAreas;
    Rect boundary;
    const Scalar color = Scalar(255, 50, 255);

public:
    Navigator(Rect boundary);
    Enemy *enemy;

    void setClusters(vector<Cluster*> clusters);
    void setConstructionAreas(vector<ConstructionArea*> constructionAreas);
    void insertElement(Element element);

    void drawPath(Mat& img, Point2f start, Point2f end);
    void showMesh(Mat& img);
    float distanceFromPath(vector<Point2f> path);
    vector<Point2f> navigate(const Point2f& start, const Point2f& end);
};
