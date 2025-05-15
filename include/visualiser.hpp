#pragma once

#include "elements/element.hpp"
#include "elements/cluster.hpp"
#include "elements/robot.hpp"
#include "elements/enemy.hpp"
#include "elements/construction_area.hpp"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Visualiser {
public:
    Mat defaultMat;

    Enemy* enemy;
    Robot* robot;

    Element* ramp;
    Element* baseY;
    Element* baseB;

    Element* M20;
    Element* M21;
    Element* M22;
    Element* M23;

    Cluster* cluster1;
    Cluster* cluster2;
    Cluster* cluster3;
    Cluster* cluster4;
    Cluster* cluster5;
    Cluster* cluster6;
    Cluster* cluster7;
    Cluster* cluster8;
    Cluster* cluster9;
    Cluster* cluster10;

    ConstructionArea* constructionArea1;
    ConstructionArea* constructionArea2;
    ConstructionArea* constructionArea3;
    ConstructionArea* constructionArea4;
    ConstructionArea* constructionArea5;
    ConstructionArea* constructionArea6;

    Element* construction12IM;
    Element* construction34IM;
    Element* construction5IM;
    Element* construction6IM;

    Element* constructionB1;
    Element* constructionB2;
    Element* constructionB3Big;
    Element* constructionB4Big;

    vector<Point2f> path = {};

    Mat latestFrame;
    mutex frameMutex;

    float elapsedTime = 0.0f;

    Visualiser(int height, int width, Scalar color = Scalar(60, 60, 30));
    void updateFrame();
    void drawImage();
    void setElapsedTime(float time);
};
