#include "elements/element.hpp"
#include "elements/cluster.hpp"
#include "elements/construction_area.hpp"
#include "elements/robot.hpp"
#include "elements/enemy.hpp"
#include "navigator.hpp"
#include "locator.hpp"
#include "visualiser.hpp"
#include "strategy.hpp"

const float ROBOT_RADIUS = 200;

int main() {
    int width = 3000, height = 2000;
    // Mat img = Mat::zeros(height, width, CV_8UC3);
    // img = Scalar(60, 60, 30);

    Rect matBoundary(0, 0, width, height);

    Subdiv2D subdiv(matBoundary);

    vector<Point2f> rampVec = {
        Point2f(650, 1800), Point2f(650, 2000), Point2f(2350, 2000),
        Point2f(2350, 1800), Point2f(1950, 1800), Point2f(1950, 1550),
        Point2f(1050, 1550), Point2f(1050, 1800)
    };

    vector<Point2f> baseBVec = { Point2f(150, 2000), Point2f(600, 2000), Point2f(600, 1550), Point2f(150, 1550) };
    vector<Point2f> baseYVec = { Point2f(2400, 2000), Point2f(2850, 2000), Point2f(2850, 1550), Point2f(2400, 1550) };

    vector<Point2f> areaBSideVec = { Point2f(2550, 650), Point2f(2550, 1100), Point2f(3000, 1100), Point2f(3000, 650) };
    vector<Point2f> areaYSideVec = { Point2f(0, 650), Point2f(0, 1100), Point2f(450, 1100), Point2f(450, 650) };

    vector<Point2f> areaBFrontVec = { Point2f(1000, 0), Point2f(1000, 450), Point2f(1450, 450), Point2f(1450, 0) };
    vector<Point2f> areaYFrontVec = { Point2f(1550, 0), Point2f(1550, 450), Point2f(2000, 450), Point2f(2000, 0) };

    vector<Point2f> constBFrontVec = { Point2f(550, 0), Point2f(550, 150), Point2f(1000, 150), Point2f(1000, 0) };
    vector<Point2f> constYFrontVec = { Point2f(2000, 0), Point2f(2000, 150), Point2f(2450, 150), Point2f(2450, 0) };

    vector<Point2f> constBEdgeVec = { Point2f(2550, 0), Point2f(2550, 150), Point2f(3000, 150), Point2f(3000, 0) };
    vector<Point2f> constYEdgeVec = { Point2f(0, 0), Point2f(0, 150), Point2f(450, 150), Point2f(450, 0) };

    vector<Point2f> cluster1Vec = { Point2f(25, 1125), Point2f(25, 1525), Point2f(125, 1525), Point2f(125, 1125) };
    vector<Point2f> cluster2Vec = { Point2f(25, 200), Point2f(25, 600), Point2f(125, 600), Point2f(125, 200) };
    vector<Point2f> cluster3Vec = { Point2f(625, 1675), Point2f(625, 1775), Point2f(1025, 1775), Point2f(1025, 1675) };
    vector<Point2f> cluster4Vec = { Point2f(575, 200), Point2f(575, 300), Point2f(975, 300), Point2f(975, 200) };
    vector<Point2f> cluster5Vec = { Point2f(900, 900), Point2f(900, 1000), Point2f(1300, 1000), Point2f(1300, 900) };
    vector<Point2f> cluster6Vec = { Point2f(1700, 900), Point2f(1700, 1000), Point2f(2100, 1000), Point2f(2100, 900) };
    vector<Point2f> cluster7Vec = { Point2f(1975, 1675), Point2f(1975, 1775), Point2f(2375, 1775), Point2f(2375, 1675) };
    vector<Point2f> cluster8Vec = { Point2f(2025, 200), Point2f(2025, 300), Point2f(2425, 300), Point2f(2425, 200) };
    vector<Point2f> cluster9Vec = { Point2f(2875, 1125), Point2f(2875, 1525), Point2f(2975, 1525), Point2f(2975, 1125) };
    vector<Point2f> cluster10Vec = { Point2f(2875, 200), Point2f(2875, 600), Point2f(2975, 600), Point2f(2975, 200) };

    vector<Point2f> M20p = { Point2f(2400, 1400) };
    vector<Point2f> M21p = { Point2f(600, 1400) };
    vector<Point2f> M22p = { Point2f(2400, 600) };
    vector<Point2f> M23p = { Point2f(600, 600) };

    Navigator navigator(matBoundary);

    Element ramp(rampVec, Scalar(0, 0, 255), ROBOT_RADIUS);
    navigator.insertElement(ramp);

    Element baseY(baseYVec, Scalar(0, 255, 255), ROBOT_RADIUS);
    // navigator.insertElement(baseY);

    Element baseB(baseBVec, Scalar(255, 0, 0), ROBOT_RADIUS);
    navigator.insertElement(baseB);

    Element M20(M20p, Scalar(255, 0, 255));
    Element M21(M21p, Scalar(255, 0, 255));
    Element M22(M22p, Scalar(255, 0, 255));
    Element M23(M23p, Scalar(255, 0, 255));
    
    vector<Cluster*> clusters;

    Cluster cluster1(cluster1Vec, Scalar(120, 255, 0), Cluster::ClusterPos::VERTICAL, ROBOT_RADIUS, "1");
    clusters.push_back(&cluster1);
    Cluster cluster2(cluster2Vec, Scalar(120, 255, 0), Cluster::ClusterPos::VERTICAL, ROBOT_RADIUS, "2");
    clusters.push_back(&cluster2);;
    Cluster cluster3(cluster3Vec, Scalar(120, 255, 0), Cluster::ClusterPos::HORIZONTAL, ROBOT_RADIUS, "3");
    clusters.push_back(&cluster3);
    Cluster cluster4(cluster4Vec, Scalar(120, 255, 0), Cluster::ClusterPos::HORIZONTAL, ROBOT_RADIUS, "4");
    clusters.push_back(&cluster4);
    Cluster cluster5(cluster5Vec, Scalar(120, 255, 0), Cluster::ClusterPos::HORIZONTAL, ROBOT_RADIUS, "5");
    clusters.push_back(&cluster5);
    Cluster cluster6(cluster6Vec, Scalar(120, 255, 0), Cluster::ClusterPos::HORIZONTAL, ROBOT_RADIUS, "6");
    clusters.push_back(&cluster6);
    Cluster cluster7(cluster7Vec, Scalar(120, 255, 0), Cluster::ClusterPos::HORIZONTAL, ROBOT_RADIUS, "7");
    clusters.push_back(&cluster7);
    Cluster cluster8(cluster8Vec, Scalar(120, 255, 0), Cluster::ClusterPos::HORIZONTAL, ROBOT_RADIUS, "8");
    clusters.push_back(&cluster8);
    Cluster cluster9(cluster9Vec, Scalar(120, 255, 0), Cluster::ClusterPos::VERTICAL, ROBOT_RADIUS, "9");
    clusters.push_back(&cluster9);
    Cluster cluster10(cluster10Vec, Scalar(120, 255, 0), Cluster::ClusterPos::VERTICAL, ROBOT_RADIUS, "10");
    clusters.push_back(&cluster10);

    Element areaYSideEl(areaYSideVec, Scalar(0, 255, 255), ROBOT_RADIUS);
    // navigator.insertElement(areaYSideEl);
    Element areaBSideEl(areaBSideVec, Scalar(255, 0, 0), ROBOT_RADIUS);
    navigator.insertElement(areaBSideEl);

    Element areaYFrontEl(areaYFrontVec, Scalar(0, 255, 255), ROBOT_RADIUS);
    // navigator.insertElement(areaYFrontEl);
    Element areaBFrontEl(areaBFrontVec, Scalar(255, 0, 0), ROBOT_RADIUS);
    navigator.insertElement(areaBFrontEl);

    Element constYFrontEl(constYFrontVec, Scalar(0, 255, 255), ROBOT_RADIUS);
    // navigator.insertElement(constYFrontEl);
    Element constBFrontEl(constBFrontVec, Scalar(255, 0, 0), ROBOT_RADIUS);
    navigator.insertElement(constBFrontEl);

    Element constYEdgeEl(constYEdgeVec, Scalar(0, 255, 255), ROBOT_RADIUS);
    // navigator.insertElement(constYEdgeEl);
    Element constBEdgeEl(constBEdgeVec, Scalar(255, 0, 0), ROBOT_RADIUS);
    navigator.insertElement(constBEdgeEl);

    navigator.setClusters(clusters);

    vector<Point2f> constructionTest1 = { Point2f(1600, 50), Point2f(1600, 200), Point2f(1950, 200), Point2f(1950, 50) };
    ConstructionArea constructionArea1(constructionTest1, Scalar(0, 0, 255), Point2f(1775, 425), 0, Point2f(1775, 275), ROBOT_RADIUS, "1");

    vector<Point2f> constructionTest2 = { Point2f(1600, 300), Point2f(1600, 450), Point2f(1950, 450), Point2f(1950, 300) };
    ConstructionArea constructionArea2(constructionTest2, Scalar(0, 0, 255), Point2f(1775, 675), 0, Point2f(1775, 525), ROBOT_RADIUS, "2");

    vector<Point2f> constructionTest3 = { Point2f(50, 700), Point2f(50, 1050), Point2f(200, 1050), Point2f(200, 700) };
    ConstructionArea constructionArea3(constructionTest3, Scalar(0, 0, 255), Point2f(425, 875), 0, Point2f(275, 875), ROBOT_RADIUS, "3");

    vector<Point2f> constructionTest4 = { Point2f(300, 700), Point2f(300, 1050), Point2f(450, 1050), Point2f(450, 700) };
    ConstructionArea constructionArea4(constructionTest4, Scalar(0, 0, 255), Point2f(675, 875), 0, Point2f(525, 875), ROBOT_RADIUS, "4");

    vector<Point2f> constructionTest5 = { Point2f(2050, 50), Point2f(2050, 200), Point2f(2400, 200), Point2f(2400, 50) };
    ConstructionArea constructionArea5(constructionTest5, Scalar(0, 0, 255), Point2f(2225, 425), 0, Point2f(2225, 275), ROBOT_RADIUS, "5");

    vector<Point2f> constructionTest6 = { Point2f(50, 50), Point2f(50, 200), Point2f(400, 200), Point2f(400, 50) };
    ConstructionArea constructionArea6(constructionTest6, Scalar(0, 0, 255), Point2f(225, 425), 0, Point2f(225, 275), ROBOT_RADIUS, "6");

    navigator.setConstructionAreas({&constructionArea1, &constructionArea2, &constructionArea3, &constructionArea4, &constructionArea5, &constructionArea6});

    // locator
    Locator locator;

    Enemy enemy(2, {350,1750}, Scalar(0,0,255), 200, ROBOT_RADIUS);
    enemy.locator = &locator;

    Robot robot(7, {2650,1750}, Scalar(0,255,255), 200, ROBOT_RADIUS);
    robot.locator = &locator;
    // Enemy(Point2f position, Scalar color, float size, float obstacleRadius = 0, string id = "Enemy") {

    //visualiser 
    Visualiser visualiser(height, width);
    visualiser.enemy = &enemy;
    visualiser.robot = &robot;
    visualiser.baseB = &baseB;
    visualiser.baseY = &baseY;
    visualiser.ramp = &ramp;
    visualiser.M20 = &M20;
    visualiser.M21 = &M21;
    visualiser.M22 = &M22;
    visualiser.M23 = &M23;
    visualiser.cluster1 = &cluster1;
    visualiser.cluster2 = &cluster2;
    visualiser.cluster3 = &cluster3;
    visualiser.cluster4 = &cluster4;
    visualiser.cluster5 = &cluster5;
    visualiser.cluster6 = &cluster6;
    visualiser.cluster7 = &cluster7;
    visualiser.cluster8 = &cluster8;
    visualiser.cluster9 = &cluster9;
    visualiser.cluster10 = &cluster10;
    visualiser.constructionArea1 = &constructionArea1;
    visualiser.constructionArea2 = &constructionArea2;
    visualiser.constructionArea3 = &constructionArea3;
    visualiser.constructionArea4 = &constructionArea4;
    visualiser.constructionArea5 = &constructionArea5;
    visualiser.constructionArea6 = &constructionArea6;

    visualiser.construction12IM = &areaYFrontEl;
    visualiser.construction34IM = &areaYSideEl;
    visualiser.construction5IM = &constYFrontEl;
    visualiser.construction6IM = &constYEdgeEl;

    visualiser.constructionB1 = &constBFrontEl;
    visualiser.constructionB2 = &constBEdgeEl;
    visualiser.constructionB3Big = &areaBFrontEl;
    visualiser.constructionB4Big = &areaBSideEl;

    // strategy
    Strategy strategy;

    strategy.enemy = &enemy;
    strategy.robot = &robot;

    strategy.visualiser = &visualiser;
    strategy.navigator = &navigator;
    strategy.locator = &locator;

    strategy.cluster1 = &cluster1;
    strategy.cluster2 = &cluster2;
    strategy.cluster3 = &cluster3;
    strategy.cluster4 = &cluster4;
    strategy.cluster5 = &cluster5;
    strategy.cluster6 = &cluster6;
    strategy.cluster7 = &cluster7;
    strategy.cluster8 = &cluster8;
    strategy.cluster9 = &cluster9;
    strategy.cluster10 = &cluster10;

    strategy.constructionArea1 = &constructionArea1;
    strategy.constructionArea2 = &constructionArea2;
    strategy.constructionArea3 = &constructionArea3;
    strategy.constructionArea4 = &constructionArea4;
    strategy.constructionArea5 = &constructionArea5;
    strategy.constructionArea6 = &constructionArea6;

    waitKey(3000);
    // visualiser.drawImage();
    // locator.start(7);
    visualiser.drawImage();
    strategy.start_test();
    waitKey(0);

    return 0;
}
