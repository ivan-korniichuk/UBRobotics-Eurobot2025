#include "config.hpp"

namespace robotVars {
 const float cameraAngle = 0; // set to the correct value
}

namespace locatorVars {
  const float stationaryMarkerSize = 100.0f;
  const float movingMarker = 70.0f;
  const float simaMarker = 80.0f;
  // const float stationaryMarkerSize = 84.0f;
  // const float movingMarker = 70.0f;
  const vector<Point3f> realMPoints = {
    {2400, 1400, 0},
    {600, 1400, 0},
    {2400, 600, 0},
    {600, 600, 0}
  };
  // const vector<Point3f> realMPoints = {
  //     {70, 60, 0},
  //     {65, 695, 0},
  //     {1335, 65, 0},
  //     {1328, 694, 0}
  // };
  // const vector<Point3f> realMPoints = {
  //   {1218, 1230, 0},
  //   {1216, 55, 0},
  //   {140, 60, 0},
  //   {142, 1230, 0}
  // };
}

const string SERVER_IP = "192.168.4.1"; // replace with aclual IP
const int SERVER_PORT = 5005;

const float ROBOT_RADIUS = 210; // avoidance
const float ROBOT_SIZE = 210; // main body

const float SIMA_RADIUS = 75; // avoidance
const float SIMA_SIZE = 75; // main body

const float ENEMY_SIZE = 230; // main body

int width = 3000, height = 2000; // arena size

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

Element ramp(rampVec, Scalar(0, 0, 255), ROBOT_RADIUS);

Element baseY(baseYVec, Scalar(0, 255, 255), ROBOT_RADIUS);

Element baseB(baseBVec, Scalar(255, 0, 0), ROBOT_RADIUS);

Element M20(M20p, Scalar(255, 0, 255));
Element M21(M21p, Scalar(255, 0, 255));
Element M22(M22p, Scalar(255, 0, 255));
Element M23(M23p, Scalar(255, 0, 255));

Cluster cluster1(cluster1Vec, Scalar(120, 255, 0), Cluster::ClusterPos::VERTICAL, ROBOT_RADIUS, "1");
Cluster cluster2(cluster2Vec, Scalar(120, 255, 0), Cluster::ClusterPos::VERTICAL, ROBOT_RADIUS, "2");
Cluster cluster3(cluster3Vec, Scalar(120, 255, 0), Cluster::ClusterPos::HORIZONTAL, ROBOT_RADIUS, "3");
Cluster cluster4(cluster4Vec, Scalar(120, 255, 0), Cluster::ClusterPos::HORIZONTAL, ROBOT_RADIUS, "4");
Cluster cluster5(cluster5Vec, Scalar(120, 255, 0), Cluster::ClusterPos::HORIZONTAL, ROBOT_RADIUS, "5");
Cluster cluster6(cluster6Vec, Scalar(120, 255, 0), Cluster::ClusterPos::HORIZONTAL, ROBOT_RADIUS, "6");
Cluster cluster7(cluster7Vec, Scalar(120, 255, 0), Cluster::ClusterPos::HORIZONTAL, ROBOT_RADIUS, "7");
Cluster cluster8(cluster8Vec, Scalar(120, 255, 0), Cluster::ClusterPos::HORIZONTAL, ROBOT_RADIUS, "8");
Cluster cluster9(cluster9Vec, Scalar(120, 255, 0), Cluster::ClusterPos::VERTICAL, ROBOT_RADIUS, "9");
Cluster cluster10(cluster10Vec, Scalar(120, 255, 0), Cluster::ClusterPos::VERTICAL, ROBOT_RADIUS, "10");

vector<Cluster*> clusters;

Element areaYSideEl(areaYSideVec, Scalar(0, 255, 255), ROBOT_RADIUS);
Element areaBSideEl(areaBSideVec, Scalar(255, 0, 0), ROBOT_RADIUS);

Element areaYFrontEl(areaYFrontVec, Scalar(0, 255, 255), ROBOT_RADIUS);
Element areaBFrontEl(areaBFrontVec, Scalar(255, 0, 0), ROBOT_RADIUS);

Element constYFrontEl(constYFrontVec, Scalar(0, 255, 255), ROBOT_RADIUS);
Element constBFrontEl(constBFrontVec, Scalar(255, 0, 0), ROBOT_RADIUS);

Element constYEdgeEl(constYEdgeVec, Scalar(0, 255, 255), ROBOT_RADIUS);
Element constBEdgeEl(constBEdgeVec, Scalar(255, 0, 0), ROBOT_RADIUS);

// Yellow
vector<Point2f> constructionY1 = { Point2f(1600, 50), Point2f(1600, 200), Point2f(1950, 200), Point2f(1950, 50) };
ConstructionArea constructionAreaY1(constructionY1, Scalar(0, 0, 50),Point2f(1775, 450), Point2f(1775, 425), 0, Point2f(1775, 275), ROBOT_RADIUS, "1");

vector<Point2f> constructionY2 = { Point2f(1600, 300), Point2f(1600, 450), Point2f(1950, 450), Point2f(1950, 300) };
ConstructionArea constructionAreaY2(constructionY2, Scalar(0, 0, 50), Point2f(1775, 700), Point2f(1775, 675), 0, Point2f(1775, 525), ROBOT_RADIUS, "2");

vector<Point2f> constructionY3 = { Point2f(50, 700), Point2f(50, 1050), Point2f(200, 1050), Point2f(200, 700) };
ConstructionArea constructionAreaY3(constructionY3, Scalar(0, 0, 50), Point2f(450, 875), Point2f(425, 875), 0, Point2f(275, 875), ROBOT_RADIUS, "3");

vector<Point2f> constructionY4 = { Point2f(300, 700), Point2f(300, 1050), Point2f(450, 1050), Point2f(450, 700) };
ConstructionArea constructionAreaY4(constructionY4, Scalar(0, 0, 50), Point2f(700, 875), Point2f(675, 875), 0, Point2f(525, 875), ROBOT_RADIUS, "4");

vector<Point2f> constructionY5 = { Point2f(2050, 50), Point2f(2050, 200), Point2f(2400, 200), Point2f(2400, 50) };
ConstructionArea constructionAreaY5(constructionY5, Scalar(0, 0, 50), Point2f(2225, 550), Point2f(2225, 425), 0, Point2f(2225, 275), ROBOT_RADIUS, "5");

vector<Point2f> constructionY6 = { Point2f(50, 50), Point2f(50, 200), Point2f(400, 200), Point2f(400, 50) };
ConstructionArea constructionAreaY6(constructionY6, Scalar(0, 0, 50), Point2f(225, 450), Point2f(225, 425), 0, Point2f(225, 275), ROBOT_RADIUS, "6");


// vector<Point2f> areaBSideVec = { Point2f(2550, 650), Point2f(2550, 1100), Point2f(3000, 1100), Point2f(3000, 650) };
// vector<Point2f> areaBFrontVec = { Point2f(1000, 0), Point2f(1000, 450), Point2f(1450, 450), Point2f(1450, 0) };
// vector<Point2f> constBFrontVec = { Point2f(550, 0), Point2f(550, 150), Point2f(1000, 150), Point2f(1000, 0) };
// vector<Point2f> constBEdgeVec = { Point2f(2550, 0), Point2f(2550, 150), Point2f(3000, 150), Point2f(3000, 0) };

// Blue
vector<Point2f> constructionB1 = { Point2f(2850, 700), Point2f(2850, 1050), Point2f(3000, 1050), Point2f(3000, 700) };
ConstructionArea constructionAreaB1(constructionB1, Scalar(0, 0, 50), Point2f(2550, 875), Point2f(2525, 875), 0, Point2f(2725, 875), ROBOT_RADIUS, "1");

vector<Point2f> constructionB2 = { Point2f(2600, 700), Point2f(2600, 1050), Point2f(2750, 1050), Point2f(2750, 700) };
ConstructionArea constructionAreaB2(constructionB2, Scalar(0, 0, 50), Point2f(2350, 875), Point2f(2325, 875), 0, Point2f(2175, 875), ROBOT_RADIUS, "2");

vector<Point2f> constructionB3 = { Point2f(1050, 0), Point2f(1050, 150), Point2f(1400, 150), Point2f(1400, 0) };
ConstructionArea constructionAreaB3(constructionB3, Scalar(0, 0, 50), Point2f(1225, 425), Point2f(1225, 450), 0, Point2f(1225, 275), ROBOT_RADIUS, "3");

vector<Point2f> constructionB4 = { Point2f(1050, 250), Point2f(1050, 400), Point2f(1400, 400), Point2f(1400, 250) };
ConstructionArea constructionAreaB4(constructionB4, Scalar(0, 0, 50), Point2f(1225, 675), Point2f(1225, 700), 0, Point2f(1225, 525), ROBOT_RADIUS, "4");

vector<Point2f> constructionB5 = { Point2f(2600, 0), Point2f(2600, 150), Point2f(2950, 150), Point2f(2950, 0) };
ConstructionArea constructionAreaB5(constructionB5, Scalar(0, 0, 50), Point2f(2775, 425), Point2f(2775, 450), 0, Point2f(2775, 275), ROBOT_RADIUS, "5");

vector<Point2f> constructionB6 = { Point2f(600, 0), Point2f(600, 150), Point2f(950, 150), Point2f(950, 0) };
ConstructionArea constructionAreaB6(constructionB6, Scalar(0, 0, 50), Point2f(775, 425), Point2f(775, 450), 0, Point2f(775, 275), ROBOT_RADIUS, "6");

Visualiser visualiser(height, width);

Navigator navigator(matBoundary);

Strategy strategy;

Locator locator;

RobotClient robotClient(SERVER_IP, SERVER_PORT); 

Enemy enemy(2, {1500,1750}, Scalar(0,0,255), ENEMY_SIZE, ROBOT_RADIUS);
Robot robotY(7, {2650,1750}, {2650,1750}, Scalar(0,255,255), ROBOT_SIZE, ROBOT_RADIUS);
Robot robotB(7, {350,1750}, {350,1750}, Scalar(255,120,150), ROBOT_SIZE, ROBOT_RADIUS);

Sima simaY71(71, {2900, 1925}, Scalar(255, 255, 0), SIMA_SIZE, SIMA_RADIUS, "71");
Sima simaY72(72, {2900, 1825}, Scalar(255, 255, 0), SIMA_SIZE, SIMA_RADIUS, "72");
Sima simaY74(74, {2900, 1725}, Scalar(255, 255, 0), SIMA_SIZE, SIMA_RADIUS, "74");
Sima simaYDRUM(0, {2900, 1625}, Scalar(255, 255, 0), SIMA_SIZE, SIMA_RADIUS, "D");

Sima simaB51(51, {100, 1625}, Scalar(255, 0, 0), SIMA_SIZE, SIMA_RADIUS, "51");
Sima simaB52(52, {100, 1725}, Scalar(255, 0, 0), SIMA_SIZE, SIMA_RADIUS, "52");
Sima simaB54(54, {100, 1825}, Scalar(255, 0, 0), SIMA_SIZE, SIMA_RADIUS, "54");
Sima simaBDRUM(0, {100, 1925}, Scalar(255, 0, 0), SIMA_SIZE, SIMA_RADIUS, "D");

void setUpEnvironment() {
  cout << "Innit start" << endl;

  locator.estimateCameraPose();
  navigator.insertElement(ramp);

  enemy.locator = &locator;
  navigator.enemy = &enemy;

  //visualiser  
  visualiser.enemy = &enemy;
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

  visualiser.construction12IM = &areaYFrontEl;
  visualiser.construction34IM = &areaYSideEl;
  visualiser.construction5IM = &constYFrontEl;
  visualiser.construction6IM = &constYEdgeEl;

  visualiser.constructionB1 = &constBFrontEl;
  visualiser.constructionB2 = &constBEdgeEl;
  visualiser.constructionB3Big = &areaBFrontEl;
  visualiser.constructionB4Big = &areaBSideEl;

  // strategy
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

  strategy.robotClient = &robotClient;

  cout << "Main Innit done" << endl;
}

void setUpYellow(int ourMaker, int theirMarker) {
  setUpEnvironment();

  enemy.setMarkerId(theirMarker);
  robotY.setMarkerId(ourMaker);

  strategy.enemy = &enemy;
  strategy.robot = &robotY;

  visualiser.robot = &robotY;
  robotY.locator = &locator;

  visualiser.constructionArea1 = &constructionAreaY1;
  visualiser.constructionArea2 = &constructionAreaY2;
  visualiser.constructionArea3 = &constructionAreaY3;
  visualiser.constructionArea4 = &constructionAreaY4;
  visualiser.constructionArea5 = &constructionAreaY5;
  visualiser.constructionArea6 = &constructionAreaY6;

  strategy.constructionArea1 = &constructionAreaY1;
  strategy.constructionArea2 = &constructionAreaY2;
  strategy.constructionArea3 = &constructionAreaY3;
  strategy.constructionArea4 = &constructionAreaY4;
  strategy.constructionArea5 = &constructionAreaY5;
  strategy.constructionArea6 = &constructionAreaY6;

  clusters.push_back(&cluster1);
  clusters.push_back(&cluster2);;
  clusters.push_back(&cluster3);
  clusters.push_back(&cluster4);
  clusters.push_back(&cluster5);
  clusters.push_back(&cluster6);
  clusters.push_back(&cluster7);
  clusters.push_back(&cluster8);
  clusters.push_back(&cluster9);
  clusters.push_back(&cluster10);

  strategy.sima1 = &simaY71;
  strategy.sima2 = &simaY72;
  strategy.sima4 = &simaY74;
  strategy.simaDRUM = &simaYDRUM;

  visualiser.sima1 = &simaY71;
  visualiser.sima2 = &simaY72;
  visualiser.sima4 = &simaY74;
  visualiser.simaDRUM = &simaYDRUM;

  simaY71.robotClient = &robotClient;
  simaY72.robotClient = &robotClient;
  simaY74.robotClient = &robotClient;
  simaYDRUM.robotClient = &robotClient;

  simaY71.setWaypoints({{2750, 1625}, {1350, 1550}, {1100, 1450}});
  simaY72.setWaypoints({{2750, 1625}, {1350, 1550}, {1500, 1450}});
  simaY74.setWaypoints({{2750, 1625}, {1350, 1550}, {1900, 1450}});

  navigator.insertElement(baseB);

  // navigator.insertElement(areaYSideEl);
  navigator.insertElement(areaBSideEl);

  // navigator.insertElement(areaYFrontEl);
  navigator.insertElement(areaBFrontEl);

  // navigator.insertElement(constYFrontEl);
  navigator.insertElement(constBFrontEl);

  // navigator.insertElement(constYEdgeEl);
  navigator.insertElement(constBEdgeEl);

  navigator.setClusters(clusters);

  navigator.setConstructionAreas({&constructionAreaY1, &constructionAreaY2, &constructionAreaY3, &constructionAreaY4, &constructionAreaY5, &constructionAreaY6});

  cout << "Yellow Innit done" << endl;
}

//change constructions
void setUpBlue(int ourMaker, int theirMarker) {
  setUpEnvironment();

  enemy.setMarkerId(theirMarker);
  robotB.setMarkerId(ourMaker);

  strategy.enemy = &enemy;
  strategy.robot = &robotB;

  visualiser.robot = &robotB;
  robotB.locator = &locator;

  visualiser.constructionArea1 = &constructionAreaB1;
  visualiser.constructionArea2 = &constructionAreaB2;
  visualiser.constructionArea3 = &constructionAreaB3;
  visualiser.constructionArea4 = &constructionAreaB4;
  visualiser.constructionArea5 = &constructionAreaB5;
  visualiser.constructionArea6 = &constructionAreaB6;

  strategy.constructionArea1 = &constructionAreaB1;
  strategy.constructionArea2 = &constructionAreaB2;
  strategy.constructionArea3 = &constructionAreaB3;
  strategy.constructionArea4 = &constructionAreaB4;
  strategy.constructionArea5 = &constructionAreaB5;
  strategy.constructionArea6 = &constructionAreaB6;

  clusters.push_back(&cluster1);
  clusters.push_back(&cluster2);;
  clusters.push_back(&cluster3);
  clusters.push_back(&cluster4);
  clusters.push_back(&cluster5);
  clusters.push_back(&cluster6);
  clusters.push_back(&cluster7);
  clusters.push_back(&cluster8);
  clusters.push_back(&cluster9);
  clusters.push_back(&cluster10);

  strategy.sima1 = &simaB51;
  strategy.sima2 = &simaB52;
  strategy.sima4 = &simaB54;
  strategy.simaDRUM = &simaBDRUM;

  visualiser.sima1 = &simaB51;
  visualiser.sima2 = &simaB52;
  visualiser.sima4 = &simaB54;
  visualiser.simaDRUM = &simaBDRUM;

  simaB51.robotClient = &robotClient;
  simaB52.robotClient = &robotClient;
  simaB54.robotClient = &robotClient;
  simaBDRUM.robotClient = &robotClient;

  simaB51.setWaypoints({{250, 1600}, {650, 1250}, {1900, 1450}});
  simaB52.setWaypoints({{250, 1600}, {650, 1250}, {1500, 1450}});
  simaB54.setWaypoints({{250, 1600}, {650, 1250}, {1100, 1450}});

  navigator.insertElement(baseY);

  navigator.insertElement(areaYSideEl);
  // navigator.insertElement(areaBSideEl);

  navigator.insertElement(areaYFrontEl);
  // navigator.insertElement(areaBFrontEl);

  navigator.insertElement(constYFrontEl);
  // navigator.insertElement(constBFrontEl);

  navigator.insertElement(constYEdgeEl);
  // navigator.insertElement(constBEdgeEl);

  navigator.setClusters(clusters);

  navigator.setConstructionAreas({&constructionAreaB1, &constructionAreaB2, &constructionAreaB3, &constructionAreaB4, &constructionAreaB5, &constructionAreaB6});

  cout << "Blue Innit done" << endl;
}