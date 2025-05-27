#pragma once
#include <opencv2/opencv.hpp>
#include "elements/element.hpp"
#include "elements/cluster.hpp"
#include "elements/construction_area.hpp"
#include "elements/robot.hpp"
#include "elements/enemy.hpp"
#include "elements/sima.hpp"
#include "navigator.hpp"
#include "locator.hpp"
#include "visualiser.hpp"
#include "strategy.hpp"
#include <client.hpp>

namespace robotVars {
  extern const float cameraAngle;
}

namespace locatorVars {
  extern const float stationaryMarkerSize;
  extern const float movingMarker;
  extern const vector<Point3f> realMPoints;
}

// Basic constants
extern const float ROBOT_RADIUS;
extern const float ROBOT_SIZE;
extern const float ENEMY_SIZE;

extern const string SERVER_IP;
extern const int SERVER_PORT;

extern int width, height;
extern Rect matBoundary;
extern Subdiv2D subdiv;

// Elements
extern Element ramp, baseY, baseB;
extern Element M20, M21, M22, M23;
extern Element areaYSideEl, areaBSideEl, areaYFrontEl, areaBFrontEl;
extern Element constYFrontEl, constBFrontEl, constYEdgeEl, constBEdgeEl;

// Clusters
extern Cluster cluster1, cluster2, cluster3, cluster4, cluster5;
extern Cluster cluster6, cluster7, cluster8, cluster9, cluster10;
extern vector<Cluster*> clusters;

// Construction areas
extern ConstructionArea constructionAreaY1, constructionAreaY2, constructionAreaY3;
extern ConstructionArea constructionAreaY4, constructionAreaY5, constructionAreaY6;

extern ConstructionArea constructionAreaB1, constructionAreaB2, constructionAreaB3;
extern ConstructionArea constructionAreaB4, constructionAreaB5, constructionAreaB6;

// Simas
extern Sima simaY1, simaY2, simaY3, simaY4;
extern Sima simaB1, simaB2, simaB3, simaB4;

// Core system components
extern Locator locator;
extern Visualiser visualiser;
extern Strategy strategy;
extern Navigator navigator;
extern Enemy enemy;
extern Robot robot;

extern RobotClient robotClient;

// Setup functions
void setUpEnvironment();
void setUpYellow(int ourMarkerId, int theirMarkerId);
void setUpBlue(int ourMarkerId, int theirMarkerId);