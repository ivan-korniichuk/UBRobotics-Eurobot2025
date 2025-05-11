#pragma once
#include <opencv2/opencv.hpp>
#include "elements/element.hpp"
#include "elements/cluster.hpp"
#include "elements/construction_area.hpp"
#include "elements/robot.hpp"
#include "elements/enemy.hpp"
#include "navigator.hpp"
#include "locator.hpp"
#include "visualiser.hpp"
#include "strategy.hpp"

// Basic constants
extern const float ROBOT_RADIUS;
extern const float ROBOT_SIZE;
extern const float ENEMY_SIZE;

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
extern ConstructionArea constructionArea1, constructionArea2, constructionArea3;
extern ConstructionArea constructionArea4, constructionArea5, constructionArea6;

// Core system components
extern Locator locator;
extern Visualiser visualiser;
extern Strategy strategy;
extern Navigator navigator;
extern Enemy enemy;
extern Robot robot;

// Setup functions
void setUpEnvironment();
void setUpYellow(int ourMarkerId, int theirMarkerId);
void setUpBlue(int ourMarkerId, int theirMarkerId);