#pragma once

#include "elements/cluster.hpp"
#include "elements/construction_area.hpp"
#include "elements/robot.hpp"
#include "elements/enemy.hpp"
#include "elements/sima.hpp"
#include "visualiser.hpp"
#include "navigator.hpp"
#include "client.hpp"
#include "locator.hpp"
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>

using namespace std;
using namespace cv;

class Strategy {
public:
    enum class StrategyStatus {
        IDLE,
        DEPLOYING_FLAG,
        FLAG_DEPLOYED,
        COLLECTING_CLUSTER_1,
        COLLECTED_CLUSTER_1,
        COLLECTING_CLUSTER_2,
        COLLECTED_CLUSTER_2,
        CONSTRUCTION_GOING,
        CONSTRUCTION_FINISHED,
        GOING_BASE,
        BASED,
        ERROR_COLLECTING_CLUSTER,
        ERROR_CONSTRUCTION,
    };

    RobotClient* robotClient;

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

    Enemy* enemy;
    Robot* robot;

    Sima* sima1;
    Sima* sima2;
    Sima* sima4;
    Sima* simaDRUM;

    Visualiser* visualiser;
    Navigator* navigator;
    Locator* locator;

    bool flagDeployed = false;
    float maxAccDistance = 30;
    vector<Point2f> targetPath = {};

    StrategyStatus currentStatus = StrategyStatus::IDLE;
    StrategyStatus previousStatus = StrategyStatus::IDLE;

    mutex visualiserMutex;

    Strategy();

    void start_test();
    // void updatePositions();
    void changeStatus(StrategyStatus newStatus);
    string strategyStatusToString(StrategyStatus status);
    void startAsyncPositionUpdates();
    void stopAsyncPositionUpdates();
    void startTimer();

private:
    Cluster* targetCluster = nullptr;
    Cluster* targetCluster1 = nullptr;
    Cluster* targetCluster2 = nullptr;
    ConstructionArea* targetConstructionArea = nullptr;
    atomic<bool> running;
    VideoCapture cap;
    Mat sharedFrame;
    mutex cameraMutex;
    thread cameraThread;
    thread positionThread;
    thread visualiserThread;
    atomic<bool> visualiserRunning;
    chrono::steady_clock::time_point startTime;
    thread timerThread;
    thread motionControlThread;
    atomic<bool> motionRunning;
    mutex frameMutex;
    
    thread robotThread;
    thread enemyThread;
    atomic<bool> frameAvailable;
    atomic<bool> aligningRobot{false};

    atomic<bool> robotFrameReady{false};
    atomic<bool> enemyFrameReady{false};

    thread simaThread1;
    thread simaThread2;
    thread simaThread4;

    atomic<bool> simasActive{false};

    void controlRobotMovement();
    void alignRobot(const Point2f& targetPosition);
    void setTargetPath(vector<Point2f> path);
    void setTargetPath();
    void setStatus(StrategyStatus newStatus);
    void getCluster(StrategyStatus continuingStatus, StrategyStatus completeStatus, StrategyStatus errorStatus);
    void putCluster(StrategyStatus continuingStatus, StrategyStatus completeStatus, StrategyStatus errorStatus);
    vector<Cluster*> getAvailableClusters();
    vector<ConstructionArea*> getAvailableConstructionAreas();
    Cluster* getHighestPriorityCluster();
    ConstructionArea* getHighestPriorityConstructionArea();
    vector<Point2f> getPathToCluster(Cluster* cluster);
    Cluster* getClosestCluster(const Point2f& fromPoint);
    ConstructionArea* getClosestConstructionArea(const Point2f& fromPoint);
    float getTargetPathDistance() const;
    float getDistance(Point2f start, Point2f end) const;
    void runCameraLoop();
    void runRobotProcessingLoop();
    void runEnemyProcessingLoop();
};
