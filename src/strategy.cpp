#include "strategy.hpp"

Strategy::Strategy() {}

void Strategy::start_test() {
    startAsyncPositionUpdates();
    targetCluster1 = getHighestPriorityCluster();
    while (targetCluster1 != nullptr) {
        getCluster(StrategyStatus::COLLECTING_CLUSTER_1, StrategyStatus::COLLECTED_CLUSTER_1, StrategyStatus::ERROR_COLLECTING_CLUSTER);
        getCluster(StrategyStatus::COLLECTING_CLUSTER_2, StrategyStatus::COLLECTED_CLUSTER_2, StrategyStatus::ERROR_COLLECTING_CLUSTER);
        putCluster(StrategyStatus::CONSTRUCTION_GOING, StrategyStatus::CONSTRUCTION_FINISHED, StrategyStatus::ERROR_CONSTRUCTION);
    }
    stopAsyncPositionUpdates();
}

void Strategy::updatePositions() {
    robot->update();
    enemy->update();
}

void Strategy::startAsyncPositionUpdates() {
    cap.open(1);  // Moved from Locator
    if (!cap.isOpened()) {
        cerr << "Error: Could not open the camera!" << endl;
        exit(-1);
    }

    running = true;
    visualiserRunning = true;

    positionThread = std::thread([this]() {
        Mat frame;
        while (running) {
            cap.read(frame);

            Point2f robotPos = locator->find(robot->getMarkerId(), frame);
            Point2f enemyPos = locator->find(enemy->getMarkerId(), frame);

            robot->setPosition(robotPos);
            enemy->setPosition(enemyPos);

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    visualiserThread = thread([this]() {
        while (visualiserRunning) {
            {
                lock_guard<mutex> lock(visualiserMutex);
                visualiser->updateFrame();
            }
            this_thread::sleep_for(chrono::milliseconds(15));
        }
    });
}

void Strategy::stopAsyncPositionUpdates() {
    running = false;
    visualiserRunning = false;

    if (positionThread.joinable()) positionThread.join();
    if (visualiserThread.joinable()) visualiserThread.join();

    cap.release();
}

void Strategy::changeStatus(StrategyStatus newStatus) {
    previousStatus = currentStatus;
    currentStatus = newStatus;
}

void Strategy::setStatus(StrategyStatus newStatus) {
    previousStatus = currentStatus;
    currentStatus = newStatus;
}

void Strategy::setTargetPath(vector<Point2f> path) {
    if (path.size() > 1) {
        targetPath = path;
    }

    setTargetPath();
}
void Strategy::setTargetPath() {
    if (targetPath.size() > 1) {
        targetPath[0] = robot->getPosition();
        lock_guard<mutex> lock(visualiserMutex);
        visualiser->path = targetPath;
    }
}

void Strategy::getCluster(StrategyStatus continuingStatus, StrategyStatus completeStatus, StrategyStatus errorStatus) {
    while (true) {
        setStatus(continuingStatus);
        // updatePositions();
        targetCluster = getHighestPriorityCluster();

        // if (!targetCluster) {
        //     setStatus(errorStatus);
        //     setTargetPath();
        //     continue;
        // }

        // vector<Point2f> path = getPathToCluster(targetCluster);
        // setTargetPath(path);
        // if (path.size() > 0) {
        //     lock_guard<mutex> lock(visualiserMutex);
        //     visualiser->path = path;
        // }

        if (targetCluster) {
            vector<Point2f> path = getPathToCluster(targetCluster);
            setTargetPath(path);
        } else {
            setTargetPath();
        }

        float dist = navigator->distanceFromPath(targetPath);

        if (dist > 0 && dist < maxAccDistance) {
            if (targetCluster) {
                targetCluster->setStatus(Cluster::ClusterStatus::TAKEN);
            } else {
                getClosestCluster(robot->getPosition())->setStatus(Cluster::ClusterStatus::TAKEN);
            }
            setStatus(completeStatus);
            return;
        }

        // {
        //     lock_guard<mutex> lock(visualiserMutex);
        //     visualiser->drawImage();
        // }
    }
}

void Strategy::putCluster(StrategyStatus continuingStatus, StrategyStatus completeStatus, StrategyStatus errorStatus) {
    while (true) {
        setStatus(continuingStatus);
        // updatePositions();
        targetConstructionArea = getHighestPriorityConstructionArea();

        // if (!targetConstructionArea) {
        //     setStatus(errorStatus);
        //     setTargetPath();
        //     continue;
        // }

        if (targetConstructionArea) {
            vector<Point2f> path = navigator->navigate(robot->getPosition(), targetConstructionArea->getAccessPoint());
            setTargetPath(path);
        } else {
            setTargetPath();
        }

        float dist = navigator->distanceFromPath(targetPath);

        if (dist > 0 && dist < maxAccDistance) {
            if (targetConstructionArea) {
                targetConstructionArea->built = true;
            } else {
                getClosestConstructionArea(robot->getPosition())->built = true;
            }
            targetConstructionArea->built = true;
            setStatus(completeStatus);
            return;
        }

        // {
        //     lock_guard<mutex> lock(visualiserMutex);
        //     visualiser->drawImage();
        // }
    }
}

vector<Cluster*> Strategy::getAvailableClusters() {
    vector<Cluster*> clusters;
    if (cluster1->status == Cluster::ClusterStatus::AVAILABLE) clusters.push_back(cluster1);
    if (cluster2->status == Cluster::ClusterStatus::AVAILABLE) clusters.push_back(cluster2);
    if (cluster3->status == Cluster::ClusterStatus::AVAILABLE) clusters.push_back(cluster3);
    if (cluster4->status == Cluster::ClusterStatus::AVAILABLE) clusters.push_back(cluster4);
    if (cluster5->status == Cluster::ClusterStatus::AVAILABLE) clusters.push_back(cluster5);
    if (cluster6->status == Cluster::ClusterStatus::AVAILABLE) clusters.push_back(cluster6);
    if (cluster7->status == Cluster::ClusterStatus::AVAILABLE) clusters.push_back(cluster7);
    if (cluster8->status == Cluster::ClusterStatus::AVAILABLE) clusters.push_back(cluster8);
    if (cluster9->status == Cluster::ClusterStatus::AVAILABLE) clusters.push_back(cluster9);
    if (cluster10->status == Cluster::ClusterStatus::AVAILABLE) clusters.push_back(cluster10);
    return clusters;
}

vector<ConstructionArea*> Strategy::getAvailableConstructionAreas() {
    vector<ConstructionArea*> areas;
    if (!constructionArea1->built) areas.push_back(constructionArea1);
    if (constructionArea1->built && !constructionArea2->built) areas.push_back(constructionArea2);
    if (!constructionArea3->built) areas.push_back(constructionArea3);
    if (constructionArea3->built && !constructionArea4->built) areas.push_back(constructionArea4);
    if (cluster8->status == Cluster::ClusterStatus::TAKEN && !constructionArea5->built) areas.push_back(constructionArea5);
    if (cluster2->status == Cluster::ClusterStatus::TAKEN && !constructionArea6->built) areas.push_back(constructionArea6);
    return areas;
}

Cluster* Strategy::getHighestPriorityCluster() {
    auto clusters = getAvailableClusters();
    Cluster* target = nullptr;
    float minDistance = 6000;

    for (Cluster* cluster : clusters) {
        auto path = getPathToCluster(cluster);
        float dist = navigator->distanceFromPath(path);
        if (dist < minDistance && dist != -1) {
            minDistance = dist;
            target = cluster;
        }
    }
    return target;
}

ConstructionArea* Strategy::getHighestPriorityConstructionArea() {
    auto areas = getAvailableConstructionAreas();
    ConstructionArea* target = nullptr;
    float minDistance = 6000;

    for (ConstructionArea* area : areas) {
        float dist = navigator->distanceFromPath(navigator->navigate(robot->getPosition(), area->getAccessPoint()));
        if (dist < minDistance && dist != -1) {
            minDistance = dist;
            target = area;
        }
    }
    return target;
}

vector<Point2f> Strategy::getPathToCluster(Cluster* cluster) {
    auto path1 = navigator->navigate(robot->getPosition(), cluster->getAccessPoint(0));
    auto path2 = navigator->navigate(robot->getPosition(), cluster->getAccessPoint(1));

    float dist1 = navigator->distanceFromPath(path1);
    float dist2 = navigator->distanceFromPath(path2);

    if (dist1 != -1 && (dist1 < dist2 || dist2 == -1)) return path1;
    if (dist2 != -1) return path2;

    return {};
}

Cluster* Strategy::getClosestCluster(const Point2f& fromPoint) {
    Cluster* closest = nullptr;
    float minDist = std::numeric_limits<float>::max();

    for (Cluster* cluster : getAvailableClusters()) {
        // Check both access points
        for (int i = 0; i < 2; ++i) {
            float dist = norm(fromPoint - cluster->getAccessPoint(i));
            if (dist < minDist) {
                minDist = dist;
                closest = cluster;
            }
        }
    }

    return closest;
}

ConstructionArea* Strategy::getClosestConstructionArea(const Point2f& fromPoint) {
    ConstructionArea* closest = nullptr;
    float minDist = std::numeric_limits<float>::max();

    for (ConstructionArea* area : getAvailableConstructionAreas()) {
        float dist = norm(fromPoint - area->getAccessPoint());
        if (dist < minDist) {
            minDist = dist;
            closest = area;
        }
    }

    return closest;
}

string Strategy::strategyStatusToString(StrategyStatus status) {
    switch (status) {
        case StrategyStatus::IDLE: return "IDLE";
        case StrategyStatus::DEPLOYING_FLAG: return "DEPLOYING_FLAG";
        case StrategyStatus::FLAG_DEPLOYED: return "FLAG_DEPLOYED";
        case StrategyStatus::COLLECTING_CLUSTER_1: return "COLLECTING_CLUSTER_1";
        case StrategyStatus::COLLECTED_CLUSTER_1: return "COLLECTED_CLUSTER_1";
        case StrategyStatus::COLLECTING_CLUSTER_2: return "COLLECTING_CLUSTER_2";
        case StrategyStatus::COLLECTED_CLUSTER_2: return "COLLECTED_CLUSTER_2";
        case StrategyStatus::CONSTRUCTION_GOING: return "CONSTRUCTION_GOING";
        case StrategyStatus::CONSTRUCTION_FINISHED: return "CONSTRUCTION_FINISHED";
        case StrategyStatus::GOING_BASE: return "GOING_BASE";
        case StrategyStatus::BASED: return "BASED";
        case StrategyStatus::ERROR_COLLECTING_CLUSTER: return "ERROR_COLLECTING_CLUSTER";
        case StrategyStatus::ERROR_CONSTRUCTION: return "ERROR_CONSTRUCTION";
        default: return "UNKNOWN";
    }
}
