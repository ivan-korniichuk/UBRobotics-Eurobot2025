#include "strategy.hpp"

Strategy::Strategy() {}


// if 0 clusters or 0 conts areas return error don't halt
void Strategy::start_test() {
    startAsyncPositionUpdates();
    robotClient->registerWithRPi();
    robotClient->waitForCordSignal();  
    startTimer();
    targetCluster1 = getHighestPriorityCluster();
    while (targetCluster1 != nullptr) {
        getCluster(StrategyStatus::COLLECTING_CLUSTER_1, StrategyStatus::COLLECTED_CLUSTER_1, StrategyStatus::ERROR_COLLECTING_CLUSTER);
        // getCluster(StrategyStatus::COLLECTING_CLUSTER_2, StrategyStatus::COLLECTED_CLUSTER_2, StrategyStatus::ERROR_COLLECTING_CLUSTER);
        putCluster(StrategyStatus::CONSTRUCTION_GOING, StrategyStatus::CONSTRUCTION_FINISHED, StrategyStatus::ERROR_CONSTRUCTION);
    }
    stopAsyncPositionUpdates();
}

// void Strategy::updatePositions() {
//     robot->update();
//     enemy->update();
// }

void Strategy::startAsyncPositionUpdates() {
    cap.open(1);
    if (!cap.isOpened()) {
        cerr << "Error: Could not open the camera!" << endl;
        exit(-1);
    }

    running = true;
    visualiserRunning = true;
    frameAvailable = false;

    // Camera thread: only reads frames
    cameraThread = thread(&Strategy::runCameraLoop, this);

    // Robot processing thread
    robotThread = thread(&Strategy::runRobotProcessingLoop, this);

    // Enemy processing thread
    enemyThread = thread(&Strategy::runEnemyProcessingLoop, this);

    simaThread1 = thread([this]() {
        while (running) {
            if (!simasActive.load()) {
                this_thread::sleep_for(chrono::milliseconds(100));
                continue;
            }

            Mat frameCopy;
            {
                lock_guard<mutex> lock(frameMutex);
                frameCopy = sharedFrame.clone();
            }

            Pose2D pose = locator->findWithYawSima(sima1->getMarkerId(), frameCopy);
            sima1->setPosition(pose.position);
            sima1->setYaw(pose.yaw);
            sima1->step(0);

            this_thread::sleep_for(chrono::milliseconds(1));
        }
    });

    simaThread2 = thread([this]() {
        bool localActive = false;
        while (running) {
            if (!simasActive.load()) {
                this_thread::sleep_for(chrono::milliseconds(100));
                continue;
            }

            if (!localActive) {
                this_thread::sleep_for(chrono::milliseconds(1500));
                localActive = true;
            }

            Mat frameCopy;
            {
                lock_guard<mutex> lock(frameMutex);
                frameCopy = sharedFrame.clone();
            }

            Pose2D pose = locator->findWithYawSima(sima2->getMarkerId(), frameCopy);
            sima2->setPosition(pose.position);
            sima2->setYaw(pose.yaw);
            sima2->step(1);

            this_thread::sleep_for(chrono::milliseconds(1));
        }
    });

    simaThread4 = thread([this]() {
        bool localActive = false;
        while (running) {
            if (!simasActive.load()) {
                this_thread::sleep_for(chrono::milliseconds(100));
                continue;
            }

            if (!localActive) {
                this_thread::sleep_for(chrono::milliseconds(3000));
                localActive = true;
            }

            Mat frameCopy;
            {
                lock_guard<mutex> lock(frameMutex);
                frameCopy = sharedFrame.clone();
            }

            Pose2D pose = locator->findWithYawSima(sima4->getMarkerId(), frameCopy);
            sima4->setPosition(pose.position);
            sima4->setYaw(pose.yaw);
            sima4->step(3);

            this_thread::sleep_for(chrono::milliseconds(1));
        }
    });

    // Visualiser thread
    visualiserThread = thread([this]() {
        while (visualiserRunning) {
            {
                lock_guard<mutex> lock(visualiserMutex);
                visualiser->updateFrame();
            }
            this_thread::sleep_for(chrono::milliseconds(1));
        }
    });
}

void Strategy::runCameraLoop() {
    Mat frame;
    cout << "[Camera] Thread started" << endl;
    while (running) {

        if (!cap.isOpened()) {
            cerr << "Error: Could not open the camera!" << endl;
            exit(-1);
        }
        auto start = chrono::high_resolution_clock::now();
        bool frameRead;

        {
            lock_guard<mutex> lock(frameMutex);
            frameRead = cap.read(sharedFrame);
        }

        if (frameRead) {
            robotFrameReady.store(true);
            enemyFrameReady.store(true);
        }
        
        auto end = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
        cout << "[Camera] Loop duration: " << duration / 1000.0 << " ms" << endl;
        // this_thread::sleep_for(chrono::milliseconds(1));
    }
}

void Strategy::runRobotProcessingLoop() {
    cout << "[Robot] Thread started" << endl;
    while (running) {
        auto start = chrono::high_resolution_clock::now();
        if (!robotFrameReady.load()) {
            this_thread::sleep_for(chrono::milliseconds(1));
            continue;
        }
        robotFrameReady.store(false);

        Mat frameCopy;
        {
            lock_guard<mutex> lock(frameMutex);
            frameCopy = sharedFrame.clone();
        }

        Pose2D pose = locator->findWithYaw(robot->getMarkerId(), frameCopy);

        if (pose.position != Point2f(-1, -1)) {
            robot->setPosition(pose.position);
            // cout << "strategy yaw: " << pose.yaw << endl;
            // cout << "Robot position: " << pose.position << endl;
            robot->setYaw(pose.yaw);
        }
        auto end = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
        cout << "[Robot] Loop duration: " << duration / 1000.0 << " ms" << endl;
        // this_thread::sleep_for(chrono::milliseconds(1));
    }
}

void Strategy::runEnemyProcessingLoop() {
    cout << "[Enemy] Thread started" << endl;
    while (running) {
        auto start = chrono::high_resolution_clock::now();

        if (!enemyFrameReady.load()) {
            this_thread::sleep_for(chrono::milliseconds(1));
            continue;
        }
        enemyFrameReady.store(false);

        Mat frameCopy;
        {
            lock_guard<mutex> lock(frameMutex);
            frameCopy = sharedFrame.clone();
        }

        Point2f enemyPos = locator->find(enemy->getMarkerId(), frameCopy);

        if (enemyPos != Point2f(-1, -1)) {
            enemy->setPosition(enemyPos);

            float threshold = 160.0f;
            for (Cluster* cluster : getAvailableClusters()) {
                float dist = getDistance(enemyPos, cluster->center);
                if (dist < threshold) {
                    cluster->setStatus(Cluster::ClusterStatus::UNAVAILABLE);
                }
            }
        }
        auto end = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(end - start).count();
        cout << "[Enemy] Loop duration: " << duration / 1000.0 << " ms" << endl;
        this_thread::sleep_for(chrono::milliseconds(10));
    }
}

void Strategy::stopAsyncPositionUpdates() {
    running = false;
    visualiserRunning = false;

    if (cameraThread.joinable()) cameraThread.join();
    if (robotThread.joinable()) robotThread.join();
    if (enemyThread.joinable()) enemyThread.join();
    if (visualiserThread.joinable()) visualiserThread.join();
    if (simaThread1.joinable()) simaThread1.join();
    if (simaThread2.joinable()) simaThread2.join();
    if (simaThread4.joinable()) simaThread4.join();

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
        targetCluster = getHighestPriorityCluster();

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
                targetCluster = getClosestCluster(robot->getPosition());
                if (getDistance(robot->getPosition(), targetCluster->getAccessPoint(0)) < maxAccDistance) {
                    targetCluster->setStatus(Cluster::ClusterStatus::TAKEN);
                } else if (getDistance(robot->getPosition(), targetCluster->getAccessPoint(1)) < maxAccDistance) {
                    targetCluster->setStatus(Cluster::ClusterStatus::TAKEN);
                }
            }
            alignRobot(targetCluster->center);
            // robotClient->sendSeek();
            // this_thread::sleep_for(chrono::milliseconds(5000));
            // robotClient->sendGrab(true);
            break;
        }
    }
    aligningRobot.store(true);
    setTargetPath({});
    robotClient->sendNewESPMoveCommand(0,0,0);
    this_thread::sleep_for(chrono::milliseconds(500));
    robotClient->sendSeek();
    this_thread::sleep_for(chrono::milliseconds(8000));
    robotClient->sendGrab(true);
    this_thread::sleep_for(chrono::milliseconds(500));
    aligningRobot.store(false);

    setStatus(completeStatus);
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
            vector<Point2f> path = navigator->navigate(robot->getPosition(), targetConstructionArea->entry);
            setTargetPath(path);
        } else {
            setTargetPath();
        }

        float dist = navigator->distanceFromPath(targetPath);

        if (dist > 0 && dist < maxAccDistance) {
            if (targetConstructionArea) {
                targetConstructionArea->built = true;
            } else {
                targetConstructionArea = getClosestConstructionArea(robot->getPosition());
                if (getDistance(robot->getPosition(), targetConstructionArea->getAccessPoint()) < maxAccDistance) {
                    targetConstructionArea->built = true;
                }
            }
            alignRobot(targetConstructionArea->center);
            break;
        }
    }

    while (true) {
        vector<Point2f> entryPath = {robot->getPosition(), targetConstructionArea->goal};
        setTargetPath(entryPath);

        float dist = navigator->distanceFromPath(entryPath);

        if (dist > 0 && dist < 20.0f) {
            aligningRobot.store(true);
            setTargetPath({});
            this_thread::sleep_for(chrono::milliseconds(100));
            robotClient->sendNewESPMoveCommand(0,0,0);
            this_thread::sleep_for(chrono::milliseconds(1000));
            robotClient->sendGrab(false);
            break;
        }
    }

    this_thread::sleep_for(chrono::milliseconds(1000));
    aligningRobot.store(false);

    while (true) {
        vector<Point2f> exitPath = {robot->getPosition(), targetConstructionArea->exit};
        setTargetPath(exitPath);

        float dist = navigator->distanceFromPath(exitPath);

        if (dist > 0 && dist < 20.0f) {
            break;
        }
    }

    setStatus(completeStatus);

    visualiser->score += 12; // Increment score for successful construction
    return;
}

float Strategy::getTargetPathDistance() const {
    if (targetPath.size() < 2) return 0.0f;

    Point2f start = targetPath.front();
    Point2f end = targetPath.back();

    float dx = end.x - start.x;
    float dy = end.y - start.y;

    return sqrt(dx * dx + dy * dy);
}

float Strategy::getDistance(Point2f start, Point2f end) const {
    float dx = end.x - start.x;
    float dy = end.y - start.y;

    return sqrt(dx * dx + dy * dy);
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
    float minDist = numeric_limits<float>::max();

    for (Cluster* cluster : getAvailableClusters()) {
        if (cluster->status != Cluster::ClusterStatus::AVAILABLE) continue;
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
    float minDist = numeric_limits<float>::max();

    for (ConstructionArea* area : getAvailableConstructionAreas()) {
        if (area->built) continue;
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

void Strategy::startTimer() {
    startTime = chrono::steady_clock::now();
    bool simasOutDone = false;

    // Start motion control thread
    motionRunning = true;
    cout << "[startTimer] Starting motionControlThread" << endl;
    motionControlThread = thread([this]() {
        this_thread::sleep_for(chrono::milliseconds(100));
        while (motionRunning) {
            // cout << "Robot position: " << robot->getPosition() << endl;
            controlRobotMovement();
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    });

    // Timer logic
    timerThread = thread([this, &simasOutDone]() {
        while (running) {
            auto elapsed = chrono::steady_clock::now() - startTime;
            auto seconds = chrono::duration_cast<chrono::seconds>(elapsed).count();

            visualiser->setElapsedTime(seconds);

            if (!simasOutDone && seconds >= 85) {
                simaDRUM->robotClient->sendSimasCommand(4, 0x00, {static_cast<uint8_t>(simaDRUM->getMarkerId() > 60 ? 1 : 0)});
                visualiser->addScore(55);
                simasActive = true;
                cout << "Simas are now active" << endl;
                simasOutDone = true;
            }

            if (seconds >= 99) {
                // simaDRUM->robotClient->sendSimasCommand(4, 0x0A, {1});
                cout << "Timer reached. Halting strategy." << endl;
                motionRunning = false;

                robotClient->sendNewESPMoveCommand(0, 0, 0);
                sima1->robotClient->sendSimasCommand(0, 0x0A, {1});
                sima2->robotClient->sendSimasCommand(1, 0x0A, {1});
                sima4->robotClient->sendSimasCommand(3, 0x0A, {1});
                simaDRUM->robotClient->sendSimasCommand(4, 0x0A, {1});
                sima1->robotClient->sendSimasCommand(0, 0x09, {0, 1, 0, 1});
                sima2->robotClient->sendSimasCommand(1, 0x09, {0, 1, 0, 1});
                sima4->robotClient->sendSimasCommand(3, 0x09, {0, 1, 0, 1});
                simaDRUM->robotClient->sendSimasCommand(4, 0x09, {0, 1, 0, 1});

                if (motionControlThread.joinable()) motionControlThread.join();
                if (simaThread1.joinable()) simaThread1.join();
                if (simaThread2.joinable()) simaThread2.join();
                if (simaThread4.joinable()) simaThread4.join();

                break;
            }

            this_thread::sleep_for(chrono::milliseconds(100));
        }
    });
}

void Strategy::controlRobotMovement() {
    if (aligningRobot.load()) {
        return;
    }

    if (targetPath.size() < 2) {
        robotClient->sendNewESPMoveCommand(0, 0, 0);
        return;
    };

    Point2f current = targetPath[0];
    Point2f next = targetPath[1];

    float dx = next.x - current.x;
    float dy = -(next.y - current.y);
    float pathAngle = atan2(dy, dx) * 180.0 / CV_PI;  // in degrees

    float robotYaw = robot->getYaw() - 90;  // in degrees
    float robotYawRad = robotYaw * CV_PI / 180.0f;
    // float deltaYaw = pathAngle - robotYaw;

    // // Normalize to [-180, 180]
    // while (deltaYaw > 180) deltaYaw -= 360;
    // while (deltaYaw < -180) deltaYaw += 360;

    float distance = sqrt(dx * dx + dy * dy);
    if (distance < 0.001) {
        robotClient->sendNewESPMoveCommand(0, 0, 0);
        return;
    }

    // float rawSpeed = -0.0004f * distance * distance - 0.79512 * distance + 1;

    float rawSpeed  = 0.000000000255374121 * distance * distance * distance * distance
    - 0.000001007507540 * distance * distance * distance
    + 0.000977126683 * distance * distance
    + 0.355521813 * distance
    + 9.05225505;

    // float rawSpeed = 100.0f * log(0.03f * distance + 1.0f);
    float speed = max(10.0f, min(500.0f, rawSpeed));
    float norm_dx = dx / distance;
    float norm_dy = -dy / distance;

    float localX = cos(robotYawRad) * norm_dx + sin(robotYawRad) * norm_dy;
    float localY = -sin(robotYawRad) * norm_dx + cos(robotYawRad) * norm_dy;

    float scaled_dx = localX * speed;
    float scaled_dy = localY * speed;
    float deltaYaw = pathAngle - robotYaw;

    // Normalize angle to [-180, 180]
    while (deltaYaw > 180) deltaYaw -= 360;
    while (deltaYaw < -180) deltaYaw += 360;

    cout << fixed << setprecision(2);
    cout << "ΔYaw: " << deltaYaw << "°, localX: " << scaled_dx << ", localY: " << scaled_dy << ", robotYaw: " << robotYaw << "°\n";

    robotClient->sendNewESPMoveCommand(-scaled_dx*2.7, -scaled_dy*2.7, 0);
}

void Strategy::alignRobot(const Point2f& targetPos) {
    aligningRobot.store(true);

    robotClient->sendNewESPMoveCommand(0, 0, 0);
    float relativeYaw = 15.0f;
    int tries = 0;
    cout << "Robot alignment angle to target: " << endl;

    while (true) {
        this_thread::sleep_for(chrono::milliseconds(50)); // just in case
        Point2f robotPos = robot->getPosition();
        float robotYaw = robot->getYaw();  // already adjusted by cameraAngle
    
        float dx = targetPos.x - robotPos.x;
        float dy = -(targetPos.y - robotPos.y);

        float distance = sqrt(dx * dx + dy * dy);

        float norm_dx = dx / distance;
        float norm_dy = -dy / distance;
    
        float targetAngle = atan2(norm_dy, norm_dx) * 180.0 / CV_PI;

        // some calculations <3
        relativeYaw = targetAngle - robotYaw;
        if (relativeYaw < -90) relativeYaw += 360;

        if (relativeYaw > 180) relativeYaw -= 360;
        if (relativeYaw < -180) relativeYaw += 360;
    
        // Normalize to [-180, 180]
        // if (relativeYaw < 90) relativeYaw = 180 - relativeYaw;
        // if (relativeYaw > 270) relativeYaw = 180 - relativeYaw;
    
        cout << fixed << setprecision(2);
        cout << "Robot alignment angle to target: " << relativeYaw 
             << "° (Robot Yaw: " << robotYaw 
             << "°, Target Angle: " << targetAngle << "°)\n";
            
        if (relativeYaw <= 3.0f && relativeYaw >= -3.0f) {
            tries++;
            if (tries > 6) {
                robotClient->sendNewESPMoveCommand(0, 0, 0);
                break;
            }
        } else {
            robotClient->sendNewESPMoveCommand(0, 0, relativeYaw*5);
        }
    }
    cout << "stop rotating" << endl;
    robotClient->sendNewESPMoveCommand(0, 0, 0);
    this_thread::sleep_for(chrono::milliseconds(1000));
    aligningRobot.store(false);
}