#include "visualiser.hpp"

Visualiser::Visualiser(int height, int width, Scalar color) {
    this->defaultMat = Mat::zeros(height, width, CV_8UC3);
    this->defaultMat = color;
}

void Visualiser::updateFrame() {
    Mat imgCopy = defaultMat.clone();

    enemy->drawElement(imgCopy);
    robot->drawElement(imgCopy);

    baseB->drawElement(imgCopy);
    baseY->drawElement(imgCopy);
    ramp->drawElement(imgCopy);

    M20->drawElement(imgCopy);
    M21->drawElement(imgCopy);
    M22->drawElement(imgCopy);
    M23->drawElement(imgCopy);

    cluster1->drawElement(imgCopy);
    cluster2->drawElement(imgCopy);
    cluster3->drawElement(imgCopy);
    cluster4->drawElement(imgCopy);
    cluster5->drawElement(imgCopy);
    cluster6->drawElement(imgCopy);
    cluster7->drawElement(imgCopy);
    cluster8->drawElement(imgCopy);
    cluster9->drawElement(imgCopy);
    cluster10->drawElement(imgCopy);

    constructionArea1->drawElement(imgCopy);
    constructionArea2->drawElement(imgCopy);
    constructionArea3->drawElement(imgCopy);
    constructionArea4->drawElement(imgCopy);
    constructionArea5->drawElement(imgCopy);
    constructionArea6->drawElement(imgCopy);

    constructionB1->drawElement(imgCopy);
    constructionB2->drawElement(imgCopy);
    constructionB3Big->drawElement(imgCopy);
    constructionB4Big->drawElement(imgCopy);

    construction12IM->drawElement(imgCopy);
    construction34IM->drawElement(imgCopy);
    construction5IM->drawElement(imgCopy);
    construction6IM->drawElement(imgCopy);

    for (size_t i = 1; i < path.size(); i++) {
        Point2f p1 = path[i - 1];
        Point2f p2 = path[i];
        line(imgCopy, p1, p2, Scalar(255, 255, 255), 5, LINE_AA);
    }

    float yaw = robot->getYaw();

    putText(imgCopy, to_string(static_cast<int>(yaw)) + "*",
    Point2f(20, 80), FONT_HERSHEY_SIMPLEX, 2, Scalar(50, 50, 255), 3);

    string timeText = "Time: " + to_string(static_cast<int>(elapsedTime)) + "s";
    putText(imgCopy, timeText, Point2f(170, 80), FONT_HERSHEY_SIMPLEX, 2, Scalar(50, 50, 255), 3);

    lock_guard<mutex> lock(frameMutex);
    latestFrame = imgCopy;
}

void Visualiser::drawImage() {
    lock_guard<mutex> lock(frameMutex);
    if (!latestFrame.empty()) {
        imshow("VISUALISER", latestFrame);
        waitKey(1);
    }
}

void Visualiser::setElapsedTime(float time) {
    elapsedTime = time;
}