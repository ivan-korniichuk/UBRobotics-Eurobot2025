#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

struct Pose2D {
    Point2f position;
    float yaw;
};

class Locator {
private:
    Mat cameraMatrix, distCoeffs;
    aruco::ArucoDetector detector;
    VideoCapture cap;

    Mat lastRBoard, lastTBoard;
    bool hasValidBoardPose = false;

    bool cameraPoseFixed = false;
    int poseSamplesCollected = 0;
    const int POSE_SAMPLE_LIMIT = 80;
    Mat accumulatedR = Mat::zeros(3, 3, CV_64F);
    Mat accumulatedT = Mat::zeros(3, 1, CV_64F); 

    Mat rodMain;
    Mat tvecMain;

    Mat arenaHomography;
    bool hasArenaHomography = false;

public:
    Locator();
    void drawMarkers(Mat &frame, const vector<vector<Point2f>> &markerCorners, const vector<int> &markerIds);
    Point2f find(int movingMarkerId);
    Pose2D findWithYaw(int markerId, const Mat& frame);
    Pose2D findWithYawSima(int markerId, const Mat& frame);
    Point2f find(int markerId, const Mat& frame);
    Point2f findSima(int simaId, const Mat& frame);
    Point2f findRecalculating(int markerId, const Mat& frame);
    void start(int movingMarkerId = 5);
    void estimateCameraPose();
};
