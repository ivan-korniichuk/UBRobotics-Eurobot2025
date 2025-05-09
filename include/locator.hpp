#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

class Locator {
private:
    Mat cameraMatrix, distCoeffs;
    aruco::ArucoDetector detector;
    VideoCapture cap;

    // float stationaryMarkerSize = 100.0f;
    // float movingMarker = 70.0f;
    float stationaryMarkerSize = 83.0f;
    float movingMarker = 83.0f;

    vector<Point3f> realMPoints;
    Mat lastRBoard, lastTBoard;
    bool hasValidBoardPose = false;


    // Point3f M20 = {2400, 1400, 0};
    // Point3f M21 = {600, 1400, 0};
    // Point3f M22 = {2400, 600, 0};
    // Point3f M23 = {600, 600, 0};

    Mat rodMain;
    Mat tvecMain;

public:
    Locator();
    void drawMarkers(Mat &frame, const vector<vector<Point2f>> &markerCorners, const vector<int> &markerIds);
    Point2f find(int movingMarkerId);
    Point2f find(int markerId, const Mat& frame);
    void start(int movingMarkerId = 5);
};
