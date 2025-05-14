#include "locator.hpp"
#include <chrono>
#include <config.hpp>

using namespace locatorVars;

Locator::Locator() {
    FileStorage fs("camera_params.yml", FileStorage::READ);
    if (fs.isOpened()) {
        fs["camera_matrix"] >> cameraMatrix;
        fs["distortion_coefficients"] >> distCoeffs;
        fs.release();
    } else {
        cerr << "Failed to open camera parameters file!" << endl;
        exit(-1);
    }

    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
    aruco::DetectorParameters parameters = aruco::DetectorParameters();

    // parameters.adaptiveThreshWinSizeMin = 5;
    // parameters.adaptiveThreshWinSizeMax = 35;
    // parameters.adaptiveThreshWinSizeStep = 5;

    // parameters.polygonalApproxAccuracyRate = 0.04;

    // parameters.cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
    // parameters.cornerRefinementWinSize = 5;
    // parameters.cornerRefinementMaxIterations = 30;
    // parameters.cornerRefinementMinAccuracy = 0.01;

    detector = aruco::ArucoDetector(dictionary, parameters);


    cap.open(1);
    if (!cap.isOpened()) {
        cerr << "Error: Could not open the camera!" << endl;
        exit(-1);
    }
}

void Locator::drawMarkers(Mat& frame, const vector<vector<Point2f>>& markerCorners, const vector<int>& markerIds) {
    for (size_t i = 0; i < markerIds.size(); i++) {
        int id = markerIds[i];
        const auto& corners = markerCorners[i];

        for (int j = 0; j < 4; j++) {
            line(frame, corners[j], corners[(j + 1) % 4], Scalar(0, 255, 0), 3);
        }

        putText(frame, std::to_string(id), corners[0], FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);
    }
}

void Locator::estimateCameraPose() {
    Mat frame, grayFrame;
    vector<vector<Point2f>> markerCorners;
    vector<int> markerIds;
    vector<vector<Point2f>> rejectedCandidates;
    Mat rvec, tvec;

    cout << "Starting pose estimation over " << POSE_SAMPLE_LIMIT << " frames..." << endl;

    while (poseSamplesCollected < POSE_SAMPLE_LIMIT) {
        if (!cap.read(frame)) continue;
        // frame = imread("/Users/ivankorniichuk/Documents/UBR/Eurobot 2025/Eurobot2025/Robot_Controls/extra/img_00.jpg");

        cvtColor(frame, grayFrame, COLOR_BGR2GRAY);
        detector.detectMarkers(grayFrame, markerCorners, markerIds, rejectedCandidates);

        vector<Point2f> boardImagePoints;
        vector<Point3f> boardRealPoints;

        for (size_t i = 0; i < markerIds.size(); ++i) {
            int id = markerIds[i];
            if (id >= 20 && id <= 23) {
                int mIndex = id % 20;
                Point2f markerCenter = (markerCorners[i][0] + markerCorners[i][1] +
                    markerCorners[i][2] + markerCorners[i][3]) / 4;
                boardImagePoints.push_back(markerCenter);
                boardRealPoints.push_back(realMPoints[mIndex]);
            }
        }

        if (boardImagePoints.size() >= 4) {
            bool ok = solvePnP(boardRealPoints, boardImagePoints, cameraMatrix, distCoeffs, rvec, tvec);
            if (ok) {
                Mat R;
                Rodrigues(rvec, R);
                accumulatedR += R;
                accumulatedT += tvec;
                poseSamplesCollected++;
            }
        }
    }

    rodMain = accumulatedR / POSE_SAMPLE_LIMIT;
    tvecMain = accumulatedT / POSE_SAMPLE_LIMIT;
    cameraPoseFixed = true;
    cout << "Camera pose fixed." << endl;
}

Point2f Locator::find(int movingMarkerId, const cv::Mat& frame) {
    if (!cameraPoseFixed) {
        std::cerr << "Error: Camera pose not yet fixed. Call estimateCameraPose() first." << std::endl;
        return Point2f(-1, -1);
    }

    Mat grayFrame;
    vector<vector<Point2f>> markerCorners;
    vector<int> markerIds;
    vector<vector<Point2f>> rejectedCandidates;

    vector<Point3f> realMarkerPoints = {
        {-movingMarker / 2, movingMarker / 2, 0},
        { movingMarker / 2, movingMarker / 2, 0},
        { movingMarker / 2, -movingMarker / 2, 0},
        {-movingMarker / 2, -movingMarker / 2, 0}
    };

    cvtColor(frame, grayFrame, COLOR_BGR2GRAY);
    detector.detectMarkers(grayFrame, markerCorners, markerIds, rejectedCandidates);

    Mat rvecMarker, tvecMarker;
    bool markerDetected = false;

    for (size_t i = 0; i < markerIds.size(); ++i) {
        if (markerIds[i] == movingMarkerId) {
            markerDetected = solvePnP(realMarkerPoints, markerCorners[i], cameraMatrix, distCoeffs, rvecMarker, tvecMarker, false, SOLVEPNP_IPPE_SQUARE);
            break;
        }
    }

    if (markerDetected) {
        Mat markerWorldPos = rodMain.t() * (tvecMarker - tvecMain);
        return Point2f(
            static_cast<float>(markerWorldPos.at<double>(0, 0)),
            static_cast<float>(markerWorldPos.at<double>(1, 0))
        );
    }

    return Point2f(-1, -1);
}

Pose2D Locator::findWithYaw(int movingMarkerId, const cv::Mat& frame) {
    if (!cameraPoseFixed) {
        std::cerr << "Error: Camera pose not yet fixed. Call estimateCameraPose() first." << std::endl;
        return Pose2D{Point2f(-1.0f, -1.0f), -999.0f};
    }

    Mat grayFrame;
    vector<vector<Point2f>> markerCorners;
    vector<int> markerIds;
    vector<vector<Point2f>> rejectedCandidates;

    vector<Point3f> realMarkerPoints = {
        {-movingMarker / 2, movingMarker / 2, 0},
        { movingMarker / 2, movingMarker / 2, 0},
        { movingMarker / 2, -movingMarker / 2, 0},
        {-movingMarker / 2, -movingMarker / 2, 0}
    };

    cvtColor(frame, grayFrame, COLOR_BGR2GRAY);
    detector.detectMarkers(grayFrame, markerCorners, markerIds, rejectedCandidates);

    Mat rvecMarker, tvecMarker;
    bool markerDetected = false;

    for (size_t i = 0; i < markerIds.size(); ++i) {
        if (markerIds[i] == movingMarkerId) {
            markerDetected = solvePnP(realMarkerPoints, markerCorners[i], cameraMatrix, distCoeffs, rvecMarker, tvecMarker, false, SOLVEPNP_IPPE_SQUARE);
            break;
        }
    }

    if (markerDetected) {
        Mat markerWorldPos = rodMain.t() * (tvecMarker - tvecMain);


        Mat R_marker;
        Rodrigues(rvecMarker, R_marker);
        Mat R_board_relative = rodMain.t() * R_marker;
        double yaw = atan2(R_board_relative.at<double>(1, 0), R_board_relative.at<double>(0, 0));
        double yaw_deg = yaw * 180.0 / CV_PI;
        
        Pose2D pose;
        pose.position = cv::Point2f(
            static_cast<float>(markerWorldPos.at<double>(0, 0)),
            static_cast<float>(markerWorldPos.at<double>(1, 0))
        );
        pose.yaw = yaw_deg;
        return pose;
    }

    return Pose2D{Point2f(-1.0f, -1.0f), -999.0f};
}

Point2f Locator::find(int movingMarkerId) {
    Mat frame, grayFrame;
    vector<vector<Point2f>> markerCorners;
    vector<int> markerIds;
    vector<vector<Point2f>> rejectedCandidates;

    Mat rvecBoard, tvecBoard, R_board;

    vector<Point3f> robotMarkerPoints = {
        {-movingMarker / 2, movingMarker / 2, 0},
        { movingMarker / 2, movingMarker / 2, 0},
        { movingMarker / 2, -movingMarker / 2, 0},
        {-movingMarker / 2, -movingMarker / 2, 0}
    };

    // Only attempt a single frame read per call
    if (cap.read(frame)) {
        cvtColor(frame, grayFrame, COLOR_BGR2GRAY);
        detector.detectMarkers(grayFrame, markerCorners, markerIds, rejectedCandidates);

        vector<Point2f> boardImagePoints;
        vector<Point3f> boardRealPoints;
        Mat rvecRobot, tvecRobot;
        bool robotDetected = false;

        if (!markerIds.empty()) {
            for (size_t i = 0; i < markerIds.size(); i++) {
                int id = markerIds[i];

                if (id >= 0 && id <= 3) {
                    boardImagePoints.push_back(markerCorners[i][0]);
                    boardRealPoints.push_back(realMPoints[id]);
                }

                if (id == movingMarkerId) {
                    robotDetected = solvePnP(robotMarkerPoints, markerCorners[i],
                                             cameraMatrix, distCoeffs,
                                             rvecRobot, tvecRobot, false, SOLVEPNP_IPPE_SQUARE);
                }
            }

            if (boardImagePoints.size() >= 4 && robotDetected) {
                bool boardFound = solvePnP(boardRealPoints, boardImagePoints,
                                           cameraMatrix, distCoeffs,
                                           rvecBoard, tvecBoard, false, SOLVEPNP_ITERATIVE);

                if (boardFound) {
                    Rodrigues(rvecBoard, R_board);
                    Mat robotPosWorld = R_board.t() * (tvecRobot - tvecBoard);
                    float x = static_cast<float>(robotPosWorld.at<double>(0));
                    float y = static_cast<float>(robotPosWorld.at<double>(1));
                    return Point2f(x, y);
                }
            }
        }
    }

    // Return invalid position if something fails
    return Point2f(-1, -1);
}

Point2f Locator::findRecalculating(int movingMarkerId, const cv::Mat& frame) {
    Mat grayFrame;
    vector<vector<Point2f>> markerCorners;
    vector<int> markerIds;
    vector<vector<Point2f>> rejectedCandidates;

    vector<Point3f> realMarkerPoints = {
        {-movingMarker / 2, movingMarker / 2, 0},
        { movingMarker / 2, movingMarker / 2, 0},
        { movingMarker / 2, -movingMarker / 2, 0},
        {-movingMarker / 2, -movingMarker / 2, 0}
    };

    cvtColor(frame, grayFrame, COLOR_BGR2GRAY);
    detector.detectMarkers(grayFrame, markerCorners, markerIds, rejectedCandidates);

    vector<Point2f> imagePoints(4);
    vector<bool> added(4, false);
    bool success = false, markerDetected = false;

    Mat rvec, tvec, R;
    Mat rvecMarker, tvecMarker;

    if (!markerIds.empty()) {
        for (size_t i = 0; i < markerIds.size(); ++i) {
            int id = markerIds[i];

            if (id >= 0 && id <= 3) {
                int mIndex = id % 20;
                Point2f markerCenter = (markerCorners[i][0] + markerCorners[i][1] +
                                        markerCorners[i][2] + markerCorners[i][3]) / 4;
                imagePoints[mIndex] = markerCenter;
                added[mIndex] = true;
            }

            if (id == movingMarkerId) {
                markerDetected = true;
                solvePnP(realMarkerPoints, markerCorners[i], cameraMatrix, distCoeffs, rvecMarker, tvecMarker);
            }
        }

        bool allInserted = all_of(added.begin(), added.end(), [](bool x) { return x; });

        if (allInserted) {
            success = solvePnP(realMPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
            if (success) {
                Rodrigues(rvec, R);
                if (rodMain.empty()) {
                    rodMain = R;
                    tvecMain = tvec;
                }
            }
        }
    }

    if (markerDetected && !rodMain.empty()) {
        Mat R5;
        Rodrigues(rvecMarker, R5);
        Mat markerWorldPos = rodMain.t() * (tvecMarker - tvecMain);
        markerWorldPos.at<double>(2, 0) *= -1;

        return Point2f(markerWorldPos.at<double>(0, 0), markerWorldPos.at<double>(1, 0));
    }

    return Point2f(-1, -1);
}

void Locator::start(int movingMarkerId) {
    Mat frame, grayFrame;
    vector<vector<Point2f>> markerCorners;
    vector<int> markerIds;
    vector<vector<Point2f>> rejectedCandidates;

    Mat rvecBoard, tvecBoard, R_board;

    vector<Point3f> robotMarkerPoints = {
        {-movingMarker / 2, movingMarker / 2, 0},
        { movingMarker / 2, movingMarker / 2, 0},
        { movingMarker / 2, -movingMarker / 2, 0},
        {-movingMarker / 2, -movingMarker / 2, 0}
    };

    while (cap.read(frame)) {
        auto startTime = std::chrono::high_resolution_clock::now();
        cvtColor(frame, grayFrame, COLOR_BGR2GRAY);
        detector.detectMarkers(grayFrame, markerCorners, markerIds, rejectedCandidates);

        vector<Point2f> boardImagePoints;
        vector<Point3f> boardRealPoints;
        Mat rvecRobot, tvecRobot;
        bool robotDetected = false;

        if (!markerIds.empty()) {
            drawMarkers(frame, markerCorners, markerIds);

            // Separate board and robot markers
            for (size_t i = 0; i < markerIds.size(); i++) {
                int id = markerIds[i];

                if (id >= 0 && id <= 3) {
                    // Collect stationary markers
                    boardImagePoints.push_back(markerCorners[i][0]);
                    boardRealPoints.push_back(realMPoints[id % 20]);
                }

                if (id == movingMarkerId) {
                    // Detect robot marker position relative to camera
                    robotDetected = solvePnP(robotMarkerPoints, markerCorners[i],
                                             cameraMatrix, distCoeffs,
                                             rvecRobot, tvecRobot, false, SOLVEPNP_IPPE_SQUARE);
                }
            }

            // Solve camera position using stationary markers each frame (essential!)
            if (boardImagePoints.size() >= 4) {
                bool boardFound = solvePnP(boardRealPoints, boardImagePoints,
                                           cameraMatrix, distCoeffs,
                                           rvecBoard, tvecBoard, false, SOLVEPNP_ITERATIVE);

                if (boardFound) {
                    Rodrigues(rvecBoard, R_board); // Convert rotation vector to matrix

                    if (robotDetected) {
                        // Now transform the robot's camera-relative position to global arena coordinates
                        Mat robotPosWorld = R_board.t() * (tvecRobot - tvecBoard);

                        cout << "Robot Position (Arena frame): "
                             << "X: " << robotPosWorld.at<double>(0)
                             << " Y: " << robotPosWorld.at<double>(1)
                             << " Z: " << robotPosWorld.at<double>(2) << endl;
                    }
                }
            }
        }
        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = endTime - startTime;
        double ms = elapsed.count() * 1000.0;
        double fps = 1.0 / elapsed.count();

        std::cout << "Time per iteration: " << ms << " ms | FPS: " << fps << std::endl;
        imshow("Frame", frame);
        waitKey(1);
    }
}
