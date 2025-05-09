#include "locator.hpp"

#include <chrono>

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
    aruco::DetectorParameters parameters;
    detector = aruco::ArucoDetector(dictionary, parameters);

    // realMPoints = {
    //     {2400, 1400, 0},
    //     {600, 1400, 0},
    //     {2400, 600, 0},
    //     {600, 600, 0}
    // };
    // realMPoints = {
    //     {70, 60, 0},
    //     {65, 695, 0},
    //     {1335, 65, 0},
    //     {1328, 694, 0}
    // };
    realMPoints = {
        {1218, 1230, 0},
        {1216, 55, 0},
        {140, 60, 0},
        {142, 1230, 0}
    };

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

Point2f Locator::find(int markerId, const Mat& frame) {
    Mat grayFrame;
    vector<vector<Point2f>> markerCorners;
    vector<int> markerIds;
    vector<vector<Point2f>> rejectedCandidates;
    Mat rvecBoard, tvecBoard, R_board;

    vector<Point3f> markerModel = {
        {-movingMarker / 2, movingMarker / 2, 0},
        { movingMarker / 2, movingMarker / 2, 0},
        { movingMarker / 2, -movingMarker / 2, 0},
        {-movingMarker / 2, -movingMarker / 2, 0}
    };

    if (frame.empty()) return {-1, -1};

    cvtColor(frame, grayFrame, COLOR_BGR2GRAY);
    detector.detectMarkers(grayFrame, markerCorners, markerIds, rejectedCandidates);

    vector<Point2f> boardImagePoints;
    vector<Point3f> boardRealPoints;
    Mat rvec, tvec;
    bool markerFound = false;

    for (size_t i = 0; i < markerIds.size(); ++i) {
        int id = markerIds[i];

        if (id >= 0 && id <= 3) {
            boardImagePoints.push_back(markerCorners[i][0]);
            boardRealPoints.push_back(realMPoints[id]);
        }

        if (id == markerId) {
            markerFound = solvePnP(markerModel, markerCorners[i], cameraMatrix, distCoeffs, rvec, tvec, false, SOLVEPNP_IPPE_SQUARE);
        }
    }

    bool boardSolved = false;
    if (boardImagePoints.size() >= 4) {
        boardSolved = solvePnP(boardRealPoints, boardImagePoints, cameraMatrix, distCoeffs, rvecBoard, tvecBoard, false, SOLVEPNP_ITERATIVE);
        if (boardSolved) {
            Rodrigues(rvecBoard, R_board);
            lastRBoard = R_board.clone();
            lastTBoard = tvecBoard.clone();
            hasValidBoardPose = true;
        }
    }
    
    if (markerFound && hasValidBoardPose) {
        Mat pos = lastRBoard.t() * (tvec - lastTBoard);
        return Point2f(static_cast<float>(pos.at<double>(0)), static_cast<float>(pos.at<double>(1)));
    }

    return {-1, -1};
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
