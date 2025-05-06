#include "locator.hpp"

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

    realMPoints = {
        {2400, 1400, 0},
        {600, 1400, 0},
        {2400, 600, 0},
        {600, 600, 0}
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
    vector<Point2f> imagePoints(4);
    vector<bool> added(4, false);
    Mat rvec, tvec, R;
    Mat rvecMarker, tvecMarker;
    bool success = false, markerDetected = false;
    int fails = 0;

    vector<Point3f> realMarkerPoints = {
        {-movingMarker / 2, movingMarker / 2, 0},
        { movingMarker / 2, movingMarker / 2, 0},
        { movingMarker / 2, -movingMarker / 2, 0},
        {-movingMarker / 2, -movingMarker / 2, 0}
    };

    while (rodMain.empty() || !markerDetected || (!success && fails < 1)) {
        cap.read(frame);
        cvtColor(frame, grayFrame, COLOR_BGR2GRAY);
        detector.detectMarkers(frame, markerCorners, markerIds, rejectedCandidates);

        imagePoints.assign(4, Point2f());
        added.assign(4, false);
        markerDetected = false;

        if (!markerIds.empty()) {
            drawMarkers(frame, markerCorners, markerIds);

            for (size_t i = 0; i < markerIds.size(); i++) {
                int id = markerIds[i];

                if (id >= 20 && id <= 23) {
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

                    Mat cameraPosition = -R.t() * tvec;
                    cameraPosition.at<double>(2, 0) *= -1;

                    cout << "Camera Position:\nX: " << cameraPosition.at<double>(0, 0)
                         << " Y: " << cameraPosition.at<double>(1, 0)
                         << " Z: " << cameraPosition.at<double>(2, 0) << endl;
                }
            }

            fails++;
        }

        if (waitKey(35) >= 0) break;
    }

    Mat R5;
    Rodrigues(rvecMarker, R5);
    Mat markerWorldPos = rodMain.t() * (tvecMarker - tvecMain);
    markerWorldPos.at<double>(2, 0) *= -1;

    cout << "Marker " << movingMarkerId << " Position:\nX: " << markerWorldPos.at<double>(0, 0)
         << " Y: " << markerWorldPos.at<double>(1, 0)
         << " Z: " << markerWorldPos.at<double>(2, 0) << endl;

    return Point2f(markerWorldPos.at<double>(0, 0), markerWorldPos.at<double>(1, 0));
}

void Locator::start(int movingMarkerId) {
    Mat frame, grayFrame;
    vector<vector<Point2f>> markerCorners;
    vector<int> markerIds;
    vector<vector<Point2f>> rejectedCandidates;
    vector<Point2f> imagePoints(4);
    vector<bool> added(4, false);
    Mat rvec, tvec, rvecMarker, tvecMarker;

    vector<Point3f> realMarkerPoints = {
        {-movingMarker / 2, movingMarker / 2, 0},
        { movingMarker / 2, movingMarker / 2, 0},
        { movingMarker / 2, -movingMarker / 2, 0},
        {-movingMarker / 2, -movingMarker / 2, 0}
    };

    while (cap.read(frame)) {
        cvtColor(frame, grayFrame, COLOR_BGR2GRAY);
        detector.detectMarkers(frame, markerCorners, markerIds, rejectedCandidates);

        imagePoints.assign(4, Point2f());
        added.assign(4, false);

        bool markerDetected = false;

        if (!markerIds.empty()) {
            drawMarkers(frame, markerCorners, markerIds);

            for (size_t i = 0; i < markerIds.size(); i++) {
                int id = markerIds[i];

                if (id >= 20 && id <= 23) {
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
                bool success = solvePnP(realMPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
                if (success) {
                    Mat R;
                    Rodrigues(rvec, R);
                    Mat cameraPosition = -R.t() * tvec;
                    cameraPosition.at<double>(2, 0) *= -1;

                    cout << "Camera Position:\nX: " << cameraPosition.at<double>(0, 0)
                         << " Y: " << cameraPosition.at<double>(1, 0)
                         << " Z: " << cameraPosition.at<double>(2, 0) << endl;

                    if (markerDetected) {
                        Mat R5;
                        Rodrigues(rvecMarker, R5);
                        Mat markerWorldPos = R.t() * (tvecMarker - tvec);
                        markerWorldPos.at<double>(2, 0) *= -1;

                        cout << "Marker " << movingMarkerId << " Position:\nX: " << markerWorldPos.at<double>(0, 0)
                             << " Y: " << markerWorldPos.at<double>(1, 0)
                             << " Z: " << markerWorldPos.at<double>(2, 0) << endl;
                    }
                }
            }
        }

        imshow("Frame", frame);
        if (waitKey(35) >= 0) break;
    }
}
