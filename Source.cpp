/**
	opencvTriangulationV2.cpp
	Purpose: Demonstrates the use of triangulation to find the 3D locations of keypoints in a pair of stereo images
	Input: The input command line arguments are a series of filenames of image pairs. 
		   Each image pair consists of an image taken from each of the cameras of a scene containing a chessboard in it. 

	Usage: ./filename ../data/imgA1.jpg ../data/imgB1.jpg ../data/imgA2.jpg ../data/imgB2.jpg …
		   The filenames should point to images of each camera alternately.

	@version 25/07/2017
*/

#include "opencv2/opencv.hpp"
#include "opencv2/ccalib/omnidir.hpp"
#include "opencv2/aruco/charuco.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <iostream>

using namespace std;
using namespace cv;

vector<Point2f> clickedCorners;
Mat imgARemapClone;
void mouseCallback(int event, int x, int y, int flags, void* param) {
    switch (event) {
    case EVENT_MOUSEMOVE: {
        break;
    }
    case EVENT_LBUTTONUP: {
        break;
    }
    case EVENT_LBUTTONDOWN: {;
        Point2f point = ((float)x, (float)y);
        clickedCorners.push_back(point);
        if ((clickedCorners.size() % 20) == 0) {
            cout << "done entering points" << endl;
        }
    }
    }
}
float euclideanDist(Point2f& p, Point2f& q) {
    Point2f diff = p - q;
    return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}
struct mouseCallback2Params {
    vector<Point3f> points3D;
    vector<Point2f> points2D;
    int index;
};
void mouseCallback2(int event, int x, int y, int flags, void* param) {
    mouseCallback2Params* mp = (mouseCallback2Params*) param;
    switch (event) {
    case EVENT_MOUSEMOVE: {
        Point2f point = Point((float)x, (float)y);
        float minDist = 10000;
        for (int i = 0; i < (mp->points3D.size()); ++i) { 
            float dist = euclideanDist((mp->points2D[i]), point);
            if (dist < minDist) {
                minDist = dist;
                mp->index = i;
            }
        }
        break;
    }
    }
}

int main(int argc, char** argv) {
    //initialize and load parameters
    vector<vector<Point3f>> objectPts;
    vector<vector<Point2f>> imgPts1, imgPts2;
    Mat intrinsics1, intrinsics2, distCoeffs1, distCoeffs2, xi1, xi2;
    Size imageSize = Size(1280, 960);
    FileStorage file1("../data/180degreeCam3/180degreeCamera3Params1.xml", FileStorage::READ);
    //FileStorage file1("../data/180degreeCamera1Params8.xml", FileStorage::READ);
    file1["intrinsic"] >> intrinsics1;
    file1["intrinsic"] >> intrinsics2;
    file1["distCoeffs"] >> distCoeffs1;
    file1["distCoeffs"] >> distCoeffs2;
    file1["xi"] >> xi1;
    file1["xi"] >> xi2;
    if (!file1.isOpened() || intrinsics1.empty() || distCoeffs1.empty()) {
        cout << "Could not load parameters" << endl;
        return 1;
    }
    file1.release();

    //chessboard description for calibration
    int numCornersHor = 5;
    int numCornersVer = 4;
    int numSquares = numCornersHor * numCornersVer;
    Size boardSz = Size(numCornersHor, numCornersVer);
    vector<Point3f> singleObjectPts;

    //build list of objects points for the chessboard in the image (0,0,0  0,0,1  0,0,2 etc.)
    for (int j = 0; j < numSquares; ++j) {
        singleObjectPts.push_back(Point3f(j%numCornersHor, (float)(0.92857*(j / numCornersHor)), 0.0f));
    }

    //build up list of image points for the 2 boards
    int count = 1;
    Mat chessBoard1, chessBoard2;
    namedWindow("board", WINDOW_AUTOSIZE);
    setMouseCallback("board", mouseCallback, NULL); //allows user to click on the chessboard corners manuallly if auto detection fails
    while (count < (argc - 3)) {
        count += 2;
        chessBoard1 = imread(argv[count]);
        chessBoard2 = imread(argv[count + 1]);
        if (!(chessBoard1.data || chessBoard2.data)) {
            cout << "could not read images" << endl;
            break;
        }

        imageSize = chessBoard1.size();
        Mat mapXUndistort, mapYUndistort;
        Matx33f reprojectionIntrinsicMat = Matx33f(imageSize.width / 8, 0, imageSize.width / 2,
            0, imageSize.width / 8, imageSize.height / 2,
            0, 0, 1);
        //create undistortion mapX and mapY
        omnidir::initUndistortRectifyMap(intrinsics1, distCoeffs1, xi1, Mat(), reprojectionIntrinsicMat, imageSize, CV_32FC1, mapXUndistort, mapYUndistort, omnidir::RECTIFY_PERSPECTIVE);
        //undistort image
        remap(chessBoard1, chessBoard1, mapXUndistort, mapYUndistort, INTER_CUBIC, BORDER_CONSTANT, Scalar::all(0));
        remap(chessBoard2, chessBoard2, mapXUndistort, mapYUndistort, INTER_CUBIC, BORDER_CONSTANT, Scalar::all(0));

        imageSize = chessBoard1.size();
        vector<Point2f> corners1;
        vector<Point2f> corners2;
        Mat chessBoard1Gray, chessBoard2Gray;
        cvtColor(chessBoard1, chessBoard1Gray, CV_BGR2GRAY);
        cvtColor(chessBoard2, chessBoard2Gray, CV_BGR2GRAY);
        bool found1 = findChessboardCorners(chessBoard1Gray, boardSz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH);
        bool found2 = findChessboardCorners(chessBoard2Gray, boardSz, corners2, CV_CALIB_CB_ADAPTIVE_THRESH);
        if (found1) {
            cornerSubPix(chessBoard1Gray, corners1, boardSz, Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1)); //refine detected corners
        }
        /*
        else {
            imshow("board", chessBoard1);
            if (waitKey(0) == 27) {
                clickedCorners.clear();
                break;
            }
            corners1 = clickedCorners; //add manually selected corners to vector of corners coordinates
            clickedCorners.clear();
        }
        */
        if (found2) {
            cornerSubPix(chessBoard2Gray, corners2, boardSz, Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        }
        /*
        else {
            imshow("board", chessBoard2);
            if (waitKey(0) == 27) {
                clickedCorners.clear();
                break;
            }
            corners2 = clickedCorners; //add manually selected corners to vector of corners coordinates
            clickedCorners.clear();
        }
        */
        if ((corners1.size() == numSquares) && (corners2.size() == numSquares)) {
            imgPts1.push_back(corners1); //add corners to vector of vector of corner points
            imgPts2.push_back(corners2);
            objectPts.push_back(singleObjectPts);
            printf("points stored\n");
        }
    }

    //calibrate cameras and determine rotation and translation vector between cameras
    Mat Rvec, Tvec, E, F;
    //new camera matrix for reprojection of image
    imageSize = Size(1280, 960);
    Mat Knew = (Mat_<double>(3, 3) << imageSize.width / 8, 0, imageSize.width / 2,
        0, imageSize.height / 8, imageSize.height / 2,
        0, 0, 1);
    Mat newDistCoeffs1, newDistCoeffs2;
    stereoCalibrate(objectPts, imgPts1, imgPts2, Knew, newDistCoeffs1, Knew, newDistCoeffs2, imageSize, Rvec, Tvec, E, F, CALIB_FIX_INTRINSIC + CALIB_ZERO_TANGENT_DIST + CALIB_FIX_K1);

    //omnidir::stereoCalibrate(objectPts, imgPts1, imgPts2, imageSize, imageSize, intrinsics1, xi1, distCoeffs1, intrinsics2, xi2, distCoeffs2, Rvec, Tvec, Rvecl, Tvecl, 0, criteria, idx);	
    cout << "Knew: " << Knew << endl;
    cout << "distCoeffs1 " << newDistCoeffs1 << endl;
    cout << "Rvec: " << Rvec << endl;
    cout << "Tvec: " << Tvec << endl;
    cout << "E: " << E << endl;
    cout << "F: " << F << endl;

    //Projection matrix P1 for cam A is intrinsics*[I|0]
    Mat P1 = (Mat_<double>(3, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
    P1 = Knew*P1;
    //Projection matrix P2 for cam B = intrinsics*[R|T]
    Mat P2(3, 4, CV_64F, Scalar::all(0));
    Rvec.copyTo(P2(Rect(0, 0, 3, 3)));
    P2.at<double>(0, 3) = Tvec.at<double>(0, 0);
    P2.at<double>(1, 3) = Tvec.at<double>(1, 0);
    P2.at<double>(2, 3) = Tvec.at<double>(2, 0);
    P2 = Knew*P2;

    Mat essentialMat = E;
    Mat fundamentalMat = F;

    //undistort the 2 test images
    Mat map1X, map1Y, map2X, map2Y;
    omnidir::initUndistortRectifyMap(intrinsics1, distCoeffs1, xi1, noArray(), Knew, imageSize, CV_16SC2, map1X, map1Y, omnidir::RECTIFY_PERSPECTIVE);
    omnidir::initUndistortRectifyMap(intrinsics2, distCoeffs2, xi2, noArray(), Knew, imageSize, CV_16SC2, map2X, map2Y, omnidir::RECTIFY_PERSPECTIVE);
    Mat imgARemap, imgBRemap;
    //Mat imgATest = imread("../data/stereoCameras4/capture35.jpg");
    //Mat imgBTest = imread("../data/stereoCameras4/capture36.jpg");
    Mat imgATest = imread("../data/180degreeCam3/stereoCameras6/capture33.bmp");
    Mat imgBTest = imread("../data/180degreeCam3/stereoCameras6/capture34.bmp");
    remap(imgATest, imgARemap, map1X, map1Y, INTER_LINEAR);
    remap(imgBTest, imgBRemap, map2X, map2Y, INTER_LINEAR);

    //find corresponding points in both images
    vector<KeyPoint> keypointsA, keypointsB;
    Mat descriptorsA, descriptorsB;
    vector<DMatch> matches;
    Ptr<xfeatures2d::SURF> SURFDetector = xfeatures2d::SURF::create(300); //initialize surf detector
    SURFDetector->detectAndCompute(imgARemap, noArray(), keypointsA, descriptorsA, false); //find keypoints on image and compute descriptor vectors for each point
    SURFDetector->detectAndCompute(imgBRemap, noArray(), keypointsB, descriptorsB, false);
    FlannBasedMatcher matcher(new cv::flann::KDTreeIndexParams(4), new cv::flann::SearchParams());
    matcher.match(descriptorsB, descriptorsA, matches, noArray()); //match the keypoints in both images based on descriptor values
    Mat resultImg, resultImgA, resultImgB;
    //drawKeypoints(imgARemap, keypointsA, resultImgA);
    drawMatches(imgBRemap, keypointsB, imgARemap, keypointsA, matches, resultImg, Scalar(0, 255, 0), Scalar(0, 0, 255));
    imshow("result", resultImg);
    waitKey(0);
    //imwrite("../data/stereoCameras5/matches.jpg", resultImg);
    cout << "Initial SURF Matches size: " << matches.size() << endl;

    vector<Point2f> imgPtsA, imgPtsB;
    vector<Point2f> rejectedA, rejectedB;
    for (int i = 0; i < matches.size(); ++i) {
        cout << matches[i].distance << endl;
        if (matches[i].distance < 0.4) {  //add keypoints to vector of image points if they match well with each other
            imgPtsB.push_back(keypointsB[i].pt);
            imgPtsA.push_back(keypointsA[matches[i].trainIdx].pt);
        }
        else {
            rejectedA.push_back(keypointsA[matches[i].trainIdx].pt);
            rejectedB.push_back(keypointsB[i].pt);
        }
    }
    if (imgPtsA.size() != imgPtsB.size()) {
        cout << "cannot find correct matches" << endl;
        return 1;
    }
    if (imgPtsA.size() < 1) {
        cout << "Not enough matches found" << endl;
        return 1;
    }

    //compute the epipolar lines of points on one image on the other, based on the rotation and translation between them. Lines are in the format ax + by + c = 0
    vector<Vec3f> epilines;
    computeCorrespondEpilines(imgPtsA, 1, fundamentalMat, epilines);
    cout << "Pre Epilines/ Post SURF distance trim size: " << epilines.size() << endl;
    if (epilines.size() == imgPtsA.size()) {
        for (int i = 0; i < epilines.size(); ++i) {
            float M = -epilines[i][0] / epilines[i][1]; //calculate gradient of line, M, in y = Mx + C
            float C = -1.0f * (epilines[i][2] / epilines[i][1]); //calculate y-intercept, C, in equation of line
            float MPerpendicular = -1.0f / M; //calculate gradient of perpendicular to line
            float CPerpendicular = imgPtsB[i].y - (MPerpendicular*imgPtsB[i].x); //calculate y-intercept of perpendicular line (the line from imgPtB to the epipolar line)
            float xIntersect = (CPerpendicular - C) / (M - MPerpendicular); //calculate intersection point of the perpendicular and epipolar line
            float yIntersect = (M*xIntersect) + C;
            float distance = euclideanDist(imgPtsB[i], Point2f(xIntersect, yIntersect)); //get distance from imgPtB to the epipolar line
            if (distance > 5.0f) { //remove any cases where imgPtB is wildly outside the epipolar line
                cout << "deleting" << endl;
                rejectedA.push_back(imgPtsA[i]);
                rejectedB.push_back(imgPtsB[i]);
                imgPtsA.erase(imgPtsA.begin() + i);
                imgPtsB.erase(imgPtsB.begin() + i);
                epilines.erase(epilines.begin() + i);
                i -= 1;
            }
            //line(imgBRemap, Point(0, -1 * (epilines[i][2] / epilines[i][1])), Point(imgBRemap.cols - 1, (-(epilines[i][0] * imgBRemap.cols) - epilines[i][2]) / epilines[i][1]), Scalar(255, 255, 255));
            //imshow("epilines", imgBRemap);
            //waitKey(0);
        }
    }
    cout << "Post epiline trim size: " << imgPtsA.size() << endl;

    //use the RANSAC algorithm to indirectly remove any further outliers by calculating the essential matrix again from the detected image points
    vector<uchar> outliers;
    Mat essentialMat2 = findEssentialMat(imgPtsA, imgPtsB, Knew, RANSAC, 0.99, 3.0, outliers);
    for (int i = 0; i < outliers.size(); ++i) {
        if ((int)outliers[i] == 0) {
            rejectedA.push_back(imgPtsA[i]);
            rejectedB.push_back(imgPtsB[i]);
            imgPtsA.erase(imgPtsA.begin() + i);
            imgPtsB.erase(imgPtsB.begin() + i);
            outliers.erase(outliers.begin() + i);
            i -= 1;
        }
    }
    cout << "Post RANSAC outliers trim size: " << imgPtsA.size() << endl;

    //triangulate the corresponding points on both images to calculate their 3D location relative to the 1st camera
    Mat outputPoints;
    triangulatePoints(P1, P2, imgPtsA, imgPtsB, outputPoints); //get 3D coordinates in the form (x,y,z,w), where w is a common factor
    vector<Point3f> points3D;
    for (int i = 0; i < (outputPoints.cols); ++i) {
        float w = outputPoints.at<float>(3, i);
        Point3f point3D;
        point3D.x = (outputPoints.at<float>(0, i) / w);
        point3D.y = (outputPoints.at<float>(1, i) / w);
        point3D.z = (outputPoints.at<float>(2, i) / w);
        points3D.push_back(point3D);
    }

    //Additional: Rotate the axes (and 3D coordinates) to upright position regardless of angle of camera, based on an image of chessboard flat on the ground
    /*
    Mat chessboardFlat = imread("../data/stereoCameras4/capture37.jpg");
    vector<Point2f> chessboardFlatImgPts;
    Mat chessboardFlatGray;
    cvtColor(chessboardFlat, chessboardFlatGray, CV_BGR2GRAY);
    bool found = findChessboardCorners(chessboardFlatGray, boardSz, chessboardFlatImgPts, CV_CALIB_CB_ADAPTIVE_THRESH);
    if (found) {
    cornerSubPix(chessboardFlatGray, chessboardFlatImgPts, boardSz, Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1)); //refine detected corners
    Mat chessboardFlatRvec, chesssboardFlatTvec;
    solvePnP(singleObjectPts, chessboardFlatImgPts, intrinsics1, distCoeffs1, chessboardFlatRvec, chesssboardFlatTvec); //calculate rotation vector from earlier chessboard
    Mat solvePNPRvec;
    Rodrigues(chessboardFlatRvec, solvePNPRvec);
    Mat solvePNPRvecInv = solvePNPRvec.inv();
    Mat rotateAboutXAxis = (Mat_<double>(3, 3) << 1, 0, 0, 0, 0, 1, 0, -1, 0);
    Mat RvecTransform = rotateAboutXAxis*solvePNPRvecInv;
    for (int i = 0; i < points3D.size(); ++i) {
    Mat point3D = (Mat_<double>(3, 1) << points3D[i].x, points3D[i].y, points3D[i].z);
    point3D = RvecTransform*point3D;
    points3D[i].x = (float)point3D.at<double>(0, 0);
    points3D[i].y = (float)point3D.at<double>(1, 0);
    points3D[i].z = (float)point3D.at<double>(2, 0);
    }
    }
    */

    for (int i = 0; i < imgPtsA.size(); ++i) {
        circle(imgARemap, Point(imgPtsA[i].x, imgPtsA[i].y), 2, Scalar(0, 0, 255));
    }

    mouseCallback2Params callback2Params;
    callback2Params.points3D = points3D;
    callback2Params.points2D = imgPtsA;
    callback2Params.index = 0;
    
    namedWindow("undistorted", WINDOW_AUTOSIZE);
    setMouseCallback("undistorted", mouseCallback2, &callback2Params);
    
    for (;;) {
        imgARemap.copyTo(imgARemapClone);
        ostringstream coordinatesStream;
        coordinatesStream << setprecision(1) << fixed << callback2Params.points3D[callback2Params.index].x << " " << callback2Params.points3D[callback2Params.index].y << " " << callback2Params.points3D[callback2Params.index].z;
        string coordinates = coordinatesStream.str();
        circle(imgARemapClone, Point(callback2Params.points2D[callback2Params.index].x, callback2Params.points2D[callback2Params.index].y), 2, Scalar(0, 0, 255));
        putText(imgARemapClone, coordinates, Point(callback2Params.points2D[callback2Params.index].x, callback2Params.points2D[callback2Params.index].y), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255, 255, 0), 2);
        imshow("undistorted", imgARemapClone);
        if (waitKey(15) == 27)
            break;
    }
    
    return 0;
}
