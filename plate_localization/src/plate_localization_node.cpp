#include "plate_localization.h"


void SubPuber::PlateRecognitionCallback(const sensor_msgs::ImageConstPtr &cameraImage)
{
  Mat img = cv_bridge::toCvShare(cameraImage, "bgr8")->image;

  std::vector<Mat> channels;
  split(img, channels);
  Mat grayImage = Mat(img.rows, img.cols, CV_8UC1);
  grayImage = channels.at(0) - 0.8 * channels.at(1) - 0.8 * channels.at(2);

  threshold(grayImage, grayImage, 70, 255, THRESH_BINARY);
  Mat element = getStructuringElement(MORPH_RECT,
                                      Size(25, 25),
                                      Point(-1, -1));

  dilate(grayImage, grayImage, element);
  erode(grayImage, grayImage, element);
  erode(grayImage, grayImage, element);
  dilate(grayImage, grayImage, element);

	Mat cornerImg = Mat::zeros(img.size(), CV_32FC1);
	cornerHarris(grayImage, cornerImg, 2, 3, 0.04, BORDER_DEFAULT);
  Mat normImage, scaledImage;
	normalize(cornerImg, normImage, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
	convertScaleAbs(normImage, scaledImage);
  
  vector<Point2f> imagePoints;
  int maxIdxX, maxIdxY, maxValue;
  for (int i = 0; i < normImage.rows; i++) {
    for (int j = 0; j < normImage.cols; j++) {
      maxValue = 120;
      if ((int)normImage.at<float>(i, j) > maxValue) {
        for (int iIt = -20; iIt <= 20; iIt++) {
          for (int jIt = -30; jIt <= 30; jIt++) {
            if (i + iIt < 0 || i + iIt >= normImage.rows || j + jIt < 0 || j + jIt >= normImage.cols) {
              continue;
            }
            if ((int)normImage.at<float>(i + iIt, j + jIt) > maxValue) {
              maxValue = (int)normImage.at<float>(i + iIt, j + jIt);
              maxIdxY = i + iIt;
              maxIdxX = j + jIt;
            }
            normImage.at<float>(i + iIt, j + jIt) = 0;
          }
        }
        imagePoints.push_back({(float) maxIdxX, (float) maxIdxY});
        // ROS_INFO("Put one point %d %d", maxIdxX, maxIdxY);
      }
    }
  }
  // ROS_INFO("Frame finished");

  // Mat contourImage;
  // grayImage.copyTo(contourImage);

  // // 轮廓发现与绘制
  // vector<vector<Point>> contours;
  // vector<Vec4i> hierarchy;
  // findContours(contourImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point());

  // int contourIndex = 0;
  // for(int i = 1; i < contours.size(); i++) {
  //   if(contourArea(contours[i]) > contourArea(contours[contourIndex])) {
  //     contourIndex = i;
  //   }
      
  //   //绘制轮廓
  //   drawContours(contourImage,contours,i,Scalar(255),1,8,hierarchy);
  // }

  // RotatedRect rRect = minAreaRect(contours[contourIndex]);

  // Point2f vertices[4];
  // rRect.points(vertices);
  // for (int i = 0; i < 4; i++) {
  //     line(contourImage, vertices[i], vertices[(i+1)%4], Scalar(255), 2);
  // }
  // vector<Point2f> imagePoints;
  // for(int i = 0; i < 4; i++) {
  //   imagePoints.push_back(vertices[i]);
  // }



  // ROS_INFO("%d", imagePoints.size());
  if (imagePoints.size() == 4) {


    for (int i = 0; i < imagePoints.size(); i++) {
        circle(img, Point(imagePoints[i].x, imagePoints[i].y), 5, Scalar(10, 10, 255), 2, 8, 0);
    }
    const Mat cameraMatrix = (Mat_<double>(3, 3) << 421.93280029296875, 0.0, 422.89239501953125,
                                                    0.0, 421.93280029296875, 235.1717071533203,
                                                    0.0, 0.0, 1.0);
    const Mat distCoeffs = (Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
    vector<Point3f> objectPoints;
    objectPoints.push_back({-0.220, 0.070, 0});
    objectPoints.push_back({0.220, 0.070, 0});
    objectPoints.push_back({0.220, -0.70, 0});
    objectPoints.push_back({-0.220, -0.70, 0});

    Mat rvec, tvec;
    solvePnP(
      objectPoints, // object points, Nx3 1-channel or 1xN/Nx1 3-channel, N is the number of points. vector<Point3d> can be also passed
      imagePoints,  // corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel, N is the number of points. vector<Point2d> can be also passed
      cameraMatrix, // camera intrinsic matrix
      distCoeffs,   // distortion coefficients
      rvec,         // rotation vector
      tvec          // translation vector
    );
    // Mat rotM;
    // Rodrigues(rvec,rotM);
    // drawFrameAxes(img, cameraMatrix, distCoeffs, rvec, tvec, 0.3);
    vector<Point2f> projectedPoints;
    // projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);
    for(unsigned int i = 0; i < projectedPoints.size(); ++i)
    {
      circle(img, projectedPoints[i], 3, Scalar(0,255,0), -1, 8);
    }

    cout << tvec << endl;
    geometry_msgs::PoseStamped platePose;
    platePose.pose.position.x = tvec.at<float>(0);
    platePose.pose.position.y = tvec.at<float>(1);
    platePose.pose.position.z = tvec.at<float>(2);

    platePosePub.publish(platePose);


  // }
  // else if(imagePoints.size() > 4){
  //     ROS_INFO("more");
  // }
  // else if(imagePoints.size() < 4){
  //     ROS_INFO("less");
  }

  // cvtColor(grayImage, grayImage, CV_GRAY2RGB);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  recognizedImagePub.publish(*msg);
}