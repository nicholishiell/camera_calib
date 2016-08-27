#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include <raspicam/raspicam_cv.h>
#include <raspicam/raspicam.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fstream>

using namespace std;
using namespace cv;


int main(int argc, char **argv){
  float centerX = 0.;
  float centerY = 0.;

  ros::init(argc, argv, "camera_calib");
  ros::NodeHandle n;
   
  raspicam::RaspiCam_Cv camera;
  cv::Mat rawImage;
  cv::Mat calibImage;
  cv::Mat keyPointsImage;
  
  SimpleBlobDetector::Params paramsCalib;
  paramsCalib.minThreshold = 200;
  paramsCalib.maxThreshold = 255;
  paramsCalib.minDistBetweenBlobs = 5.0f;
  paramsCalib.filterByInertia = false;
  paramsCalib.filterByConvexity = false;
  paramsCalib.minConvexity = 0.75;
  paramsCalib.maxConvexity = 1.0;
  paramsCalib.filterByColor = false;
  paramsCalib.filterByCircularity = false;
  paramsCalib.minCircularity = 0.01;
  paramsCalib.maxCircularity = 1.;
  paramsCalib.filterByArea = true;
  paramsCalib.minArea = 5000.0f;
  paramsCalib.maxArea = 1000000.0f;
  SimpleBlobDetector detectorCalib(paramsCalib);
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Used to find center of ROI and by extension forward direction
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  camera.set( CV_CAP_PROP_FORMAT, CV_8UC3 ); // Set image format
  camera.set( CV_CAP_PROP_EXPOSURE, 1); // Set shutter speed (1.3 for calib) 
  if ( !camera.open()) printf("Error opening camera\n");
    
  camera.grab();
  camera.retrieve(rawImage);
  
  // Create binary mask of dark areas and use it to calibrate.
  inRange(rawImage, cv::Scalar(0,0,0), cv::Scalar(10,20,40), calibImage);

  std::vector<KeyPoint> calibKeyPoints;
  detectorCalib.detect( calibImage, calibKeyPoints);
      
  drawKeypoints( calibImage, calibKeyPoints, keyPointsImage, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
      
  cv::imwrite("calib.jpg", keyPointsImage);
  cv::imwrite("raw.jpg", rawImage);
  //for(int i = 0; i < calibKeyPoints.size(); i++)
  //printf("Center: (%f, %f) \t Size:%f\n", calibKeyPoints[i].pt.x, calibKeyPoints[i].pt.y, calibKeyPoints[i].size);
  centerX = calibKeyPoints[0].pt.x;
  centerY = calibKeyPoints[0].pt.y;

  camera.release();

  ros::spinOnce();

  fstream outputFile;
  outputFile.open("/home/pi/ns_catkin_ws/calibData/center_of_roi", std::fstream::out);
  outputFile << centerX <<"\t"<< centerY<<"\n";
  outputFile.close();
  
  return 0;
}
