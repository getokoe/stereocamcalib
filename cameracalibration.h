/*******************************************************************************
 *
 *   Copyright (C) 2016 by Tom Koeppchen
 *   gtkoeppchen (at) googlemail (dot) com
 *   This file is part of stereocamcalib.
 *
 *   Stereocamcalib is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   Stereocamcalib is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with stereocamcalib.  If not, see <http://www.gnu.org/licenses/>.
 *
 ******************************************************************************/

#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H

#include <string>
#include <vector>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>

#include "path.h"
#include "cnpy.h"

namespace scc {

  class CameraCalibration {

  public:
      // constructor
      CameraCalibration();
      CameraCalibration(int size, int patternH, int patternV,
                        std::vector<std::string> files, std::string desc);

      // Calibration process
      int processNextImage();
      double calibrate();
      int removeHighRms(unsigned int minAmount, double percentageToRemove);
      unsigned int removeHighRmsForAllImg(float maxRms, unsigned int minAmountLeft);
      void computeRotAndTransForAllImg();
      void computeReprojectionErrorForAllImg();
      bool checkUsedImagesForRotation(double minDegree);
      int checkForRotatedImage(double minDegree);
      int checkForDistributedImage();
      void addImageFromUseableToUsed(unsigned int imageToAdd);
      bool computeImagePointDistribution();

      // get data
      std::string getDescription();
      cv::Mat loadImage(unsigned int filePosition);
      std::vector<std::string> getUsedSourceImages();
      std::vector<std::string> getSourceImages();
      std::vector<std::string> getTargetImages();
      std::vector<unsigned int> getUsableSourceImagePos();
      unsigned int getUsableSourceImageAmount();
      std::vector<unsigned int> getUsedSourceImagePos();
      std::vector<std::vector<cv::Point3f> > getObjectPoints();
      std::vector<std::vector<cv::Point2f> > getImagePoints();
      std::vector<cv::Point2f> getImagePointsForPos(unsigned int position);
      float getRmsForPos(unsigned int position);
      double getMaxRotationForPos(unsigned int position);
      cv::Mat getImagePointsDistributionImg();
      cv::Size getPatternDim();
      int getPatternSize();
      cv::Size getImageSize();
      unsigned int getSourceImagesNo();
      cv::Mat getCameraMatrix();
      cv::Mat getDistortionCoeffcients();
      std::string getImagePointsDistributionImagePath();
      bool saveData(std::string folderName);
      std::string getCameraMatrixString();
      std::string getDistortionMatrixString();

      // Status information
      std::string getStatus();
      std::string print();

  private:
      std::string description;

      // Pre calibration vars
      int patternSize;
      cv::Size patternDim;
      cv::Size imageSize;
      std::vector<std::string> sourceImages;
      std::vector<std::string> targetImages;
      cv::Mat lookUpTable;

      // Calibration vars
      int sourceImagePosition;
      std::vector<unsigned int> usableSourceImagePos;
      std::vector<unsigned int> usedSourceImagePos;
      std::vector<std::vector<cv::Point2f> > imagePoints;
      std::vector<std::vector<cv::Point2f> > allImagePoints;
      std::vector<std::vector<cv::Point3f> > objectPoints;

      // Post calibration vars
      std::string imagePointsDistributionImagePath;
      cv::Mat cameraMatrix;
      cv::Mat distortionCoeffcients;
      std::vector<cv::Mat> rotationVectors;
      std::vector<cv::Mat> allRotationVectors;
      std::vector<cv::Mat> translationVectors;
      std::vector<cv::Mat> allTranslationVectors;
      std::vector<float> perViewErrors;
      std::vector<float> allPerViewErrors;
      std::vector<cv::Vec3d> allAngles;
      std::vector<unsigned int> imagePointDistribution;
      std::vector<std::vector<unsigned int> > imagePointDistributionPerView;

      // Set pre calibration var
      void initializeImageSize();
      void computeLookUpTable();
      void initializeCalibrationParameter(std::vector<std::vector<cv::Point2f> > imgPoints);
      void initializeObjectPoints(std::vector<std::vector<cv::Point2f> > imgPoints);

      // find positions
      unsigned int findPositionInUsableSourceImgPosByPos(unsigned int usedSourceImagePosToFind);
      bool isPosUsed(unsigned int posToCheck);

      // Calibration
      cv::Mat loadImageGray(unsigned int filePosition);
      std::vector<cv::Point2f> findPatternCorners(cv::Mat image);
      std::vector<cv::Point2f> refineCorners(cv::Mat image,
                                            std::vector<cv::Point2f> corners);

      std::vector<float> computeReprojectionErrors(std::vector<std::vector<cv::Point2f> > imgPoints,
                                                  std::vector<cv::Mat> rotVectors,
                                                  std::vector<cv::Mat> transVectors);

      // sort
      void sortByRmsForUsedImg();
      void sortByRmsForAllImg();

      // saving
      void saveTargetImagesUndistorted(Path p);
      void saveImagePointsDistribution(Path p);
      void saveCameraMatrixToNumpy(Path p);
      void saveCameraMatrixToCsv(Path p);
      void saveDistCoeffToNumpy(Path p);
      void saveDistCoeffToCsv(Path p);
      std::string printNumber(int number);
      std::string printVectorOfStrings(std::vector<std::string> vec);

  };
}
#endif // CAMERACALIBRATION_H
