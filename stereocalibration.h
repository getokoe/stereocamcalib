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

#ifndef STEREOCALIBRATION_H
#define STEREOCALIBRATION_H

#include <cnpy.h>
#include <string>

#include "cameracalibration.h"
#include "path.h"

namespace scc {

  class StereoCalibration {

  public:
      StereoCalibration();
      StereoCalibration(CameraCalibration *left, CameraCalibration *right);

      // Calibration process
      unsigned int checkForMatchingImages();
      double calibrate(unsigned int imageAmount);
      double recalibrate();
      bool checkUsedImagesForRotation(double minRotation);
      int checkForRotatedImage(double minRotation);
      void addImageFromUseableToUsed(unsigned int rotatedImagePos);
      bool saveData(std::string folderName);

      std::vector<std::string> getTargetImageStrings();
      unsigned int getUsableSourceImageAmount();
      std::string getRotationMatrixString();
      std::string getTranslationMatrixString();
      std::string getEssentialMatrixString();
      std::string getFundamentalMatrixString();
      std::string getQMatrixString();

  private:
      CameraCalibration *leftCam;
      CameraCalibration *rightCam;

      // Calibration vars
      std::vector<unsigned int> usableSourceImages;
      std::vector<unsigned int> usedSourceImages;
      std::vector<std::vector<cv::Point2f> > leftImagePointsAll;
      std::vector<std::vector<cv::Point2f> > rightImagePointsAll;
      std::vector<std::vector<cv::Point2f> > leftImagePoints;
      std::vector<std::vector<cv::Point2f> > rightImagePoints;
      std::vector<std::vector<cv::Point3f> > objectPoints;
      std::vector<std::string> targetImages;

      // rms is mean of left and right
      std::vector<float> rms;

      cv::Mat R;
      cv::Mat T;
      cv::Mat E;
      cv::Mat F;

      cv::Mat rLeft;
      cv::Mat rRight;
      cv::Mat pLeft;
      cv::Mat pRight;

      cv::Mat Q;

      cv::Mat newCamMatrixLeft;
      cv::Mat newCamMatrixRight;

      cv::Mat leftMapOne;
      cv::Mat leftMapTwo;
      cv::Mat rightMapOne;
      cv::Mat rightMapTwo;

      bool checkCams(CameraCalibration *left, CameraCalibration *right);
      void collectImagePointsFromCams();
      void collectRmsFromCams();
  //    void sortAllByRms();
      void initializeObjectPoints();
      void prepareRectification();
      bool posIsUsed(unsigned int posToCheck);

      std::string getStringFromMat(cv::Mat matToString);

      void saveTargetImagesUndistortedandRectified(Path p);
      void saveMatOfDoublesToNumpy(Path p, std::string fileName, cv::Mat matTosave);
      void saveMatOfDoublesToCsv(Path p, std::string fileName, std::string matName, cv::Mat matTosave);
      void saveTToNumpy(Path p, std::string fileName);
      void saveQToNumpy(Path p, std::string fileName);
  };
}
#endif // STEREOCALIBRATION_H
