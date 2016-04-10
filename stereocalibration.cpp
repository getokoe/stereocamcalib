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

#include "stereocalibration.h"

namespace scc {

  /**
  * Default constructor.
  */
  StereoCalibration::StereoCalibration() {

  }

  /**
  * Constructor. Checks both cameras to have identical settings.
  * @param left
  * @param right
  */
  StereoCalibration::StereoCalibration(CameraCalibration *left,
                                      CameraCalibration *right) {
      if (checkCams(left, right)) {
          leftCam = left;
          rightCam = right;
      }
  }

  /**
  * Runs a stereo calibration with the given amount of pictures.
  * @param imageAmount The amount of pictures to use for the calibration.
  * @return The RMS retrieved by the 'opencv' funtion 'stereoCalibrate'.
  */
  double StereoCalibration::calibrate(unsigned int imageAmount) {
      unsigned int i = 0;
      while (i < imageAmount && i < usableSourceImages.size()) {
          usedSourceImages.push_back(usableSourceImages[i]);
          leftImagePoints.push_back(leftImagePointsAll[i]);
          rightImagePoints.push_back(rightImagePointsAll[i]);
          i++;
      }
      initializeObjectPoints();
      double retVal = 0;
      retVal = cv::stereoCalibrate(objectPoints,
                                  leftImagePoints,rightImagePoints,
                                  leftCam->getCameraMatrix(),
                                  leftCam->getDistortionCoeffcients(),
                                  rightCam->getCameraMatrix(),
                                  rightCam->getDistortionCoeffcients(),
                                  leftCam->getImageSize(),
                                  R, T, E, F,
                                  cv::TermCriteria(cv::TermCriteria::COUNT+
                                                    cv::TermCriteria::EPS,
                                                    30, 1e-6),
                                  CV_CALIB_FIX_INTRINSIC); // because intrinsic
                                                            // values are provided
      return retVal;
  }

  /**
  * Runs a stereo calibration.
  * @return The RMS retrieved by the 'opencv' funtion 'stereoCalibrate'.
  */
  double StereoCalibration::recalibrate() {
      initializeObjectPoints();
      double retVal = 0;
      retVal = cv::stereoCalibrate(objectPoints,
                                  leftImagePoints,rightImagePoints,
                                  leftCam->getCameraMatrix(),
                                  leftCam->getDistortionCoeffcients(),
                                  rightCam->getCameraMatrix(),
                                  rightCam->getDistortionCoeffcients(),
                                  leftCam->getImageSize(),
                                  R, T, E, F,
                                  cv::TermCriteria(cv::TermCriteria::COUNT+
                                                    cv::TermCriteria::EPS,
                                                    30, 1e-6),
                                  CV_CALIB_FIX_INTRINSIC); // because intrinsic
                                                            // values are provided
      return retVal;
  }

  /**
  * Checks the images used for calibration to have the given minimum of rotation.
  * @param minRotation The minimum rotation to check for.
  * @return True if an image with desired rotation was found otherwise false.
  */
  bool StereoCalibration::checkUsedImagesForRotation(double minRotation) {
      unsigned int counter = 0;
      bool higher = false;
      while (counter < usedSourceImages.size() && higher == false) {
          if (leftCam->getMaxRotationForPos(usedSourceImages[counter]) > minRotation) {
              higher = true;
          }
          counter++;
      }
      return higher;
  }

  /**
  * Checks the available images for the given minimum of rotation. If an image is
  * found and not already used for calibration its position is return.
  * @param minRotation The minimum rotation to check for.
  * @return -1 if no image was found otherwise the position of the found image.
  */
  int StereoCalibration::checkForRotatedImage(double minRotation) {
      unsigned int counter = 0;
      int pos = -1;
      while (counter < usableSourceImages.size() && pos != -1) {
          if (leftCam->getMaxRotationForPos(usableSourceImages[counter]) > minRotation) {
              if (!posIsUsed(usableSourceImages[counter])) {
                  pos = usableSourceImages[counter];
              }
          }
          counter++;
      }
      return pos;
  }

  /**
  * Checks wether the image described by the given position is already used for
  * calibration or not.
  * @param posToCheck Position of the image to check for.
  * @return True if the image is already used for calibration, otherwise false.
  */
  bool StereoCalibration::posIsUsed(unsigned int posToCheck) {
      bool isUsed = false;
      unsigned int counter = 0;
      while (counter < usedSourceImages.size() && isUsed == false) {
          if (usedSourceImages[counter] == posToCheck) {
              isUsed = true;
          }
          counter++;
      }
      return isUsed;
  }

  /**
  * Adds an image position and the related image points to the lists used for
  * calibration.
  * @param rotatedImagePos The position describing the image to add.
  */
  void StereoCalibration::addImageFromUseableToUsed(unsigned int rotatedImagePos) {
      usedSourceImages.push_back(rotatedImagePos);
      leftImagePoints.push_back(leftCam->getImagePointsForPos(rotatedImagePos));
      rightImagePoints.push_back(rightCam->getImagePointsForPos(rotatedImagePos));
  }

  /**
  * Returns the strings describing the path of the used images for calibration.
  * @return A Vector containing the pathes to the images on the system.
  */
  std::vector<std::string> StereoCalibration::getTargetImageStrings() {
      return targetImages;
  }

  /**
  * Returns the number of useable images for calibration.
  * @return The number of images useable for calibration.
  */
  unsigned int StereoCalibration::getUsableSourceImageAmount() {
      return usableSourceImages.size();
  }

  /**
  * Checks the given 'CameraCalibration' objects to have the same core values.
  * @param left One 'CameraCalibration' object to check
  * @param right Another 'CameraCalibration' object to check
  * @return True if the core values are the same, otherwise false.
  */
  bool StereoCalibration::checkCams(CameraCalibration *left,
                                    CameraCalibration *right) {
      bool retVal = true;
      if (left->getSourceImagesNo() != right->getSourceImagesNo()) {
          retVal = false;
      }
      if (left->getPatternDim() != right->getPatternDim()) {
          retVal = false;
      }
      if (left->getPatternSize() != right->getPatternSize()) {
          retVal = false;
      }
      if (left->getImageSize() != right->getImageSize()) {
          retVal = false;
      }
      return retVal;
  }

  /**
  * Checks the cameras for usable stereo image pairs.
  * @return The number of found pairs.
  */
  unsigned int StereoCalibration::checkForMatchingImages() {
      std::vector<unsigned int> left = leftCam->getUsableSourceImagePos();
      std::vector<unsigned int> right = rightCam->getUsableSourceImagePos();
      for (unsigned int i = 0; i < left.size(); i++) {
          unsigned int j = 0;
          bool found = false;
          while (j < right.size() && found == false) {
              if (left.at(i) == right.at(j)) {
                  found = true;
                  usableSourceImages.push_back(left.at(i));
              }
              j++;
          }
      }
      collectImagePointsFromCams();
      collectRmsFromCams();
      return usableSourceImages.size();
  }

  /**
  * Returns a string describing the rotation matrix.
  * @return The string.
  */
  std::string StereoCalibration::getRotationMatrixString() {
      return getStringFromMat(R);
  }

  /**
  * Returns a string describing the translation matrix.
  * @return The string.
  */
  std::string StereoCalibration::getTranslationMatrixString() {
      return getStringFromMat(T);
  }

  /**
  * Returns a string describing the essential matrix.
  * @return The string.
  */
  std::string StereoCalibration::getEssentialMatrixString() {
      return getStringFromMat(E);
  }

  /**
  * Returns a string describing the fundamental matrix.
  * @return The string.
  */
  std::string StereoCalibration::getFundamentalMatrixString() {
      return getStringFromMat(F);
  }

  /**
  * Returns a string describing the q matrix.
  * @return the string.
  */
  std::string StereoCalibration::getQMatrixString() {
      return getStringFromMat(Q);
  }

  /**
  * Builds a string describing the given 'cv::Mat' object.
  * @param matToString The 'cv::Mat' object to describe.
  * @return The string.
  */
  std::string StereoCalibration::getStringFromMat(cv::Mat matToString) {
      std::stringstream ss;
      ss << std::setprecision(std::numeric_limits<double>::digits10 + 2);

      for (int k = 0; k < matToString.rows; k++) {
          for (int l = 0; l < matToString.cols; l++) {
              ss << matToString.at<double>(k, l);
              if (l < (matToString.cols - 1)) {
                  ss << ", ";
              }
          }
          ss << "\n";
      }
      return ss.str();
  }

  /**
  * Collects the image points for the stereo image pairs from both cameras.
  */
  void StereoCalibration::collectImagePointsFromCams() {
      leftImagePointsAll.clear();
      rightImagePointsAll.clear();
      for (unsigned int i = 0; i < usableSourceImages.size(); i++) {
          leftImagePointsAll.push_back(leftCam->getImagePointsForPos(usableSourceImages[i]));
          rightImagePointsAll.push_back(rightCam->getImagePointsForPos(usableSourceImages[i]));
      }
  }

  /**
  * Collects the RMS for the stereo image pairs and calculates the mean value of
  * it for each pair.
  */
  void StereoCalibration::collectRmsFromCams() {
      rms.clear();
      for (unsigned int i = 0; i < usableSourceImages.size(); i++) {
          float r = (leftCam->getRmsForPos(usableSourceImages[i]) + rightCam->getRmsForPos(usableSourceImages[i])) * 0.5;
          rms.push_back(r);
      }
  }

  ///**
  // * Sorts all image data by RMS.
  // */
  //void StereoCalibration::sortAllByRms() {
  //    unsigned int n = rms.size();
  //    while (n > 0) {
  //        unsigned int newn = 0;
  //        for (unsigned int i = 0; i < n - 1; i++) {
  //            if (rms[i] > rms[i + 1]) {
  //                float tempError = rms[i + 1];
  //                rms[i + 1] = rms[i];
  //                rms[i] = tempError;

  //                unsigned int tempPos = usableSourceImages[i + 1];
  //                usableSourceImages[i + 1] = usableSourceImages[i];
  //                usableSourceImages[i] = tempPos;

  //                std::vector<cv::Point2f> tempImagePoints = leftImagePointsAll[i + 1];
  //                leftImagePointsAll[i + 1] = leftImagePointsAll[i];
  //                leftImagePointsAll[i] = tempImagePoints;

  //                tempImagePoints = rightImagePointsAll[i + 1];
  //                rightImagePointsAll[i + 1] = rightImagePointsAll[i];
  //                rightImagePointsAll[i] = tempImagePoints;

  //                newn = i + 1;
  //            }
  //        }
  //        n = newn;
  //    }
  //}

  /**
  * Initializes the object points.
  */
  void StereoCalibration::initializeObjectPoints() {
      objectPoints.clear();
      std::vector<cv::Point3f> vec;
      vec.push_back(cv::Point3f(0, 0, 0));
      objectPoints.push_back(vec);
      objectPoints[0].clear();
      cv::Size dim = leftCam->getPatternDim();
      int siz = leftCam->getPatternSize();
      for(int i = 0; i < dim.height; ++i)
          for(int j = 0; j < dim.width; ++j)
              objectPoints[0].push_back(cv::Point3f(float(j * siz),
                                                    float(i * siz), 0));
      objectPoints.resize(leftImagePoints.size(), objectPoints[0]);
  }

  /**
  * Saves images and calibration results to the given folder.
  * @param folderName The folder to save to.
  * @return True if data was saved, otherwise false.
  */
  bool StereoCalibration::saveData(std::string folderName) {
      bool retVal = false;
      std::vector<std::string> str = leftCam->getSourceImages();
      Path *p = new Path(str.at(0));
      Path pp = p->getGrandParentDir();
      Path ppp = pp.getParentDir();
      Path calibration = ppp.createDir(folderName);
      if (calibration.getPath() != "") {
          saveMatOfDoublesToNumpy(calibration, "R.npy", R);
          saveMatOfDoublesToCsv(calibration, "R.csv", "Rotationmatrix", R);
          saveTToNumpy(calibration, "T.npy");
          saveMatOfDoublesToCsv(calibration, "T.csv", "Translationmatrix", T);
          saveMatOfDoublesToNumpy(calibration, "E.npy", E);
          saveMatOfDoublesToCsv(calibration, "E.csv", "Essentialmatrix", E);
          saveMatOfDoublesToNumpy(calibration, "F.npy", F);
          saveMatOfDoublesToCsv(calibration, "F.csv", "Fundamentalmatrix", F);
          prepareRectification();
          saveQToNumpy(calibration, "Q.npy");
          saveMatOfDoublesToCsv(calibration, "Q.csv", "Qmatrix", Q);

          saveTargetImagesUndistortedandRectified(calibration);
          retVal = true;
      }
      return retVal;
  }

  /**
  * Prepares the rectification by computing the necessary rectification maps.
  */
  void StereoCalibration::prepareRectification() {
      cv::stereoRectify(leftCam->getCameraMatrix(),leftCam->getDistortionCoeffcients(), rightCam->getCameraMatrix(), rightCam->getDistortionCoeffcients(), leftCam->getImageSize(), R, T, rLeft, rRight, pLeft, pRight, Q, cv::CALIB_ZERO_DISPARITY, 0);
      cv::initUndistortRectifyMap(leftCam->getCameraMatrix(), leftCam->getDistortionCoeffcients(), rLeft, newCamMatrixLeft, leftCam->getImageSize(), CV_32FC2, leftMapOne, leftMapTwo);
      cv::initUndistortRectifyMap(rightCam->getCameraMatrix(), rightCam->getDistortionCoeffcients(), rRight, newCamMatrixRight, rightCam->getImageSize(), CV_32FC2, rightMapOne, rightMapTwo);
  }

  /**
  * Rectifies and saves the used images to the given path.
  * @param p The path where to save the images.
  */
  void StereoCalibration::saveTargetImagesUndistortedandRectified(Path p) {
      Path pictures = p.createDir("calibration_pictures");
      targetImages.clear();
      std::vector<std::string> imgPathsLeft = leftCam->getSourceImages();
      std::vector<std::string> imgPathsRight = rightCam->getSourceImages();
      for (unsigned int i = 0; i < usedSourceImages.size(); i++) {
          Path imageLeft = Path(imgPathsLeft[usedSourceImages[i]]);
          cv::Mat mLeft = leftCam->loadImage(usedSourceImages[i]);
          cv::Mat m2Left;
          mLeft.copyTo(m2Left);
          cv::remap(mLeft, m2Left, leftMapOne, leftMapTwo, cv::INTER_LINEAR);
          std::stringstream ssLeft;
          ssLeft << pictures.getPath() << imageLeft.getFile();
          cv::imwrite(ssLeft.str(), m2Left);
          targetImages.push_back(ssLeft.str());

          Path imageRight = Path(imgPathsRight[usedSourceImages[i]]);
          cv::Mat mRight = rightCam->loadImage(usedSourceImages[i]);
          cv::Mat m2Right;
          mRight.copyTo(m2Right);
          cv::remap(mRight, m2Right, rightMapOne, rightMapTwo, cv::INTER_LINEAR);
          std::stringstream ssRight;
          ssRight << pictures.getPath() << imageRight.getFile();
          cv::imwrite(ssRight.str(), m2Right);
          targetImages.push_back(ssRight.str());
      }
  }

  /**
  * Saves a 'cv::Mat' object to a numpy file.
  * @param p The path where to save.
  * @param fileName The name of the file to save.
  * @param matTosave The 'cv::Mat' object to save.
  */
  void StereoCalibration::saveMatOfDoublesToNumpy(Path p, std::string fileName, cv::Mat matTosave) {
      std::stringstream ss;
      ss << p.getPath() << fileName;

      double dataCam[matTosave.rows * matTosave.cols];
      for (int i = 0; i < matTosave.rows; i++) {
          for (int j = 0; j < matTosave.cols; j++) {
              dataCam[i * matTosave.cols + j] = matTosave.at<double>(i, j);
          }
      }

      const unsigned int shapeCam[] = {(const unsigned int) matTosave.rows,
                                      (const unsigned int) matTosave.cols};

      cnpy::npy_save(ss.str(), dataCam, shapeCam, 2, "w");
  }

  /**
  * Adjust and save the values of the translation vector to a numpy file.
  * @param p The path where to save.
  * @param fileName The name of the file to save.
  */
  void StereoCalibration::saveTToNumpy(Path p, std::string fileName) {
      std::stringstream ss;
      ss << p.getPath() << fileName;

      double dataCam[T.rows * T.cols];
      for (int i = 0; i < T.rows; i++) {
          for (int j = 0; j < T.cols; j++) {
              dataCam[i * T.cols + j] = T.at<double>(i, j) / 1000;
          }
      }

      const unsigned int shapeCam[] = {(const unsigned int) T.rows,
                                      (const unsigned int) T.cols};

      cnpy::npy_save(ss.str(), dataCam, shapeCam, 2, "w");
  }

  /**
  * Adjust and save the values of the q matrix to a numpy file.
  * @param p The path where to save.
  * @param fileName The name of the file to save.
  */
  void StereoCalibration::saveQToNumpy(Path p, std::string fileName) {
      std::stringstream ss;
      ss << p.getPath() << fileName;

      double dataCam[T.rows * T.cols];
      for (int i = 0; i < T.rows; i++) {
          for (int j = 0; j < T.cols; j++) {
              if (i == 4 && j == 3) {
                  dataCam[i * T.cols + j] = T.at<double>(i, j) / 1000;
              } else {
                  dataCam[i * T.cols + j] = T.at<double>(i, j);
              }
          }
      }

      const unsigned int shapeCam[] = {(const unsigned int) T.rows,
                                      (const unsigned int) T.cols};

      cnpy::npy_save(ss.str(), dataCam, shapeCam, 2, "w");
  }

  /**
  * Saves a 'cv::Mat' object to a csv file.
  * @param p The path where to save.
  * @param fileName The name of the file to save.
  * @param matName The name of the 'cv::Mat' object to save inside the file.
  * @param matTosave The 'cv::Mat' object to save.
  */
  void StereoCalibration::saveMatOfDoublesToCsv(Path p, std::string fileName, std::string matName, cv::Mat matTosave) {
      std::stringstream ss;
      ss << p.getPath() << fileName;

      std::ofstream f;
      f.open(ss.str().c_str());
      f << std::setprecision(std::numeric_limits<double>::digits10 + 2);

      f << matName << "\n";
      for (int k = 0; k < matTosave.rows; k++) {
          for (int l = 0; l < matTosave.cols; l++) {
              f << matTosave.at<double>(k, l);
              if (l < (matTosave.cols - 1)) {
                  f << ",";
              }
          }
          f << "\n";
      }

      f.close();
  }
}