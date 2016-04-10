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

#include "cameracalibration.h"

namespace scc {

  /**
  * Default constructor.
  */
  CameraCalibration::CameraCalibration() {
      sourceImagePosition = -1;
      patternSize = 0;
  }

  /**
  * Constructor.
  * @param size Size of calibration pattern elements.
  * @param patternH Horizontal amount of calibration pattern elements.
  * @param patternV Vertical amount of calibration pattern elements.
  * @param files List of source files pathes for calibration.
  */
  CameraCalibration::CameraCalibration(int size, int patternH, int patternV,
                                      std::vector<std::string> files,
                                      std::string desc) {
      description = desc;
      patternSize = size;
      // How to set pattern dimensions: patternDim =
      // cvSize(points_per_row, points_per_colum) =  cvSize(columns, rows)
      patternDim = cv::Size(patternH - 1, patternV - 1);
      sourceImages = files;

      sourceImagePosition = -1;
      // Set empty first element to objectPoints
      std::vector<cv::Point3f> vec;
      vec.push_back(cv::Point3f(0, 0, 0));
      objectPoints.push_back(vec);
      objectPoints[0].clear();
      initializeImageSize();

      cameraMatrix = cv::Mat::eye(3, 3, CV_64F); // matrix with ones on its
                                                // diagonal, others zeros
      distortionCoeffcients = cv::Mat::zeros(8, 1, CV_64F);
      computeLookUpTable();
  }

  /**
  *
  * @return
  */
  std::string CameraCalibration::getDescription() {
      return description;
  }

  /**
  * Processes the next image for the calibration process.
  * @return -1 if there is no image data, 0 if no calibration pattern was found,
  *          1 if calibration pattern was found
  */
  int CameraCalibration::processNextImage() {
      int retVal = -1;
      cv::Mat m = loadImageGray(sourceImagePosition + 1);
      if (m.data) {
          retVal = 0;
          sourceImagePosition++;
          std::vector<cv::Point2f> corners = findPatternCorners(m);
          if (corners.size() == (unsigned int) patternDim.width *
                  patternDim.height) {
              retVal = 1;
              usableSourceImagePos.push_back(sourceImagePosition);
              imagePoints.push_back(corners);
          }
      }
      return retVal;
  }

  /**
  * Runs the calibration based on the already processed images.
  * @return The reprojection error returned by opencv function 'calibrateCamera'
  */
  double CameraCalibration::calibrate() {
      double retVal;
      initializeCalibrationParameter(imagePoints);
      retVal = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
                                  cameraMatrix, distortionCoeffcients,
                                  rotationVectors, translationVectors);
      perViewErrors = computeReprojectionErrors(imagePoints, rotationVectors,
                                                translationVectors);
      if (usedSourceImagePos.size() == 0) {
          usedSourceImagePos = usableSourceImagePos;
          allImagePoints = imagePoints;
      }
      return retVal;
  }

  /**
  * Removes data about images used for calibration. Images with worst RMS are
  * removed first. Images are removed till a given minimum of remaining images or
  * a desired perentage of removed images is reached.
  * @param minAmount Minimum amount of remaining images.
  * @param percentageToRemove Percentage of images to remove.
  * @return 1 if there are less or an equal amount of images remaining than the
  * 'minAmount', 0 if the desired percentage was removed and -1 in all other
  * cases.
  */
  int CameraCalibration::removeHighRms(unsigned int minAmount,
                                      double percentageToRemove) {
      int retVal = -1;
      sortByRmsForUsedImg();
      unsigned int amountToRemove = usedSourceImagePos.size() *
              percentageToRemove;
      unsigned int removedCounter = 0;
      if (usedSourceImagePos.size() > minAmount && amountToRemove == 0) {
          amountToRemove = 1;
      }
      while (usedSourceImagePos.size() > minAmount &&
            removedCounter < amountToRemove) {
          usedSourceImagePos.pop_back();
          imagePoints.pop_back();
          perViewErrors.pop_back();
          removedCounter++;
      }
      if (usedSourceImagePos.size() <= minAmount) {
          retVal = 1;
      } else if (removedCounter == amountToRemove) {
          retVal = 0;
      } else {
          retVal = -1;
      }

      return retVal;
  }

  /**
  * Removes data about all images. Images with worst RMS are removed first.
  * Images are removed till a given minimum of remaining images or a desired
  * perentage of removed images is reached.
  * @param minAmount Minimum amount of remaining images.
  * @param percentageToRemove Percentage of images to remove.
  * @return Amount of removed images and their data.
  */
  unsigned int CameraCalibration::removeHighRmsForAllImg(float maxRms,
                                                  unsigned int minAmountLeft) {
      unsigned int old = usableSourceImagePos.size();
      sortByRmsForAllImg();
      while (allPerViewErrors[allPerViewErrors.size() - 1] > maxRms &&
            allPerViewErrors.size() > minAmountLeft) {
          usableSourceImagePos.pop_back();
          allImagePoints.pop_back();
          allAngles.pop_back();
          allPerViewErrors.pop_back();
      }
      return old - usableSourceImagePos.size();
  }

  /**
  * Computes rotation matrix and translation vector for all usable images.
  */
  void CameraCalibration::computeRotAndTransForAllImg() {
      initializeObjectPoints(allImagePoints);
      allRotationVectors.resize(objectPoints.size());
      allTranslationVectors.resize(objectPoints.size());
      allAngles.clear();
      for (unsigned int i = 0; i < usableSourceImagePos.size(); i++) {
          cv::solvePnP(objectPoints[i], allImagePoints[i], cameraMatrix,
                      distortionCoeffcients, allRotationVectors[i],
                      allTranslationVectors[i]);
          cv::Mat rotationMatrix;
          cv::Rodrigues(allRotationVectors[i], rotationMatrix);
          cv::Mat cameraMatrix, rotMatrix, transVect;
          cv::Mat rotMatrixX, rotMatrixY, rotMatrixZ;
          double* _r = rotationMatrix.ptr<double>();
          double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                                _r[3],_r[4],_r[5],0,
                                _r[6],_r[7],_r[8],0};
          cv::Vec3d eulerAngles;
          decomposeProjectionMatrix(cv::Mat(3, 4, CV_64FC1, projMatrix),
                                    cameraMatrix,
                                    rotMatrix,
                                    transVect,
                                    rotMatrixX,
                                    rotMatrixY,
                                    rotMatrixZ,
                                    eulerAngles);
          allAngles.push_back(eulerAngles);
      }
  }

  /**
  * Computes the RMS for all usable images.
  */
  void CameraCalibration::computeReprojectionErrorForAllImg() {
      allPerViewErrors = computeReprojectionErrors(allImagePoints,
                                                  allRotationVectors,
                                                  allTranslationVectors);
  }

  /**
  * Checks the rotation of the images used for calibration.
  * @param minDegree The minimum angle desired.
  * @return True if an image has the desired rotation, otherwise false.
  */
  bool CameraCalibration::checkUsedImagesForRotation(double minDegree) {
      unsigned int i = 0;
      bool found = false;
      while (i < usedSourceImagePos.size() && found == false) {
          unsigned int j = findPositionInUsableSourceImgPosByPos(usedSourceImagePos[i]);
          cv::Vec3d angles = allAngles[j];
          if (abs(angles[0]) >= minDegree || abs(angles[1]) >= minDegree ||
                  abs(angles[2]) >= minDegree) {
              found = true;
          }
          i++;
      }
      return found;
  }

  /**
  * Searches all usable images for an image with desired rotation and checks
  * wheter it is already used or not.
  * @param minDegree The minimum angle desired.
  * @return -1 if no image was found, otherwise the position of the found image.
  */
  int CameraCalibration::checkForRotatedImage(double minDegree) {
      unsigned int i = 0;
      int pos = -1;
      while (pos == -1 && i < usableSourceImagePos.size()) {
          cv::Vec3d angles = allAngles[i];
          if (abs(angles[0]) >= minDegree || abs(angles[1]) >= minDegree ||
              abs(angles[2]) >= minDegree) {
              if (!isPosUsed(usableSourceImagePos[i])) {
                  pos = usableSourceImagePos[i];
              }
          }
          i++;
      }
      return pos;
  }

  /**
  * Searches all usable images for an image with desired distribution and checks
  * wheter it is already used or not.
  * @return -1 if no image was found, otherwise the position of the found image.
  */
  int CameraCalibration::checkForDistributedImage() {
      int imagePos = -1;
      bool found = false;
      unsigned int i = 0;
      while (i < imagePointDistribution.size() && found == false) {
          if (imagePointDistribution[i] == 0) {
              unsigned int counter = 0;
              while (counter < imagePointDistributionPerView.size() &&
                    found == false) {
                  std::vector<unsigned int> distribution = imagePointDistributionPerView[counter];
                  if (distribution[i] != 0) {
                      found = true;
                      imagePos = counter;
                  }
                  counter++;
              }
          }
          i++;
      }
      return imagePos;
  }

  /**
  * Adds an image and its image points from useable to used.
  * @param imageToAdd Position of the image to add.
  */
  void CameraCalibration::addImageFromUseableToUsed(unsigned int imageToAdd) {
      unsigned int j = findPositionInUsableSourceImgPosByPos(imageToAdd);
      usedSourceImagePos.push_back(usableSourceImagePos[j]);
      imagePoints.push_back(allImagePoints[j]);
  }

  /**
  * Compute the distribution of the image points across the frame and checks it.
  * @return True if there is a full distribution across the frame otherwise
  * false.
  */
  bool CameraCalibration::computeImagePointDistribution() {
      imagePointDistribution.resize((patternDim.height - 1) *
                                    (patternDim.width - 1));
      imagePointDistributionPerView.resize(imagePoints.size());
      std::fill(imagePointDistribution.begin(), imagePointDistribution.end(), 0);
      for (unsigned int i = 0; i < imagePoints.size(); i++) {
          std::vector<cv::Point2f> points = imagePoints[i];
          std::vector<unsigned int> distributionPerView;
          distributionPerView.resize(imagePointDistribution.size());
          std::fill(distributionPerView.begin(), distributionPerView.end(), 0);
          for (unsigned int j = 0; j < points.size(); j++) {
              cv::Point2f p = points[j];
              unsigned int location = lookUpTable.at<uchar>(p);
              imagePointDistribution[location] += 1;
              distributionPerView[location] += 1;
          }
          imagePointDistributionPerView[i] = distributionPerView;
      }
      bool completlyFilled = true;
      for (unsigned int i = 0; i < imagePointDistribution.size(); i++) {
          if (imagePointDistribution[i] == 0) {
              completlyFilled = false;
          }
      }
      return completlyFilled;
  }

  /**
  * Returns the paths to the images used for calibration.
  * @return The vector containing the strings.
  */
  std::vector<std::string> CameraCalibration::getUsedSourceImages() {
      std::vector<std::string> retVal;
      for (unsigned int i = 0; i < usedSourceImagePos.size(); i++) {
          retVal.push_back(sourceImages.at(usedSourceImagePos.at(i)));
      }
      return retVal;
  }

  /**
  * Returns the paths to all source images.
  * @return The vector containing the strings.
  */
  std::vector<std::string> CameraCalibration::getSourceImages() {
      return sourceImages;
  }

  /**
  * Returns the paths to the saved calibration images.
  * @return The vector containing the strings.
  */
  std::vector<std::string> CameraCalibration::getTargetImages() {
      return targetImages;
  }

  /**
  * Returns all usable image positions.
  * @return The vector containing the positions.
  */
  std::vector<unsigned int> CameraCalibration::getUsableSourceImagePos() {
      return usableSourceImagePos;
  }

  /**
  * Returns the number of useable source images.
  * @return The number of useable source images
  */
  unsigned int CameraCalibration::getUsableSourceImageAmount() {
      return usableSourceImagePos.size();
  }

  /**
  * Returns all used source image positions.
  * @return A vector containing the positions.
  */
  std::vector<unsigned int> CameraCalibration::getUsedSourceImagePos() {
      return usedSourceImagePos;
  }

  /**
  * Returns the object points.
  * @return A vector containing vectors containing each images object points.
  */
  std::vector<std::vector<cv::Point3f> > CameraCalibration::getObjectPoints() {
      return objectPoints;
  }

  /**
  * Returns the image points.
  * @return A vector containing vectors containing each images image points.
  */
  std::vector<std::vector<cv::Point2f> > CameraCalibration::getImagePoints() {
      return imagePoints;
  }

  /**
  * Returns the image points for a specific image position.
  * @param position The position to return image points for.
  * @return A vector containing the image points.
  */
  std::vector<cv::Point2f> CameraCalibration::getImagePointsForPos(
          unsigned int position) {
      unsigned int pos = findPositionInUsableSourceImgPosByPos(position);
      return allImagePoints[pos];
  }

  /**
  * Returns the RMS for a specific image position.
  * @param position The position to return the RMS for.
  * @return The RMS value.
  */
  float CameraCalibration::getRmsForPos(unsigned int position) {
      unsigned int pos = findPositionInUsableSourceImgPosByPos(position);
      return allPerViewErrors[pos];
  }

  /**
  * Returns the maximum rotation angle for a specific image position.
  * @param position The position to return the maximum rotation for.
  * @return The angle value.
  */
  double CameraCalibration::getMaxRotationForPos(unsigned int position) {
      unsigned int pos = findPositionInUsableSourceImgPosByPos(position);
      cv::Vec3d ang = allAngles[pos];
      double retVal = abs(ang[0]);
      if (abs(ang[1]) > retVal) {
          retVal = abs(ang[1]);
      }
      if (abs(ang[2]) > retVal) {
          retVal = abs(ang[2]);
      }
      return retVal;
  }

  /**
  * Returns a 'cv::Mat' representing the image point distribution across the
  * frame.
  * @return A 'cv::Mat' object.
  */
  cv::Mat CameraCalibration::getImagePointsDistributionImg() {
      cv::Mat *retVal = new cv::Mat(imageSize, CV_8UC3, CV_RGB(255, 255, 255));
      unsigned int xSectors = patternDim.width - 1;
      unsigned int ySectors = patternDim.height - 1;
      unsigned int sectorWidth = imageSize.width / xSectors;
      unsigned int sectorHeight = imageSize.height / ySectors;

      for (unsigned int i = 1; i < xSectors; i++) {
          cv::line(*retVal, cv::Point(i * sectorWidth, 0),
                  cv::Point(i * sectorWidth, retVal->rows), CV_RGB(255, 0, 0),
                  3, 8, 0);
      }

      for (unsigned int i = 1; i < ySectors; i++) {
          cv::line(*retVal, cv::Point(0, i * sectorHeight),
                  cv::Point(retVal->cols, i * sectorHeight), CV_RGB(255, 0, 0),
                  3, 8, 0);
      }

      for (unsigned int i = 0; i < imagePoints.size(); i++) {
          std::vector<cv::Point2f> points = imagePoints.at(i);
          for (unsigned int j = 0; j < points.size(); j++) {
              cv::circle(*retVal, points.at(j), 5, 0, -1);
          }

      }
      return *retVal;
  }

  /**
  * Returns a string describing the camera matrix.
  * @return The string.
  */
  std::string CameraCalibration::getCameraMatrixString() {
      std::stringstream ss;
      ss << std::setprecision(std::numeric_limits<double>::digits10 + 2);

      for (int k = 0; k < cameraMatrix.rows; k++) {
          for (int l = 0; l < cameraMatrix.cols; l++) {
              ss << cameraMatrix.at<double>(k, l);
              if (l < (cameraMatrix.cols - 1)) {
                  ss << ", ";
              }
          }
          ss << "\n";
      }
      return ss.str();
  }

  /**
  * Returns a string describing the distortion matrix.
  * @return The string.
  */
  std::string CameraCalibration::getDistortionMatrixString() {
      std::stringstream ss;
      ss << std::setprecision(std::numeric_limits<double>::digits10 + 2);

      for (int k = 0; k < distortionCoeffcients.rows; k++) {
          for (int l = 0; l < distortionCoeffcients.cols; l++) {
              ss << distortionCoeffcients.at<double>(k, l);
              if (l < (distortionCoeffcients.cols - 1)) {
                  ss << ", ";
              }
          }
          ss << "\n";
      }
      return ss.str();
  }

  /**
  * Returns a string describing the status of already processed images based
  * on all provided images.
  * @return A string describing the image processing status.
  */
  std::string CameraCalibration::getStatus() {
      std::stringstream retVal;
      retVal << "Image " << sourceImagePosition + 1 << " of "
            << sourceImages.size() << " processed. "
            << usableSourceImagePos.size()
            << " images are usable for calibration.";
      return retVal.str();
  }

  /**
  * Returns the pattern dimensions.
  * @return A 'cv::Sizes' object.
  */
  cv::Size CameraCalibration::getPatternDim() {
      return patternDim;
  }

  /**
  * Returns the pattern size.
  * @return The pattern size value.
  */
  int CameraCalibration::getPatternSize() {
      return patternSize;
  }

  /**
  * Returns the image size.
  * @return A 'cv::Sizes' object.
  */
  cv::Size CameraCalibration::getImageSize() {
      return imageSize;
  }

  /**
  * Returns the amount of source images.
  * @return The amount of source images.
  */
  unsigned int CameraCalibration::getSourceImagesNo() {
      return sourceImages.size();
  }

  /**
  * Return the camera matrix.
  * @return The 'cv::Mat' object.
  */
  cv::Mat CameraCalibration::getCameraMatrix() {
      return cameraMatrix;
  }

  /**
  * Returns the distortion coefficients.
  * @return The 'cv::Mat' object.
  */
  cv::Mat CameraCalibration::getDistortionCoeffcients() {
      return distortionCoeffcients;
  }

  /**
  * Returns the path to the image of the distibution of image points.
  * @return A string holding the path.
  */
  std::string CameraCalibration::getImagePointsDistributionImagePath() {
      return imagePointsDistributionImagePath;
  }

  /**
  * Returns a string describing the object and its attributes.
  * @return A String describing the object.
  */
  std::string CameraCalibration::print() {
      std::string retString = "";
      retString.append("CameraCalibration Object\n");
      retString.append("=========================\n");
      retString.append("Square size: ");
      retString.append(printNumber(patternSize));
      retString.append("\nSquare dimensions: ");
      retString.append(printNumber(patternDim.width));
      retString.append(" x ");
      retString.append(printNumber(patternDim.height));
      retString.append("\nSource file position: ");
      retString.append(printNumber(sourceImagePosition));
      retString.append("\nSource files:\n");
      retString.append(printVectorOfStrings(sourceImages));
      return retString;
  }

  /**
  * Initialises the image size based on the size of the provided images trough
  * the given source files.
  */
  void CameraCalibration::initializeImageSize() {
      imageSize = cv::Size(0, 0);
      if (sourceImages.size() > 0) {
          cv::Mat m = loadImageGray(0);
          if (m.data) {
              imageSize = m.size();
          }
      }
  }

  /**
  * Computes the lookup table based on image size and pattern dimension. It is
  * used to check the image point distribution.
  */
  void CameraCalibration::computeLookUpTable() {
      lookUpTable = cv::Mat::zeros(imageSize.height, imageSize.width, CV_8U);
      unsigned int xSectors = patternDim.width - 1;
      unsigned int ySectors = patternDim.height - 1;
      unsigned int sectorWidth = imageSize.width / xSectors;
      unsigned int sectorHeight = imageSize.height / ySectors;
      for (int y = 0; y < lookUpTable.rows; y++) {
          for (int x = 0; x < lookUpTable.cols; x++) {
              unsigned int a = y / sectorHeight;
              if (a >= ySectors) {
                  a = ySectors - 1;
              }
              unsigned int b = x / sectorWidth;
              if (b >= xSectors) {
                  b = xSectors - 1;
              }
              lookUpTable.at<uchar>(y, x) = a * xSectors + b;
          }
      }
  }

  /**
  * Initializes all the parameter used for the calibration.
  * @param imgPoints The image points that will be used for calibration.
  */
  void CameraCalibration::initializeCalibrationParameter(
          std::vector<std::vector<cv::Point2f> > imgPoints) {
      initializeObjectPoints(imgPoints);
      cameraMatrix = cv::Mat::eye(3, 3, CV_64F); // matrix with ones on its
                                                // diagonal, others zeros
      distortionCoeffcients = cv::Mat::zeros(8, 1, CV_64F);
      rotationVectors.clear();
      translationVectors.clear();
  }

  /**
  * Initializes the object points for calibration.
  * @param imgPoints The image points that will be used for calibration.
  */
  void CameraCalibration::initializeObjectPoints(
          std::vector<std::vector<cv::Point2f> > imgPoints) {
      objectPoints.clear();
      std::vector<cv::Point3f> vec;
      vec.push_back(cv::Point3f(0, 0, 0));
      objectPoints.push_back(vec);
      objectPoints[0].clear();
      for (int i = 0; i < patternDim.height; ++i) {
          for (int j = 0; j < patternDim.width; ++j) {
              objectPoints[0].push_back(cv::Point3f(float(j * patternSize),
                                                    float(i * patternSize), 0));
          }
      }
      objectPoints.resize(imgPoints.size(), objectPoints[0]);
  }

  /**
  * Returns the position of an image poisition inside of all usable images.
  * @param usedSourceImagePosToFind The image position to search for.
  * @return The position where the searched image position was found.
  */
  unsigned int CameraCalibration::findPositionInUsableSourceImgPosByPos(
          unsigned int usedSourceImagePosToFind) {
      bool found = false;
      unsigned int i = 0;
      unsigned int retVal = 0;
      while (i < usableSourceImagePos.size() && found == false) {
          if (usedSourceImagePosToFind == usableSourceImagePos[i]) {
              retVal = i;
              found = true;
          }
          i++;
      }
      return retVal;
  }

  /**
  * Checks wheter an image is used for calibration or not.
  * @param posToCheck Position refrencing the image to check for.
  * @return True if it is used for calibration otherwise false.
  */
  bool CameraCalibration::isPosUsed(unsigned int posToCheck) {
      bool isUsed = false;
      unsigned int counter = 0;
      while (counter < usedSourceImagePos.size() && isUsed == false) {
          if (usedSourceImagePos[counter] == posToCheck) {
              isUsed = true;
          }
          counter++;
      }
      return isUsed;
  }

  /**
  * Loads the image from the given position of the provided list of source
  * files.
  * @param filePosition Position of source file list, where to load image.
  * @return cv::Mat object, containing the image data.
  */
  cv::Mat CameraCalibration::loadImageGray(unsigned int filePosition) {
      cv::Mat retMat;
      if (filePosition < sourceImages.size()) {
          retMat = cv::imread(sourceImages.at(filePosition),
                              CV_LOAD_IMAGE_GRAYSCALE);
      }
      return retMat;
  }

  /**
  * Loads a specific image file from the list of source files.
  * @param filePosition Position referencing the image to load.
  * @return A 'cv::Mat' containing the loaded image.
  */
  cv::Mat CameraCalibration::loadImage(unsigned int filePosition) {
      cv::Mat retMat;
      if (filePosition < sourceImages.size()) {
          retMat = cv::imread(sourceImages.at(filePosition),
                              CV_LOAD_IMAGE_UNCHANGED);
      }
      return retMat;
  }

  /**
  * Finds the calibration pattern corner in the given image data. And refines
  * their position if found.
  * @param image The image data where to find the calibration pattern corners.
  * @return True if the corners wehere found, otherwise false.
  */
  std::vector<cv::Point2f> CameraCalibration::findPatternCorners(cv::Mat image) {
      std::vector<cv::Point2f> corners;
      bool patternFound = cv::findChessboardCorners(image, patternDim, corners,
                                                    cv::CALIB_CB_ADAPTIVE_THRESH);
      if (patternFound) {
          corners = refineCorners(image, corners);
      }
      return corners;
  }

  /**
  * Refines the given calibration pattern corners found in the given image data.
  * @param image The image data where the given corners where found.
  * @param corners The corners positions to refine.
  * @return The refined corner positions.
  */
  std::vector<cv::Point2f> CameraCalibration::refineCorners(cv::Mat image,
                                              std::vector<cv::Point2f> corners) {
      cv::cornerSubPix(image, corners, cv::Size(3, 3), cv::Size(-1, -1),
                  cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.01));
      return corners;
  }

  /**
  * Computes the RMS for a given set of image points after a calibration was done.
  * @return The calculated RMS for each set of image points.
  */
  std::vector<float> CameraCalibration::computeReprojectionErrors(
          std::vector<std::vector<cv::Point2f> > imgPoints,
          std::vector<cv::Mat> rotVectors, std::vector<cv::Mat> transVectors) {
      std::vector<cv::Point2f> imagePoints2;
      std::vector<float> errors;
      initializeObjectPoints(imgPoints);
      errors.resize(objectPoints.size());
      double error = 0.0;
      for(unsigned int i = 0; i < objectPoints.size(); i++) {
          projectPoints(cv::Mat(objectPoints[i]), rotVectors[i], transVectors[i],
          cameraMatrix, distortionCoeffcients, imagePoints2);
          error = norm(cv::Mat(imgPoints[i]), cv::Mat(imagePoints2), cv::NORM_L2);
          unsigned int n = objectPoints[i].size();
          errors[i] = (float) std::sqrt(error * error / n);
      }
      return errors;
  }

  /**
  * Sorts the images used for calibration by their RMS.
  */
  void CameraCalibration::sortByRmsForUsedImg() {
      unsigned int n = perViewErrors.size();
      while (n > 0) {
          unsigned int newn = 0;
          for (unsigned int i = 0; i < n - 1; i++) {
              if (perViewErrors[i] > perViewErrors[i + 1]) {
                  float tempError = perViewErrors[i + 1];
                  int tempPos = usedSourceImagePos[i + 1];
                  std::vector<cv::Point2f> tempImagePoints = imagePoints[i + 1];
                  perViewErrors[i + 1] = perViewErrors[i];
                  imagePoints[i + 1] = imagePoints[i];
                  usedSourceImagePos[i + 1] = usedSourceImagePos[i];
                  perViewErrors[i] = tempError;
                  usedSourceImagePos[i] = tempPos;
                  imagePoints[i] = tempImagePoints;
                  newn = i + 1;
              }
          }
          n = newn;
      }
  }

  /**
  * Sorts all useable images by their RMS.
  */
  void CameraCalibration::sortByRmsForAllImg() {
      unsigned int n = allPerViewErrors.size();
      while (n > 0) {
          unsigned int newn = 0;
          for (unsigned int i = 0; i < n - 1; i++) {
              if (allPerViewErrors[i] > allPerViewErrors[i + 1]) {
                  float tempError = allPerViewErrors[i + 1];
                  allPerViewErrors[i + 1] = allPerViewErrors[i];
                  allPerViewErrors[i] = tempError;

                  int tempPos = usableSourceImagePos[i + 1];
                  usableSourceImagePos[i + 1] = usableSourceImagePos[i];
                  usableSourceImagePos[i] = tempPos;

                  std::vector<cv::Point2f> tempImagePoints =
                          allImagePoints[i + 1];
                  allImagePoints[i + 1] = allImagePoints[i];
                  allImagePoints[i] = tempImagePoints;

                  cv::Vec3d tempAngles = allAngles[i + 1];
                  allAngles[i + 1] = allAngles[i];
                  allAngles[i] = tempAngles;

                  newn = i + 1;
              }
          }
          n = newn;
      }
  }

  /**
  * Saves the calibration images and the calibration data.
  * @return True if saving was successful otherwise false.
  */
  bool CameraCalibration::saveData(std::string folderName) {
      bool retVal = false;
      Path *p = new Path(sourceImages[0]);
      Path pp = p->getGrandParentDir();
      Path calibration = pp.createDir(folderName);
      if (calibration.getPath() != "") {
          saveTargetImagesUndistorted(calibration);
          saveImagePointsDistribution(calibration);
          saveCameraMatrixToNumpy(calibration);
          saveCameraMatrixToCsv(calibration);
          saveDistCoeffToNumpy(calibration);
          saveDistCoeffToCsv(calibration);
          // TODO: save other data
          retVal = true;
      }
      return retVal;
  }

  /**
  * Saves the undistorted and marked images used for calibration.
  * @param p The path where to save the images.
  */
  void CameraCalibration::saveTargetImagesUndistorted(Path p) {
      std::stringstream ss;
      ss << "calibration_pictures_" << description;
      Path pictures = p.createDir(ss.str());
      for (unsigned int i = 0; i < usedSourceImagePos.size(); i++) {
          Path image = Path(sourceImages.at(usedSourceImagePos.at(i)));
          cv::Mat m = loadImage(usedSourceImagePos.at(i));
          cv::drawChessboardCorners(m, patternDim, imagePoints[i], true);
          cv::Mat m2;
          cv::undistort(m, m2, cameraMatrix, distortionCoeffcients);
          std::stringstream s;
          s << pictures.getPath() << image.getFile();
          cv::imwrite(s.str(), m2);
          targetImages.push_back(s.str());
      }
  }

  /**
  * Saves the image representing the image points distribution.
  * @param p The path where to save the image.
  */
  void CameraCalibration::saveImagePointsDistribution(Path p) {
      //Path pictures = p.createDir("calibration_pictures");
      cv::Mat m = getImagePointsDistributionImg();
      std::stringstream ss;
      ss << p.getPath() << "image_poits_distribution.jpg";
      cv::imwrite(ss.str(), m);
      imagePointsDistributionImagePath = ss.str();
  }

  /**
  TODO:
  translation vectors               -    tvecs_l.npy
  rotation vectors                    -    rvecs_l.npy
  image points                        -    image_points.npy
  object points                        -    object_points.npy
  **/

  /**
  * Save the camera matrix to a numpy file.
  * @param p Path where to save the file.
  */
  void CameraCalibration::saveCameraMatrixToNumpy(Path p) {
      //save camera matrix
      std::stringstream ss;
      ss << p.getPath() << "camera_matrix_" << description << ".npy";

      double dataCam[cameraMatrix.rows * cameraMatrix.cols];
      for (int i = 0; i < cameraMatrix.rows; i++) {
          for (int j = 0; j < cameraMatrix.cols; j++) {
              dataCam[i * cameraMatrix.cols + j] = cameraMatrix.at<double>(i, j);
          }
      }

      const unsigned int shapeCam[] = {(const unsigned int) cameraMatrix.rows,
                                      (const unsigned int) cameraMatrix.cols};

      cnpy::npy_save(ss.str(), dataCam, shapeCam, 2, "w");
  }

  /**
  * Saves the camera matrix to a csv file.
  * @param p Path where to save the file.
  */
  void CameraCalibration::saveCameraMatrixToCsv(Path p) {
      //save camera matrix
      std::stringstream ss;
      ss << p.getPath() << "camera_matrix_" << description << ".csv";

      std::ofstream f;
      f.open(ss.str().c_str());
      f << std::setprecision(std::numeric_limits<double>::digits10 + 2);

      f << "Camera matrix\n";
      for (int k = 0; k < cameraMatrix.rows; k++) {
          for (int l = 0; l < cameraMatrix.cols; l++) {
              f << cameraMatrix.at<double>(k, l);
              if (l < (cameraMatrix.cols - 1)) {
                  f << ",";
              }
          }
          f << "\n";
      }
      f.close();
  }

  /**
  * Saves the distortion coefficients to a numpy file.
  * @param p Path where to save the file.
  */
  void CameraCalibration::saveDistCoeffToNumpy(Path p) {
      // save distortion coefficients
      std::stringstream ss;
      ss << p.getPath() << "distortion_coefficients_" << description << ".npy";

      const int arrSize = distortionCoeffcients.rows * distortionCoeffcients.cols;
      double dataDist[arrSize];
      for (int k = 0; k < distortionCoeffcients.rows; k++) {
          for (int l = 0; l < distortionCoeffcients.cols; l++) {
              dataDist[k * distortionCoeffcients.cols + l] =
                                          distortionCoeffcients.at<double>(k, l);
          }
      }

      const unsigned int shapeDist[] = {(const unsigned int)
                                        distortionCoeffcients.rows,
                                        (const unsigned int)
                                        distortionCoeffcients.cols};

      cnpy::npy_save(ss.str(), dataDist, shapeDist, 2, "w");
  }

  /**
  * Saves the distortion coefficients to a csv file.
  * @param p Path where to save the file.
  */
  void CameraCalibration::saveDistCoeffToCsv(Path p) {
      // save distortion coefficients
      std::stringstream ss;
      ss << p.getPath() << "distortion_coefficients_" << description << ".csv";

      std::ofstream f;
      f.open(ss.str().c_str());
      f << std::setprecision(std::numeric_limits<double>::digits10 + 2);

      f << "Distortion coefficents matrix\n";
      for (int k = 0; k < distortionCoeffcients.rows; k++) {
          for (int l = 0; l < distortionCoeffcients.cols; l++) {
              f << distortionCoeffcients.at<double>(k, l);
              if (l < (distortionCoeffcients.cols - 1)) {
                  f << ",";
              }
          }
          f << "\n";
      }

      f.close();
  }

  /**
  * Converts an integer to a string.
  * @param number The integer to convert.
  * @return The string representing given integer.
  */
  std::string CameraCalibration::printNumber(int number) {
      std::stringstream ss;
      ss << number;
      return ss.str();
  }

  /**
  * Converts a vector of strings to one string, where the string ending is
  * marked by a newline.
  * @param vec The vector of string to convert to a single string.
  * @return The string containing all strings of the given vector.
  */
  std::string CameraCalibration::printVectorOfStrings(
                                                  std::vector<std::string> vec) {
      std::string retString = "";
      for (unsigned int i = 0; i < vec.size(); i++) {
        retString.append(vec.at(i));
        retString.append("\n");
      }
      return retString;
  }
}