# stereocamcalib
Purpose of this library is to provide additional functionality for the calibration of stereo cameras with chessboard patterns and provide as good as possible calibration data. It heavily relies on the openCV library and the cnpy library.
Functionality for sorting used images by different characteristics is also provided as checking pattern distribution and rotation for example.
The calibration data can saved as .csv and .npy files along with the images used for calibration.

Dependencies: openCV, cnpy

Install:

1. Download stereocamcalib (for example via wget)
2. cd ./stereocamcalib
2. mkdir build
3. cd ./build
4. cmake ..
5. make .
6. make install (as root)

