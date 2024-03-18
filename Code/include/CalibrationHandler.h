// Modified MIT License
//
// Copyright (c) 2019 tgwaAo
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Modified part:
//
// THIS SOFTWARE DOES NOT CHECK YOUR SURROUNDINGS NOR DOES IT CONTROL YOUR
// MOVEMENT, SO IT IS UNDER YOUR OWN RESPONSIBILITY TO ENSURE NOBODY GETS HURT
// AND NOTHING GETS DAMAGED. PLAY CAREFULLY AND CHECK YOUR SURROUNDINGS BEFORE
// PLAYING.

#ifndef CODE_INCLUDE_CALIBRATIONHANDLER_H_
#define CODE_INCLUDE_CALIBRATIONHANDLER_H_

#define EIGEN_MPL2_ONLY

#include <time.h>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>


// Define own matrix to save space
typedef Eigen::Matrix<
uint8_t,
Eigen::Dynamic,
Eigen::Dynamic,
Eigen::RowMajor> Matrix8u;

/**
 * @class CalibrationHandler
 * @author Uwe Simon
 * @date 08/10/19
 * @file CalibrationHandler.h
 * @brief Class to handle calibration. Marked areas of an image are used to calibrate values with eigens BDCSVD.
 */
class CalibrationHandler {
 public:
    /**
     * @brief Constructor with a variable list of possible parameters.
     * @param title Title of windows shown.
     * @param distanceText2Border Distance to left and lower border of window.
     * @param font Font of description text.
     * @param textScale Scale of description text.
     * @param textColor Color of description text.
     * @param textThickness Thickness of description text.
     */
    CalibrationHandler(
        std::string title = "Calibration",
        uint16_t distanceText2Border = 10,
        uint8_t font = cv::FONT_HERSHEY_SIMPLEX,
        float textScale = 1.2,
        cv::Scalar textColor = cv::Scalar(255, 255, 0),
        uint8_t textThickness = 2);

    /**
     * @brief Destoying object.
     */
    ~CalibrationHandler();

    /**
     * @brief Calibrates params. Values without error are found using histograms,
     * factors for predictions are calculated using eigens BDCSVD.
     * @param img Image used to visualize steps and to get values for calibration.
     * @param hsvIntern Values to decide, whether a color is inside
     *                  bright color spectrum or not.
     * @param factorsIntern Factors to calculate prediction of dark color spectrum.
     * @return Acception of new values.
     */
    bool calibrate(
        cv::Mat *img,
        std::vector<double> *const optimalValuesIntern,
        std::vector<double> *const errorFactorsIntern);

    /**
     * @brief Set title of shown window.
     * @param title Title of shown window.
     */
    void setTitle(const std::string &title);

    /**
     * @brief Set font of explanation text.
     * @param font Font of explanation text.
     */
    void setFont(const uint8_t font);

    /**
     * @brief Set color of explanation text.
     * @param textColor Color of explanation text.
     */
    void setTextColor(const cv::Scalar textColor);

    /**
     * @brief Set scale of explanation text.
     * @param textScale Scale of explanation text.
     */
    void setTextScale(float textScale);

    /**
     * @brief Set thickness of explanation text.
     * @param textThickness Thickness of explanation text.
     */
    void setTextThickness(const uint8_t textThickness);

    /**
     * @brief Set maximum of iterations in calibration of error factors.
     * @param maxIteration Max. iterations in calibration of error factors.
     */
    void setMaxIterationFactors(const uint16_t maxIterationFactors);

    /**
     * @brief Set minimum of false positives in factor calibration.
     * @param minError Minimum of false positives in factor clibration.
     */
    void setMinErrorFactors(const double minErrorFactors);

    /**
    * @brief Set minimum border to finish calibration of factors.
    * @param minCorrection Factors Minimum correction border to finish calibration of factors.
    */
    void setMinCorrectionFactors(const double minCorrectionFactors);

    /**
    * @brief Set start index of positive values in matrix of input points.
    * @param startIdxPositives Start index of positives in matrix of input points.
    */
    void setStartIdxPositives(const uint64_t startIdxPositives);

    /**
     * @brief Set distance of negative colors picked in image.
     * @param negDist Distance of negative colors picked in image.
     */
    void setNegDist(const uint8_t negDist);

    /**
     * @brief Set distance of positive colors picked in image.
     * @param posDist Distance of positive colors picked in image.
     */
    void setPosDist(const uint8_t posDist);

    /**
     * @brief Get title of shown window.
     * @return Title of shown window.
     */
    std::string getTitle() const;

    /**
     * @brief Get font of shown explanation text.
     * @return Font of shown explanation text.
     */
    uint8_t getFont() const;

    /**
     * @brief Get color of explanation text.
     * @return Color of explanation text.
     */
    cv::Scalar getTextColor() const;

    /**
     * @brief Get scale of explanation text.
     * @return Scale of explanation text.
     */
    float getTextScale() const;

    /**
     * @brief Get thickness of explanation text.
     * @return Thickness of explanation text.
     */
    uint8_t getTextThickness() const;

    /**
     * @brief Get maximum iterations in calculation of optimal values.
     * @return Maximum iterations in calculation of optimal values.
     */
    uint16_t getMaxIterationOptimalValues() const;

    /**
     * @brief Get maximum iterations in calculation of error factors.
     * @return Maximum iterations in calculation of error factors.
     */
    uint16_t getMaxIterationFactors() const;

    /**
     * @brief Get minimal false positives in calculation needed to stop.
     * @return Minimal false positives in calculation needed to stop.
     */
    double getMinErrorOptimalValues() const;

    /**
     * @brief Get minimal false positives in calculation needed to stop.
     * @return Minimal false positives in calculation needed to stop.
     */
    double getMinErrorFactors() const;

    /**
    * @brief Get minimal correction border needed to stop calculation of optimal values.
    * @return Minimal correction border needed to stop calculation of optimal values.
    */
    double getMinCorrectionOptimalValues() const;

    /**
    * @brief Get minimal correction border needed to stop calculation of factors.
    * @return Minimal correction border needed to stop calculation of factors.
    */
    double getMinCorrectionFactors() const;

    /**
    * @brief Get start index of positives in matrix of input points.
    * @return Start index of positives in matrix of input points.
    */
    uint64_t getStartIdxPositives() const;

    /**
     * @brief Get distance of negative colors picked in image.
     * @return Distance of negative colors picked in image.
     */
    uint8_t getNegDist() const;

    /**
     * @brief Get distance of positive colors picked in image.
     * @return Distance of positive colors picked in image.
     */
    uint8_t getPosDist() const;

 private:
    /**
     * @brief Draw rectangle to get roi.
     * @param description String to describe goal.
     * @param img Image to draw rectangle.
     * @return Returns false for abort.
     */
    bool drawRectangle(std::string description, cv::Mat *img);

    /**
     * @brief Find min. and max. x- and y-values of a rectangle.
     * @param square Rectangle representation as two points.
     * @param smallestX Smallest searched x-value.
     * @param biggestX Biggest searched y-value.
     * @param smallestY Smallest searched x-value.
     * @param biggestY Biggest searched y-value.
     */
    void findMinMaxXY(
        const std::vector<cv::Point> &square,
        uint16_t *smallestX,
        uint16_t *biggestX,
        uint16_t *smallestY,
        uint16_t *biggestY);

    /**
     * @brief Calculate probability to be searched color.
     * @param err2 Squared error used to calculate probability.
     * @return Calculated probability of being searched color.
     */
    double getProbability(const double squaredError);

    /**
     * @brief Predict pixel being bright or dark searched color. Use higher value.
     * @param row Row of pixel.
     * @param col Column of pixel.
     * @return Better pobability of being color.
     */
    double getPrediction(const uint16_t &row, const uint16_t &col);

    /**
     * @brief Calculate values to predict being searched color or not using eigens BDCSVD.
     */
    void calculate();

    /**
    * @brief Calculate factors to decide the estimated propability of a correct color.
    * @return True, if no error occured.
    */
    bool calculateFactors();

    /**
     * @brief Visualization of predicted result and user decision to accept new values
     *        or decline.
     * @return User decision, if values should be used or older values are kept.
     */
    bool visualizeResult(cv::Mat *const img);

    /**
     * @brief Calculate number of false positives.
     * @param startGoodValues End of calculation. Only false positives are interesting.
     * @return Number of false positives.
     */
    uint64_t getFalsePositives(const uint64_t &startGoodValues);

    /**
     * @brief Fill bright and dark matrices with bad values.
     * @param hsvCols Number of cols in image.
     * @param row Number of row of pixel.
     * @param col Number of column of pixel.
     * @param counter Index in matrices to store pixel values.
     */
    void fillMatricesWithBadValues(
        const uint16_t &hsvCols,
        const uint16_t &row,
        const uint16_t &col,
        const uint64_t &pos);

    /**
     * @brief Add a point in image in bad color and increase false positives.
     * @param row Row of point in image.
     * @param col Column of point in image.
     */
    void addPointInImg(const uint16_t row, const uint16_t col);

    /**
     * @brief Function to handle mouse callback in image.
     * @param event Event of mouse.
     * @param x X-coordinate in image.
     * @param y Y-coordinate in image.
     */
    void clickAndCrop(int event, int x, int y);

    /**
     * @brief OpenCV can not handle a member function. This is a hack around to use member
     *        variables in mouse callback. Needed to use own clickAndCrop.
     * @param event Mouse event.
     * @param x X-coordinate in image.
     * @param y Y-coordinate in image.
     * @param flags Keys used.
     * @param userdata Additional data given in setMouseCallback.
     */
    static void clickAndCrop(
        int event,
        int x,
        int y,
        int flags,
        void *userdata);

    // Used for clickAndCrop
    std::vector<cv::Point> squarePoints;

    const int16_t HSV_CHANNELS = 3;
    const int16_t KEY_ACCEPT = 13;
    const int16_t KEY_S = 115;
    const int16_t KEY_R = 114;
    const int16_t KEY_D = 100;
    const int16_t KEY_ESC = 27;
    const uint8_t POINTS_OF_RECTANGLE = 2;
    const double START_VALUE_FACTORS = 1e-3;
    const uint8_t WAIT_TIME = 20;

    cv::Mat imgCopy;
    std::string title;
    uint16_t distanceText2Border;
    uint8_t font;
    float textScale;
    cv::Scalar textColor;
    uint8_t textThickness;
    bool drawActive;

    uint8_t negDist;
    uint8_t posDist;

    cv::Scalar badColor;
    cv::Scalar goodColor;

    uint8_t *hsvPtr;
    Matrix8u hsvValues;
    Eigen::VectorXd results;
    std::vector<Matrix8u> allGoodValues;

    uint64_t startIdxPositives;

    double minErrorFactors;
    double minCorrectionFactors;
    uint16_t maxIterationFactors;

    Eigen::Vector3d optimalValuesIntern;
    Eigen::Vector3d errorFactorsIntern;
};

#endif  // CODE_INCLUDE_CALIBRATIONHANDLER_H_
