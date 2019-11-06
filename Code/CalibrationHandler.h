#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#define EIGEN_MPL2_ONLY

#include <opencv2/opencv.hpp>
#include <math.h>
#include <time.h>
#include <vector>
#include <eigen3/Eigen/Dense>

// Define own matrix to save space
typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix8u;

/**
 * @class CalibrationHandler
 * @author Uwe Simon
 * @date 08/10/19
 * @file CalibrationHandler.h
 * @brief Class to handle calibration. Marked areas of an image are used to calibrate values with eigens BDCSVD.
 */
class CalibrationHandler
{
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
    CalibrationHandler(std::string title = "Calibration", uint16_t distanceText2Border = 10,
                       uint8_t font = cv::FONT_HERSHEY_SIMPLEX, float textScale = 1.2,
                       cv::Scalar textColor = cv::Scalar(255,255,0), uint8_t textThickness = 2);
    /**
     * @brief Destoying object.
     */
    ~CalibrationHandler();
    
    /**
     * @brief Calibrates params. Values without error are found using histograms,
     * factors for predictions are calculated using eigens BDCSVD.
     * @param img Image used to visualize steps and to get values for calibration.
     * @param hsvColor Values to decide, whether a color is inside 
     *                  bright color spectrum or not.
     * @param factorsColor Factors to calculate prediction of dark color spectrum.
     * @return Acception of new values.
     */
    bool calibrate(cv::Mat *img,
                   std::vector<uint16_t> &hsvColor, 
                   std::vector<double> &factorsColor);

private:   
    /**
     * @brief Find min. and max. x- and y-values of a rectangle.
     * @param square Rectangle representation as two points.
     * @param smallestX Smallest searched x-value.
     * @param biggestX Biggest searched y-value.
     * @param smallestY Smallest searched x-value.
     * @param biggestY Biggest searched y-value.
     */
    void findMinMaxXY(const std::vector<cv::Point> &square,
                        uint16_t &smallestX, uint16_t &biggestX, uint16_t &smallestY, uint16_t &biggestY);
                        
    /**
     * @brief Calculate probability to be searched color.
     * @param err2 Squared error used to calculate probability.
     * @return Calculated probability of being searched color.
     */
    double getProbability(const double err2);
    
    /**
     * @brief Predict pixel being bright or dark searched color. Use higher value.
     * @param row Row of pixel.
     * @param col Column of pixel.
     * @return Better pobability of being color.
     */
    double getPrediction(const uint16_t &row, const uint16_t &col);
    
    /**
     * @brief Calculate factors to predict being searched color or not using eigens BDCSVD.
     *        Result is returned, if number of false positives is lower than bound or 
     *        max. iterations are reached.
     * @param startGoodValues Index of begin of good values.
     */
    void calculate(const uint64_t &startGoodValues);
    
    /**
     * @brief Visualization of predicted result and user decision to accept new values
     *        or decline.
     * @return User decision, if values should be used or older values are kept.
     */
    bool visualizeResult();
    
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
    void fillMatricesWithBadValues(const uint16_t &hsvCols, const uint16_t &row,
                            const uint16_t &col, uint64_t &counter);
                            
    /**
     * @brief Add a point in image in bad color and increase false positives.
     * @param row Row of point in image.
     * @param col Column of point in image.
     */
    void addPointInImg(const uint16_t &row, const uint16_t &col);
    
    /**
     * @brief Function to handle mouse callback in image.
     * @param event Event of mouse.
     * @param x X-coordinate in image.
     * @param y Y-coordinate in image.
     */
    void clickAndCrop(int event, int x, int y);
    
    /**
     * @brief OpenCV can not handle a member function. This is a hack around to use member
     *        variables in mouse callback.
     * @param event Mouse event.
     * @param x X-coordinate in image.
     * @param y Y-coordinate in image.
     * @param flags Keys used.
     * @param userdata Additional data given in setMouseCallback.
     */
    static void clickAndCrop(int event, int x, int y, // Needed to use own clickAndCrop.
                               int flags, void *userdata);
    

    std::vector<cv::Point> squarePoints; // Used for clickAndCrop

    const int16_t HSV_CHANNELS = 3;
    const int16_t KEY_ACCEPT = 13;
    const int16_t KEY_S = 115;
    const int16_t KEY_R = 114;
    const int16_t KEY_ESC = 27;
    const uint8_t POINTS_OF_RECTANGLE = 2;
    
    cv::Mat imgCopy;
    std::string title_;
    uint16_t distanceText2Border_;
    uint8_t font_;
    float textScale_;
    cv::Scalar textColor_;
    uint8_t textThickness_;
    bool drawActive;

    uint8_t negDist;
    uint8_t posDist;

    cv::Scalar badColor;
    cv::Scalar goodColor;

    uint8_t *hsvPtr;
    Matrix8u hsvValues;
    Eigen::VectorXd results;
    std::vector<Matrix8u> allGoodValues;

    double startValueFactors;
    double minError;
    uint16_t maxIteration;

    Eigen::Vector3i hsvColor_;
    Eigen::Vector3d factorsColor_;

};

#endif //CALIBRATOR_H
