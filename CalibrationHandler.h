#ifndef CALIBRATOR_H
#define CALIBRATOR_H


#include <opencv2/opencv.hpp>
#include <math.h>
#include <time.h>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "ParticleWeighting.h"


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
     * @brief Calibrates params. Values without error are found using median,
     * factors to calculate prediction are calculated using eigens BDCSVD.
     * @param img Image used to visualize steps and to get values for calibration.
     * @param hsvBright Values to decide, whether a color is inside 
     *                  bright color spectrum or not.
     * @param factorsBright Factors to calculate prediction of dark color spectrum.
     * @param hsvDark Values to decide, whether a color is inside 
     *                bright color spectrum or not.
     * @param factorsDark Factors to calculate prediction of dark color spectrum.
     */
    void calibrate(cv::Mat img,
                   std::vector<uint16_t> &hsvBright, std::vector<double> &factorsBright,
                   std::vector<uint16_t> &hsvDark, std::vector<double> &factorsDark);

private:
    /**
     * @brief Calculate median (nthelement) of values inside vector.
     * @param Vector to calculate median.
     * @return Calculated median.
     */
    int median(Eigen::VectorXi &v);
    
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
     * @param factor Factor to strech function.
     * @return Calculated probability of being searched color.
     */
    double getProbability(const double &err2,const double &factor);
    
    /**
     * @brief Predict pixel being bright or dark searched color. Use higher value.
     * @param row Row of pixel.
     * @param col Column of pixel.
     * @return Better pobability of being color.
     */
    double getPrediction(const uint16_t &row, const uint16_t &col);
    
    /**
     * @brief Calculation of bright and dark color values using median.
     * @param offset Offset of allGoodValues to only get bright or dark values.
     * @param numElements Number of elements in bright or dark. Darkest and 
     *                    brightest positive values are not used in any calculation.
     */
    void getMedianValues(const uint32_t &offset, const uint32_t &numElements);
    
    /**
     * @brief Calculate factors to predict being searched color or not using eigens BDCSVD.
     *        Result is returned, if number of false positives is lower than bound or 
     *        max. iterations are reached.
     * @param hsvBrightOrDark Hsv values of bright or dark color.
     * @param x0 Calculated factors to create min. squared error with initialized start values.
     * @param hsvValues Values used to calculate errors.
     * @param results Expected results of equation A*x=l.
     * @param description Text of bright or dark calculation.
     * @param startGoodValues Index of begin of good values.
     */
    void calculate(const Eigen::Vector3i &hsvBrightOrDark, Eigen::Vector3d &x0,
                   const Matrix8u &hsvValues, const Eigen::VectorXd &results,
                   const std::string &description, const uint64_t &startGoodValues);
    
    /**
     * @brief Visualization of predicted result and user decision to accept new values
     *        or decline.
     * @return User decision, if values should be used or older values are kept.
     */
    bool visualizeResult();
    
    /**
     * @brief Calculate number of false positives. 
     * @param hsv Ideal color.
     * @param factors Factors multiplied by errors.
     * @param hsvValues Matrix containing values for calculation.
     * @param startGoodValues End of calculation. Only false positives are interesting.
     * @return 
     */
    uint64_t getFalsePositives(const Eigen::Vector3i &hsv,const Eigen::Vector3d &factors,
                      const Matrix8u &hsvValues,const uint64_t &startGoodValues);
    void fillAllBadMatrices(const uint16_t &hsvCols, const uint16_t &row,
                            const uint16_t &col, uint64_t &counter);
    void addNegativePoint(const uint16_t &row, const uint16_t &col, uint64_t &falsePositive);
    void clickAndCrop(int event, int x, int y);
    static void clickAndCrop(int event, int x, int y, // Needed to use own clickAndCrop.
                               int flags, void *userdata);
    
    
    std::vector<cv::Point> square_points;

    uint8_t hsvChannels;

    cv::Mat imgCopy;
    std::string title_;
    uint16_t distanceText2Border_;
    uint8_t font_;
    float textScale_;
    cv::Scalar textColor_;
    uint8_t textThickness_;

    uint8_t negDist = 2;
    uint8_t posDist = 1;

    cv::Scalar badColor;
    cv::Scalar goodColor;

    uint8_t *hsvPtr;
    Matrix8u hsvValuesBright;
    Matrix8u hsvValuesDark;
    Eigen::VectorXd results;
    std::vector<Matrix8u> allGoodValues;

    double startValueFactors;
    double minError;
    uint16_t maxIteration;

    Eigen::Vector3i hsvBright_;
    Eigen::Vector3i hsvDark_;
    Eigen::Vector3d factorsBright_;
    Eigen::Vector3d factorsDark_;


};

#endif //CALIBRATOR_H
