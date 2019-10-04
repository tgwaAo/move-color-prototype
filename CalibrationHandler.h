#ifndef CALIBRATOR_H
#define CALIBRATOR_H

/**
* Take picture after a short time,
* drag and drop over it and calibrate using
* Eigens BDCSVD.
*/

#include <opencv2/opencv.hpp>
#include <math.h>
#include <time.h>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "ParticleWeighting.h"

typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix8u;

class CalibrationHandler
{
private:
    std::vector<cv::Point> square_points;

    const uint8_t hsvChannels = 3; // hsv is always 3!
    cv::Mat mirror;
    std::string &title_;
    uint16_t distanceText2Border_ = 10;
    uint8_t font_ = cv::FONT_HERSHEY_SIMPLEX;
    float textScale_ = 1.2;
    cv::Scalar textColor_(255,255,0);
    uint8_t textThickness_ = 2;

    uint8_t NEG_DIST = 2; // for 480*640
    uint8_t POS_DIST = 1;

    cv::Scalar badColor(0,0,255);
    cv::Scalar goodColorBright(255,0,0);
    cv::Scalar goodColorDark(0,255,0);

    uint8_t *hsv_ptr;
    Matrix8u hsvValuesBright; // (numAllBrightAndDark,3); // better heap
    Matrix8u hsvValuesDark; //(numAllBrightAndDark,3); // better heap
    std::vector<Matrix8u> allGoodValues;

    double START_VALUE_FACTORS = 0.0005;
    double MIN_ERROR = 50;
    uint16_t MAX_ITERATION = 100;

    Eigen::Vector3i hsvBrightIntern;
    Eigen::Vector3i hsvDarkIntern;
    Eigen::Vector3d factorsBrightIntern;
    Eigen::Vector3d factorsDarkIntern;


public:
    CalibrationHandler();
    ~CalibrationHandler();

    void calibrate(cv::VideoCapture &cap, const std::string &TITLE,
                   const double &WIDTH, const double &HEIGHT,const std::string &settings_filename,
                   std::vector<uint16_t> &hsvBright, std::vector<double> &factorsBright,
                   std::vector<uint16_t> &hsvDark, std::vector<double> &factorsDark);
    int median(Eigen::VectorXi &v);
    void click_and_crop(int event, int x, int y,
                        int flags, void *userdata);
    void find_minmax_xy(const std::vector<cv::Point> &square,
                        uint16_t &smallest_x, uint16_t &biggest_x, uint16_t &smallest_y, uint16_t &biggest_y);
    double get_trust(const double &v,const double &factor);
    double getPrediction(std::vector<uint16_t> &hsvBrightOrDark,
                         std::vector<double> &factors, const uint8_t *hsv_ptr,
                         const uint16_t &row, const uint16_t &col,
                         const uint8_t &channels, const int &hsv_cols);
    bool compareBrightness(const Matrix8u& lhs, const Matrix8u& rhs);
    void getMedianValues(Eigen::Vector3i &hsvBrightOrDark, const Matrix8u &hsvValues,
                         const uint64_t &startIdx,const uint64_t &endIdx);
    void calculate(Eigen::Vector3d &x0,
                   std::vector<double> &factorsBrightOrDark,std::string brightOrDarkText,
                   const Matrix8u &hsvValues, const double &factorsStart,
                   const Eigen::VectorXd &results,const double &minErrorBound,
                   const uint16_t &maxIteration,const uint64_t &startGoodValues);
    bool visualizeResult();
    uint64_t getError(const std::vector<uint16_t> &hsv,const std::vector<double> &factors,
                      const Matrix8u &hsvValues,const uint64_t &startGoodValues);
    void fillAllBadMatrices(const uint16_t &hsvCols, const uint16_t &row,
                            const uint16_t &col, uint64_t &counter);
    void addNegativePoint(const uint16_t &row, const uint16_t &col, uint64_t &falsePositive);
};

#endif
