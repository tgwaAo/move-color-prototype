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

class Calibrator
{
private:
    ParticleWeighting weighting;
    std::vector<cv::Point> square_points;
    cv::Mat mirror;
    cv::Scalar textColor(255,255,0);
    uint8_t font = cv::FONT_HERSHEY_SIMPLEX;
    uint16_t distanceText2Border = 10;
    uint8_t textThickness = 2;
    uint8_t NEG_DIST = 2; // for 480*640
    uint8_t POS_DIST = 1;
    Matrix8u hsvValuesBright; // (numAllBrightAndDark,3); // better heap
    Matrix8u hsvValuesDark; //(numAllBrightAndDark,3); // better heap
    cv::Scalar badColor(0,0,255);
    cv::Scalar goodColorBright(255,0,0);
    cv::Scalar goodColorDark(0,255,0);
    uint8_t *hsv_ptr;
    std::vector<Matrix8u> allGoodValues;
    double START_VALUE_FACTORS = 0.0005;
    double MIN_ERROR = 50;
    uint16_t MAX_ITERATION = 100;
    std::string &title

    const uint8_t hsvChannels = 3; // hsv is always 3!


public:
    Calibrator();
    Calibrator(const ParticleWeighting &pw);
    ~Calibrator();

    void calibrate(cv::VideoCapture &cap, const std::string &TITLE,
                   const double &WIDTH, const double &HEIGHT,const std::string &settings_filename,
                   std::vector<uint16_t> &hsvBright, std::vector<double> &factorsBright,
                   std::vector<uint16_t> &hsvDark, std::vector<double> &factorsDark);
    int median(Eigen::VectorXi &v);
    void click_and_crop(int event, int x, int y,
                        int flags, void *userdata);
    void fill_matrix(Matrix8u &mx,const uint8_t &hsvChannels, const int &hsvCols, uint8_t *hsv_ptr,
                     const int &row, const int &col, uint64_t &counter);
    void find_minmax_xy(const std::vector<cv::Point> &square,
                        uint16_t &smallest_x, uint16_t &biggest_x, uint16_t &smallest_y, uint16_t &biggest_y);
    double get_trust(const double &v,const double &factor);
    double getPrediction(std::vector<uint16_t> &hsvBrightOrDark,
                         std::vector<double> &factors, const uint8_t *hsv_ptr,
                         const uint16_t &row, const uint16_t &col,
                         const uint8_t &channels, const int &hsv_cols);
    bool compare_head(const Matrix8u& lhs, const Matrix8u& rhs);
    void getMedianValues(std::vector<uint16_t> &hsvBrightOrDark,
                         const Matrix8u &hsvValues,const uint64_t &startIdx,const uint64_t &endIdx);
    void preparedCalibration(const std::vector<uint16_t> &hsvBrightOrDark,
                             std::vector<double> &factorsBrightOrDark,uint16_t &iter,
                             const Matrix8u &hsvValues, const double &factorsStart,
                             const Eigen::VectorXd &results,const double &minErrorBound,
                             const uint16_t &maxIteration,const uint64_t &startGoodValues);
    bool visualize(cv::Mat &img, const std::string title,const cv::Scalar &badColor,
                   const cv::Scalar &goodColor,std::vector<uint16_t> &hsv,
                   std::vector<double> &factors,const uint8_t *hsv_ptr,
                   std::vector<cv::Point> squareAllGoodValues,
                   std::vector<cv::Point> squareOnlyGoodValues,std::string textBrightOrDark);
    void visualizePoint(cv::Mat &img,const double &prediction,const double &bound,const int &row,const int &col,
                        const cv::Scalar &badColor,const cv::Scalar &goodColor,uint64_t &falsePositive);
    uint64_t getError(const std::vector<uint16_t> &hsv,const std::vector<double> &factors,
                      const Matrix8u &hsvValues,const uint64_t &startGoodValues);
    void fillAllBadMatrices();
};

#endif
