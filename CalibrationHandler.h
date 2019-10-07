/**
* Take picture after a short time,
* drag and drop over it and calibrate using
* Eigens BDCSVD.
*/


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


class CalibrationHandler
{
private:
    std::vector<cv::Point> square_points;

    uint8_t hsvChannels;

    cv::Mat mirror;
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


public:
    CalibrationHandler(std::string title = "Calibration", uint16_t distanceText2Border = 10,
                       uint8_t font = cv::FONT_HERSHEY_SIMPLEX, float textScale = 1.2,
                       cv::Scalar textColor = cv::Scalar(255,255,0), uint8_t textThickness = 2);
    ~CalibrationHandler();

    void calibrate(cv::Mat img,
                   std::vector<uint16_t> &hsvBright, std::vector<double> &factorsBright,
                   std::vector<uint16_t> &hsvDark, std::vector<double> &factorsDark);
    int median(Eigen::VectorXi &v);
    // declare local click_and_crop
    void find_minmax_xy(const std::vector<cv::Point> &square,
                        uint16_t &smallest_x, uint16_t &biggest_x, uint16_t &smallest_y, uint16_t &biggest_y);
    double get_trust(const double &v,const double &factor);
    double getPrediction(const uint16_t &row, const uint16_t &col);
    void getMedianValues(const uint64_t &startIdx,const uint64_t &endIdx);
    void calculate(const Eigen::Vector3i &hsvBrightOrDark, Eigen::Vector3d &x0,
                   const Matrix8u &hsvValues, const Eigen::VectorXd &results,
                   const std::string &description, const uint64_t &startGoodValues);
    bool visualizeResult();
    uint64_t getError(const Eigen::Vector3i &hsv,const Eigen::Vector3d &factors,
                      const Matrix8u &hsvValues,const uint64_t &startGoodValues);
    void fillAllBadMatrices(const uint16_t &hsvCols, const uint16_t &row,
                            const uint16_t &col, uint64_t &counter);
    void addNegativePoint(const uint16_t &row, const uint16_t &col, uint64_t &falsePositive);
    void clickAndCrop(int event, int x, int y);
    static void click_and_crop(int event, int x, int y,
                               int flags, void *userdata);
};

#endif
