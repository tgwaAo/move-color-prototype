/**
* Take picture after a short time,
* drag and drop over it and calibrate.
*/
#include <opencv2/opencv.hpp>
#include <math.h>
#include <time.h>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "ParticleWeighting.h"
//
//using namespace cv;
//using namespace std;
//using namespace Eigen;

typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix8u;

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
