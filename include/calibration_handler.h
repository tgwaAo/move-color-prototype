// Modified MIT License
//
// Copyright (c) 2019 tgwa_ao
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

#include <math.h>
#include <time.h>

#include <eigen3/Eigen/Dense>

#include <fstream>
#include <random>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <opencv2/opencv.hpp>

namespace mc {
/**
 * @brief Matrix8u Save space with own matrix definition.
 */
typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    Matrix8u;

/**
 * @class CalibrationHandler
 * @date 08/10/19
 * @file Calibration_handler.h
 * @brief Class to handle calibration. Marked areas of an image are used to
 * calibrate values with eigens BDCSVD.
 */
class CalibrationHandler {
  public:
    /**
     * @brief Constructor with a variable list of possible parameters.
     * @param title Title of windows shown.
     * @param distance_text_2_Border Distance to left and lower border of
     * window.
     * @param font Font of description text.
     * @param text_scale Scale of description text.
     * @param text_color Color of description text.
     * @param text_thickness Thickness of description text.
     */
    CalibrationHandler(std::string title = "Calibration",
                       uint16_t distance_text_2_Border = 10,
                       uint8_t font = cv::FONT_HERSHEY_SIMPLEX,
                       float text_scale = 1.2,
                       cv::Scalar text_color = cv::Scalar(255, 255, 0),
                       uint8_t text_thickness = 2);

    /**
     * @brief Destoying object.
     */
    ~CalibrationHandler();

    /**
     * @brief Calibrates params. Values without error are found using
     * histograms, factors for predictions are calculated using eigens BDCSVD.
     * @param img Image used to visualize steps and to get values for
     * calibration.
     * @param optimal_values_intern Internal saved calculated optimal values.
     * @param error_factors_intern Factors to calculate prediction of dark color
     * spectrum.
     * @return Acception of new values.
     */
    bool calibrate(cv::Mat *img,
                   std::vector<double> *const optimal_values_intern,
                   std::vector<double> *const error_factors_intern);

    /**
     * @brief Set title of shown window.
     * @param title Title of shown window.
     */
    void set_title(const std::string &title);

    /**
     * @brief Set font of explanation text.
     * @param font Font of explanation text.
     */
    void set_font(const uint8_t font);

    /**
     * @brief Set color of explanation text.
     * @param text_color Color of explanation text.
     */
    void set_text_color(const cv::Scalar text_color);

    /**
     * @brief Set scale of explanation text.
     * @param text_scale Scale of explanation text.
     */
    void set_text_scale(float text_scale);

    /**
     * @brief Set thickness of explanation text.
     * @param text_thickness Thickness of explanation text.
     */
    void set_text_thickness(const uint8_t text_thickness);

    /**
     * @brief Set maximum of iterations in calibration of error factors.
     * @param max_iteration_factors Max. iterations in calibration of error
     * factors.
     */
    void set_max_iteration_factors(const uint16_t max_iteration_factors);

    /**
     * @brief Set minimum of false positives in factor calibration.
     * @param min_error_factors Minimum of false positives in factor
     * calibration.
     */
    void set_min_error_factors(const double min_error_factors);

    /**
     * @brief Set minimum border to finish calibration of factors.
     * @param min_correction_factors Factors Minimum correction border to finish
     * calibration of factors.
     */
    void set_min_correction_factors(const double min_correction_factors);

    /**
     * @brief Set start index of positive values in matrix of input points.
     * @param start_idx_positives Start index of positives in matrix of input
     * points.
     */
    void set_start_idx_positives(const uint64_t start_idx_positives);

    /**
     * @brief Set distance of negative colors picked in image.
     * @param neg_dist Distance of negative colors picked in image.
     */
    void set_neg_dist(const uint8_t neg_dist);

    /**
     * @brief Set distance of positive colors picked in image.
     * @param pos_dist Distance of positive colors picked in image.
     */
    void set_pos_dist(const uint8_t pos_dist);

    /**
     * @brief Get title of shown window.
     * @return Title of shown window.
     */
    std::string get_title() const;

    /**
     * @brief Get font of shown explanation text.
     * @return Font of shown explanation text.
     */
    uint8_t get_font() const;

    /**
     * @brief Get color of explanation text.
     * @return Color of explanation text.
     */
    cv::Scalar get_text_color() const;

    /**
     * @brief Get scale of explanation text.
     * @return Scale of explanation text.
     */
    float get_text_scale() const;

    /**
     * @brief Get thickness of explanation text.
     * @return Thickness of explanation text.
     */
    uint8_t get_text_thickness() const;

    /**
     * @brief Get maximum iterations in calculation of optimal values.
     * @return Maximum iterations in calculation of optimal values.
     */
    uint16_t get_max_iteration_optimal_values() const;

    /**
     * @brief Get maximum iterations in calculation of error factors.
     * @return Maximum iterations in calculation of error factors.
     */
    uint16_t get_max_iteration_factors() const;

    /**
     * @brief Get minimal false positives in calculation needed to stop.
     * @return Minimal false positives in calculation needed to stop.
     */
    double get_min_error_optimal_values() const;

    /**
     * @brief Get minimal false positives in calculation needed to stop.
     * @return Minimal false positives in calculation needed to stop.
     */
    double get_min_error_factors() const;

    /**
     * @brief Get minimal correction border needed to stop calculation of
     * optimal values.
     * @return Minimal correction border needed to stop calculation of optimal
     * values.
     */
    double get_min_correction_optimal_values() const;

    /**
     * @brief Get minimal correction border needed to stop calculation of
     * factors.
     * @return Minimal correction border needed to stop calculation of factors.
     */
    double get_min_correction_factors() const;

    /**
     * @brief Get start index of positives in matrix of input points.
     * @return Start index of positives in matrix of input points.
     */
    uint64_t get_start_idx_positives() const;

    /**
     * @brief Get distance of negative colors picked in image.
     * @return Distance of negative colors picked in image.
     */
    uint8_t get_neg_dist() const;

    /**
     * @brief Get distance of positive colors picked in image.
     * @return Distance of positive colors picked in image.
     */
    uint8_t get_pos_dist() const;

  private:
    /**
     * @brief Draw rectangle to get roi.
     * @param description String to describe goal.
     * @param img Image to draw rectangle.
     * @return Returns false for abort.
     */
    bool draw_rectangle(std::string description, cv::Mat *img);

    /**
     * @brief Find min. and max. x- and y-values of a rectangle.
     * @return tuple Min and max values of roi.
     */
    std::tuple<uint16_t, uint16_t, uint16_t, uint16_t>
    find_min_max_xy_and_empty_square();

    /**
     * @brief Calculate probability to be searched color.
     * @param err2 Squared error used to calculate probability.
     * @return Calculated probability of being searched color.
     */
    //    double get_probability(const double squared_error);
    double get_probability(const int16_t err_hue,
                           const int16_t err_sat,
                           const int16_t err_val,
                           const double factor_hue,
                           const double factor_sat,
                           const double factor_val);

    /**
     * @brief Predict pixel being bright or dark searched color. Use higher
     * value.
     * @param row Row of pixel.
     * @param col Column of pixel.
     * @return Better pobability of being color.
     */
    double get_prediction(const uint16_t &row, const uint16_t &col);

    /**
     * @brief Calculate values to predict being searched color or not using
     * eigens BDCSVD.
     */
    void calculate();

    /**
     * @brief Calculate factors to decide the estimated propability of a correct
     * color.
     * @return True, if no error occured.
     */
    bool calculate_factors();

    /**
     * @brief Check the correction of factors.
     * @return True if the correction is small enough, else false.
     */
    bool check_correction_of_factors(const Eigen::VectorXd &dx,
                                     const bool &without_hue);

    /**
     * @brief Visualization of predicted result and user decision to accept new
     * values or decline.
     * @return User decision, if values should be used or older values are kept.
     */
    bool visualize_result(cv::Mat *const img);

    /**
     * @brief Calculate number of false positives.
     * @param start_good_values End of calculation. Only false positives are
     * interesting.
     * @return Number of false positives.
     */
    uint64_t get_false_positives(const uint64_t &start_good_values);

    /**
     * @brief Calculate number of false negatives.
     * @param start_good_values End of calculation. Only false positives are
     * interesting.
     * @return Number of false negatives.
     */
    uint64_t get_false_negatives(const uint64_t &start_good_values);

    /**
     * @brief Fill bright and dark matrices with bad values.
     * @param hsv_cols Number of cols in image.
     * @param row Number of row of pixel.
     * @param col Number of column of pixel.
     * @param counter Index in matrices to store pixel values.
     */
    void fill_matrices_with_bad_values(const uint16_t &hsv_cols,
                                       const uint16_t &row,
                                       const uint16_t &col,
                                       const uint64_t &pos);

    /**
     * @brief Add a point in image in bad color and increase false positives.
     * @param row Row of point in image.
     * @param col Column of point in image.
     */
    void add_point_in_img(const uint16_t row, const uint16_t col);

    /**
     * @brief Function to handle mouse callback in image.
     * @param event Event of mouse.
     * @param x X-coordinate in image.
     * @param y Y-coordinate in image.
     */
    void click_and_crop(int event, int x, int y);

    /**
     * @brief Open_c_v can not handle a member function. This is a hack around
     * to use member variables in mouse callback. Needed to use own
     * click_and_crop.
     * @param event Mouse event.
     * @param x X-coordinate in image.
     * @param y Y-coordinate in image.
     * @param flags Keys used.
     * @param userdata Additional data given in set_mouse_callback.
     */
    static void
    click_and_crop(int event, int x, int y, int flags, void *userdata);

    // Used for click_and_crop
    std::vector<cv::Point> square_points;

    const int16_t HSV_CHANNELS = 3;
    const int16_t KEY_ACCEPT = 13;
    const int16_t KEY_S = 115;
    const int16_t KEY_R = 114;
    const int16_t KEY_D = 100;
    const int16_t KEY_ESC = 27;
    const uint8_t POINTS_OF_RECTANGLE = 2;
    const double START_VALUE_FACTORS = 1e-3;
    const uint8_t WAIT_TIME = 20;

    cv::Mat img_copy;
    std::string title;
    uint16_t distance_text_2_Border;
    uint8_t font;
    float text_scale;
    cv::Scalar text_color;
    uint8_t text_thickness;
    bool draw_active;

    uint8_t neg_dist;
    uint8_t pos_dist;

    cv::Scalar bad_color;
    cv::Scalar good_color;

    uint8_t *hsv_ptr;
    Matrix8u hsv_values;
    Eigen::VectorXd results;
    std::vector<Matrix8u> all_good_values;

    uint64_t start_idx_positives;

    double min_error_factors;
    double min_correction_factors;
    uint16_t max_iteration_factors;

    Eigen::Vector3d optimal_values_intern;
    Eigen::Vector3d error_factors_intern;
};
} // namespace mc
#endif // CODE_INCLUDE_CALIBRATIONHANDLER_H_
