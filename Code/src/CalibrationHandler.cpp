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

/**
* CalibrationHandler.cpp
*
* Take picture after a short time,
* drag and drop over it and calibrate.
*/

#include "CalibrationHandler.h"

CalibrationHandler::CalibrationHandler(
        std::string title,
        uint16_t distance_text_2_Border,
        uint8_t font,
        float text_scale,
        cv::Scalar text_color,
        uint8_t text_thickness)
        : title(title),
          distance_text_2_Border(distance_text_2_Border),
          font(font),
          text_scale(text_scale),
          text_color(text_color),
          text_thickness(text_thickness) {
    hsv_ptr = 0;

    neg_dist = 2;
    pos_dist = 1;

    bad_color = cv::Scalar(0, 0, 255);
    good_color = cv::Scalar(0, 255, 0);

    Eigen::IOFormat Short_format(4);
    set_max_iteration_factors(50);
    set_min_error_factors(80);  // false positives
    set_min_correction_factors(1e-4);
}

CalibrationHandler::~CalibrationHandler() {}

bool CalibrationHandler::calibrate(
        cv::Mat *const img,
        std::vector<double> *const hsv_color,
        std::vector<double> *const factors_color) {
    /***************************************************************
     * Let user select positives.
     * ************************************************************/
    if (!draw_rectangle("Select ONLY good color", img))
        return false;


    uint16_t good_smallest_x, good_biggest_x, good_smallest_y, good_biggest_y;
    std::tie(good_smallest_x, good_biggest_x, good_smallest_y, good_biggest_y) = find_min_max_xy_and_empty_square();

    /******************************************************************
     * Let user select non-negatives (more than all positives).
     * ***************************************************************/
    if (!draw_rectangle("Select MORE than good color!", img))
        return false;

    uint16_t smallest_x, biggest_x, smallest_y, biggest_y;
    std::tie(smallest_x, biggest_x, smallest_y, biggest_y) = find_min_max_xy_and_empty_square();

    /*************************************************************
     * Calculate sizes for preallocation
     * **********************************************************/
    img->copyTo(img_copy);
    cv::putText(
        img_copy,
        "Calibrating ...",
        cv::Point(
            distance_text_2_Border,
            img_copy.rows - distance_text_2_Border),
        font,
        text_scale,
        text_color,
        text_thickness);

    // Above square
    uint16_t num_x = ceil(img_copy.cols/ static_cast<float>(neg_dist));
    uint16_t num_y = ceil(smallest_y / static_cast<float>(neg_dist));
    uint64_t num_all = num_x * num_y;

    // Below square
    num_y = ceil((img_copy.rows - biggest_y) / static_cast<float>(neg_dist));
    num_all += num_x * num_y;

    // Left next to square
    num_x = ceil(smallest_x / static_cast<float>(neg_dist));
    num_y = ceil((biggest_y - smallest_y) / static_cast<float>(neg_dist));
    num_all += num_x * num_y;

    // Right next to square
    num_x = ceil((img_copy.cols - biggest_x) / static_cast<float>(neg_dist));
    num_all += num_x * num_y;

    // Only good values
    num_x = ceil((good_biggest_x - good_smallest_x) / static_cast<float>(pos_dist));
    num_y = ceil((good_biggest_y - good_smallest_y) / static_cast<float>(pos_dist));
    uint32_t num_all_good_values = num_x * num_y;
    uint16_t mult = num_all / (num_all_good_values);
    num_all += mult * num_all_good_values;

    /**************************************************************
     * Fill matrices with wrong colors.
     * ***********************************************************/
    hsv_values = Matrix8u(num_all, 3);
    results = Eigen::VectorXd::Zero(num_all);

    size_t counter = 0;
    double min_value = .1;
    img->copyTo(img_copy);
    cv::Mat hsv;
    cv::cvtColor(img_copy, hsv, cv::COLOR_BGR2HSV);
    hsv_ptr = reinterpret_cast<uint8_t*>(hsv.data);

    for (int col = 0; col < img_copy.cols; col += neg_dist) {
        // Above square
        for (int row = 0; row < smallest_y; row += neg_dist) {
            fill_matrices_with_bad_values(img_copy.cols, row, col, counter);
            results(counter) = min_value;
            ++counter;
        }

        // Below square
        for (int row = biggest_y; row < img_copy.rows; row += neg_dist) {
            fill_matrices_with_bad_values(img_copy.cols, row, col, counter);
            results(counter) = min_value;
            ++counter;
        }
    }

    // Left next to square
    for (int row = smallest_y; row < biggest_y; row += neg_dist) {
        for (int col = 0; col < smallest_x; col += neg_dist) {
            fill_matrices_with_bad_values(img_copy.cols, row, col, counter);
            results(counter) = min_value;
            ++counter;
        }

        // Right next to square
        for (int col = biggest_x; col < img_copy.cols; col += neg_dist) {
            fill_matrices_with_bad_values(img_copy.cols, row, col, counter);
            results(counter) = min_value;
            ++counter;
        }
    }

    /********************************************************
     * Collect positive values and create histograms.
     * *****************************************************/
    set_start_idx_positives(counter);
    Eigen::VectorXi hue_hist = Eigen::VectorXi::Zero(256);
    Eigen::VectorXi sat_hist = Eigen::VectorXi::Zero(256);
    Eigen::VectorXi val_hist = Eigen::VectorXi::Zero(256);
    std::random_device rd;
    std::mt19937 eng = std::mt19937(rd());
    std::normal_distribution<double> normal_dist(0, 1);
    for (uint32_t col = good_smallest_x; col < good_biggest_x; col += pos_dist) {
        for (uint32_t row = good_smallest_y; row < good_biggest_y; row += pos_dist) {
            line(img_copy, cv::Point(col, row), cv::Point(col, row), good_color);

            // Fill histograms.
            ++hue_hist(
                hsv_ptr[
                    row * img_copy.cols * HSV_CHANNELS
                    + col * HSV_CHANNELS]);

            ++sat_hist(
                hsv_ptr[
                    row * img_copy.cols * HSV_CHANNELS
                    + col * HSV_CHANNELS
                    + 1]);

            ++val_hist(
                hsv_ptr[
                    row * img_copy.cols * HSV_CHANNELS
                    + col * HSV_CHANNELS
                    + 2]);

            for (uint16_t m = 0; m < mult; ++m) {
                hsv_values(counter, 0) = hsv_ptr[
                                            row * img_copy.cols * HSV_CHANNELS
                                            + col * HSV_CHANNELS]
                        + normal_dist(eng);

                hsv_values(counter, 1) = hsv_ptr[
                                            row * img_copy.cols * HSV_CHANNELS
                                            + col * HSV_CHANNELS
                                            + 1]
                        + normal_dist(eng);

                hsv_values(counter, 2) = hsv_ptr[
                                            row * img_copy.cols * HSV_CHANNELS
                                            + col * HSV_CHANNELS
                                            + 2]
                        + normal_dist(eng);

                results(counter) = .99;
                ++counter;
            }
        }
    }

    cv::imshow(title, img_copy);
    int16_t key = cv::waitKey(0);

    if (key == KEY_ESC) {
        return false;
    }

    cv::putText(
        img_copy,
        "Calculating ...",
        cv::Point(distance_text_2_Border, img_copy.rows - distance_text_2_Border),
        font,
        text_scale,
        text_color,
        text_thickness);
    cv::imshow(title, img_copy);
    cv::waitKey(WAIT_TIME);

    /**********************************************************
     * Find optimal values using histograms.
     * *******************************************************/
    int max_hue = 0;
    int max_sat = 0;
    int max_val = 0;

    for (int i = 0; i < hue_hist.size(); ++i) {
        if (max_hue < hue_hist(i)) {
            optimal_values_intern(0) = i;
            max_hue = hue_hist(i);
        }

        if (max_sat < sat_hist(i)) {
            optimal_values_intern(1) = i;
            max_sat = sat_hist(i);
        }

        if (max_val < val_hist(i)) {
            optimal_values_intern(2) = i;
            max_val = val_hist(i);
        }
    }

    std::cout
            << "optimal start values in hsv = "
            << optimal_values_intern(0)
            << " "
            << optimal_values_intern(1)
            << " "
            << optimal_values_intern(2)
            << std::endl;

    /*************************************************************
     * Start calibration for factors (tolerances) of colors
     * **********************************************************/
    std::cout << "starting calibration" << std::endl;

    calculate();
    std::cout
            << "factors for bright = "
            << std::endl
            << error_factors_intern(0)
            << " "
            << error_factors_intern(1)
            << " "
            << error_factors_intern(2)
            << std::endl;

    /***************************************************************
     * Visualisation of bright color search and optional save.
     * ************************************************************/
    img->copyTo(img_copy);
    bool accept_values =  visualize_result(img);

    if (accept_values) {
        (*hsv_color)[0] = optimal_values_intern(0);
        (*hsv_color)[1] = optimal_values_intern(1);
        (*hsv_color)[2] = optimal_values_intern(2);
        (*factors_color)[0] = error_factors_intern[0];
        (*factors_color)[1] = error_factors_intern[1];
        (*factors_color)[2] = error_factors_intern[2];
    }

    *hsv_ptr = 0;
    return accept_values;
}

std::tuple<uint16_t, uint16_t, uint16_t, uint16_t> CalibrationHandler::find_min_max_xy_and_empty_square() {
    uint16_t smallest_x, biggest_x, smallest_y, biggest_y;

    if (square_points[0].x < square_points[1].x) {
        smallest_x = square_points[0].x;
        biggest_x = square_points[1].x;
    } else {
        smallest_x = square_points[1].x;
        biggest_x = square_points[0].x;
    }

    if (square_points[0].y < square_points[1].y) {
        smallest_y = square_points[0].y;
        biggest_y = square_points[1].y;
    } else {
        smallest_y = square_points[1].y;
        biggest_y = square_points[0].y;
    }
    square_points.pop_back();
    square_points.pop_back();
    return std::make_tuple(smallest_x, biggest_x, smallest_y, biggest_y);
}

double CalibrationHandler::get_probability(
        const int16_t err_hue,
        const int16_t err_sat,
        const int16_t err_val,
        const double factor_hue,
        const double factor_sat,
        const double factor_val) {
    if (factor_hue == 0)
        return (1 / (1
                 + (pow(err_sat, 2) * factor_sat
                 + pow(err_val, 2) * factor_val)));
    else
        return (1 / (1
                 + (pow(err_hue, 2) * factor_hue
                 + pow(err_sat, 2) * factor_sat
                 + pow(err_val, 2) * factor_val)));
}


double CalibrationHandler::get_prediction(
        const uint16_t &row,
        const uint16_t &col) {
    if (hsv_ptr != 0) {
        int16_t err_hue = optimal_values_intern(0) - hsv_ptr[
                      row * img_copy.cols * HSV_CHANNELS
                      + col * HSV_CHANNELS];
        int16_t err_sat = optimal_values_intern(1) - hsv_ptr[
                      row * img_copy.cols * HSV_CHANNELS
                      + col * HSV_CHANNELS
                      + 1];
        int16_t err_val = optimal_values_intern(2) - hsv_ptr[
                      row * img_copy.cols * HSV_CHANNELS
                      + col * HSV_CHANNELS
                      + 2];

        return get_probability(
            err_hue,
            err_sat,
            err_val,
            error_factors_intern(0),
            error_factors_intern(1),
            error_factors_intern(2));
    } else {
        return -1;
    }
}

void CalibrationHandler::calculate() {
    if (!calculate_factors()) {
        std::cout << "could not calculate factors" << std::endl;
    }
}

bool CalibrationHandler::calculate_factors() {
    Eigen::IOFormat Short_format(4);

    double err_hue;
    double err_sat;
    double err_val;

    double factor_hue;
    double factor_sat;
    double factor_val;

    double err_sum_hue = 0;
    double err_sum_sat = 0;
    double err_sum_val = 0;

    for (int i = get_start_idx_positives(); i < hsv_values.rows(); ++i) {
        err_hue = optimal_values_intern(0) - hsv_values(i, 0);  // x-a
        err_sat = optimal_values_intern(1) - hsv_values(i, 1);  // y-b
        err_val = optimal_values_intern(2) - hsv_values(i, 2);  // z-c
        err_sum_hue += pow(err_hue, 2);
        err_sum_sat += pow(err_sat, 2);
        err_sum_val += pow(err_val, 2);
    }

    bool without_hue = false;
    uint8_t used_entries;

    if (optimal_values_intern(1) < 30) {
        without_hue = true;
        factor_hue = 0;
        used_entries = 2;

    } else {
        factor_hue = .001;
        used_entries = 3;
    }

    factor_sat = .001;
    factor_val = .001;

    error_factors_intern = {
        factor_hue,
        factor_sat,
        factor_val
    };

    Eigen::VectorXd dx(used_entries);
    Eigen::MatrixXd A(hsv_values.rows(), used_entries);
    Eigen::BDCSVD<Eigen::MatrixXd> svd_solver(hsv_values.rows(), used_entries,
        Eigen::ComputeThinU|Eigen::ComputeThinV);

    Eigen::VectorXd error_results(results.rows());

    double calculated_result;
    double sum_denominator;
    double squared_sum_denominator;
    uint64_t number_false_positives;

    uint16_t iterations = 0;

    while (iterations < get_max_iteration_factors()) {
        // derivative(1/(1+(u*(x-a)^2+v*(y-b)^2+w*(z-c)^2)), u)
        // = -1 * (x-a)^2 / (u*(x-a)^2 + v*(y-b)^2 + w*(z-c)^2 + 1)^2
        if (without_hue) {
            for (int i = 0; i < hsv_values.rows(); ++i) {
                err_sat = optimal_values_intern(1) - hsv_values(i, 1);  // y-b
                err_val = optimal_values_intern(2) - hsv_values(i, 2);  // z-c

                sum_denominator = 1
                                 + pow(err_sat, 2) * error_factors_intern(1)
                                 + pow(err_val, 2) * error_factors_intern(2);
                squared_sum_denominator = pow(sum_denominator, 2);

                A(i, 0) = -pow(err_sat, 2) / squared_sum_denominator;
                A(i, 1) = -pow(err_val, 2) / squared_sum_denominator;
                calculated_result = get_probability(
                    0,
                    err_sat,
                    err_val,
                    error_factors_intern(0),
                    error_factors_intern(1),
                    error_factors_intern(2));
                error_results(i) = results(i) - calculated_result;
            }
        } else {
            for (int i = 0; i < hsv_values.rows(); ++i) {
                err_hue = optimal_values_intern(0) - hsv_values(i, 0);  // x-a
                err_sat = optimal_values_intern(1) - hsv_values(i, 1);  // y-b
                err_val = optimal_values_intern(2) - hsv_values(i, 2);  // z-c

                sum_denominator = 1
                                 + pow(err_hue, 2) * error_factors_intern(0)
                                 + pow(err_sat, 2) * error_factors_intern(1)
                                 + pow(err_val, 2) * error_factors_intern(2);
                squared_sum_denominator = pow(sum_denominator, 2);

                A(i, 0) = -pow(err_hue, 2) / squared_sum_denominator;
                A(i, 1) = -pow(err_sat, 2) / squared_sum_denominator;
                A(i, 2) = -pow(err_val, 2) / squared_sum_denominator;
                calculated_result = get_probability(
                    err_hue,
                    err_sat,
                    err_val,
                    error_factors_intern(0),
                    error_factors_intern(1),
                    error_factors_intern(2));
                error_results(i) = results(i) - calculated_result;
            }
        }
        svd_solver.compute(A);

        if (svd_solver.info() != Eigen::Success) {
            std::cout << "could not compute solver" << std::endl;
            return false;
        }

        dx = svd_solver.solve(error_results);

        if (svd_solver.info() != Eigen::Success) {
            std::cout << "could not solve computation" << std::endl;
            return false;
        }

        if (without_hue) {
            error_factors_intern(1) += dx(0);
            error_factors_intern(2) += dx(1);
        } else {
            error_factors_intern += dx;
        }
        for (uint8_t pos = 0; pos < used_entries; ++pos)
            if (dx(pos) < 0) dx(pos) *= -1;

        ++iterations;

        if (check_correction_of_factors(dx, without_hue)) {
            if (get_min_error_factors() != -1) {
                number_false_positives = get_false_positives(
                            get_start_idx_positives());

                if (number_false_positives < get_min_error_factors()) {
                    break;
                }
            } else {
                break;
            }
        }
    }

    std::cout
            << "used iterations for calculation of error factors = "
            << unsigned(iterations)
            << std::endl;
    return true;
}


bool CalibrationHandler::check_correction_of_factors(
        const Eigen::VectorXd& dx, const bool& without_hue) {
    if (dx(0) < get_min_correction_factors() &&
            dx(1) < get_min_correction_factors()) {
        if (without_hue || dx(2) < get_min_correction_factors()) {
            return true;
        }
    }
    return false;
}


bool CalibrationHandler::visualize_result(cv::Mat *const img) {
    for (uint16_t col = 0; col < img_copy.cols; col += neg_dist) {
        // Above
        for (uint16_t row = 0; row < square_points[0].y; row += neg_dist) {
            add_point_in_img(row, col);
        }

        // Below
        for (int row = square_points[1].y; row < img_copy.rows; row += neg_dist) {
            add_point_in_img(row, col);
        }
    }

    // Left
    for (int row = square_points[0].y; row < square_points[1].y; row += neg_dist) {
        for (int col = 0; col < square_points[0].x; col += neg_dist) {
            add_point_in_img(row, col);
        }

        // Right
        for (int col = square_points[1].x; col < img_copy.cols; col += neg_dist) {
            add_point_in_img(row, col);
        }
    }

    // Good and more values
    double prediction;

    for (
        int col = square_points[0].x;
        col < square_points[1].x;
        col += pos_dist) {
        for (
            int row = square_points[0].y;
            row < square_points[1].y;
            row += pos_dist) {
            prediction = get_prediction(row, col);

            if (prediction < 0.5)
                line(
                    img_copy,
                    cv::Point(col, row),
                    cv::Point(col, row),
                    bad_color);
            else
                line(
                    img_copy,
                    cv::Point(col, row),
                    cv::Point(col, row),
                    good_color);
        }
    }

    cv::imshow(title, img_copy);

    int key = cv::waitKey(0);

    if (key == KEY_D) {
        cv::imwrite("debug_image.jpg", *img);
        std::ofstream save_file;
        save_file.open("debug_hsv.txt");

        save_file << unsigned(optimal_values_intern(0)) << std::endl;
        save_file << unsigned(optimal_values_intern(1)) << std::endl;
        save_file << unsigned(optimal_values_intern(2)) << std::endl;
        save_file << error_factors_intern(0) << std::endl;
        save_file << error_factors_intern(1) << std::endl;
        save_file << error_factors_intern(2) << std::endl;

        save_file.close();
    } else if (key == KEY_S) {
        return true;
    } else if (key == KEY_ESC) {
        return false;
    }
    return false;
}


uint64_t CalibrationHandler::get_false_positives(
        const uint64_t &start_good_values) {
    uint64_t false_positives = 0;
    double prediction;
    int16_t err_hue;
    int16_t err_sat;
    int16_t err_val;

    for (uint64_t i = 0; i < start_good_values; ++i) {
        err_hue = optimal_values_intern(0) - hsv_values(i, 0);
        err_sat = optimal_values_intern(1) - hsv_values(i, 1);
        err_val = optimal_values_intern(2) - hsv_values(i, 2);

        prediction = get_probability(
                    err_hue,
                    err_sat,
                    err_val,
                    error_factors_intern(0),
                    error_factors_intern(1),
                    error_factors_intern(2));
        if (prediction >= 0.5) {
            ++false_positives;
        }
    }

    return false_positives;
}

uint64_t CalibrationHandler::get_false_negatives(
        const uint64_t &start_good_values) {
    uint64_t false_negatives = 0;
    double prediction;
    int16_t err_hue;
    int16_t err_sat;
    int16_t err_val;

    for (uint64_t i = start_good_values; i < hsv_values.rows(); ++i) {
        err_hue = optimal_values_intern(0) - hsv_values(i, 0);
        err_sat = optimal_values_intern(1) - hsv_values(i, 1);
        err_val = optimal_values_intern(2) - hsv_values(i, 2);

        prediction = get_probability(
                    err_hue,
                    err_sat,
                    err_val,
                    error_factors_intern(0),
                    error_factors_intern(1),
                    error_factors_intern(2));
        if (prediction < 0.5) {
            ++false_negatives;
        }
    }

    return false_negatives;
}

void CalibrationHandler::fill_matrices_with_bad_values(
    const uint16_t &hsv_cols,
    const uint16_t &row,
    const uint16_t &col,
    const uint64_t &pos) {
    hsv_values(pos, 0) = hsv_ptr[
                            row * hsv_cols * HSV_CHANNELS
                            + col * HSV_CHANNELS];
    hsv_values(pos, 1) = hsv_ptr[
                            row * hsv_cols * HSV_CHANNELS
                            + col * HSV_CHANNELS
                            + 1];
    hsv_values(pos, 2) = hsv_ptr[
                            row * hsv_cols * HSV_CHANNELS
                            + col * HSV_CHANNELS
                            + 2];

    cv::line(
        img_copy,
        cv::Point(col, row),
        cv::Point(col, row),
        bad_color);
}

void CalibrationHandler::add_point_in_img(const uint16_t row, const uint16_t col) {
    double prediction = get_prediction(row, col);

    if (prediction < 0.5)
        line(img_copy, cv::Point(col, row), cv::Point(col, row), bad_color);
    else
        line(img_copy, cv::Point(col, row), cv::Point(col, row), good_color);
}

void CalibrationHandler::set_font(const uint8_t font) {
    this->font = font;
}

void CalibrationHandler::set_max_iteration_factors(
        const uint16_t max_iteration_factors) {
    this->max_iteration_factors = max_iteration_factors;
}

void CalibrationHandler::set_min_error_factors(const double min_error_factors) {
    this->min_error_factors = min_error_factors;
}

void CalibrationHandler::set_min_correction_factors(
        const double min_correction_factors) {
    this->min_correction_factors = min_correction_factors;
}

void CalibrationHandler::set_start_idx_positives(
        const uint64_t start_idx_positives) {
    this->start_idx_positives = start_idx_positives;
}

void CalibrationHandler::set_neg_dist(const uint8_t neg_dist) {
    this->neg_dist = neg_dist;
}

void CalibrationHandler::set_pos_dist(const uint8_t pos_dist) {
    this->pos_dist = pos_dist;
}

void CalibrationHandler::set_text_color(const cv::Scalar text_color) {
    this->text_color = text_color;
}

void CalibrationHandler::set_text_scale(float text_scale) {
    this->text_scale = text_scale;
}

void CalibrationHandler::set_text_thickness(const uint8_t text_thickness) {
    this->text_thickness = text_thickness;
}

void CalibrationHandler::set_title(const std::string &title) {
    this->title = title;
}

uint8_t CalibrationHandler::get_font() const {
    return font;
}

uint16_t CalibrationHandler::get_max_iteration_factors() const {
    return max_iteration_factors;
}

double CalibrationHandler::get_min_error_factors() const {
    return min_error_factors;
}

double CalibrationHandler::get_min_correction_factors() const {
    return min_correction_factors;
}

uint64_t CalibrationHandler::get_start_idx_positives() const {
    return start_idx_positives;
}

uint8_t CalibrationHandler::get_neg_dist() const {
    return neg_dist;
}

uint8_t CalibrationHandler::get_pos_dist() const {
    return pos_dist;
}

cv::Scalar CalibrationHandler::get_text_color() const {
    return text_color;
}

float CalibrationHandler::get_text_scale() const {
    return text_scale;
}

uint8_t CalibrationHandler::get_text_thickness() const {
    return text_thickness;
}

std::string CalibrationHandler::get_title() const {
    return title;
}

bool CalibrationHandler::draw_rectangle(
        std::string description, cv::Mat *const img) {
    img->copyTo(img_copy);
    cv::putText(
        img_copy,
        description,
        cv::Point(distance_text_2_Border, img_copy.rows-distance_text_2_Border),
        font,
        text_scale,
        text_color,
        text_thickness);

    cv::Rect2d rect = cv::selectROI(title, img_copy, false, false);
    if (rect.width == 0 || rect.height == 0) {
        return false;
    }

    square_points.push_back(cv::Point(rect.x, rect.y));
    square_points.push_back(cv::Point(rect.x + rect.width, rect.y + rect.height));
    return true;
}
