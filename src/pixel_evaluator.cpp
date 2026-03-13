#include "pixel_evaluator.h"

#include <algorithm>
#include <fstream>

using namespace mc;

PixelEvaluator::PixelEvaluator(const std::vector<double> &factors,
                               const std::vector<double> &optimal_values) {
    this->factors = factors;
    this->optimal_values = optimal_values;
}


std::vector<std::vector<uint16_t>> PixelEvaluator::search_for_pixel_with_color(const cv::Mat *img, const uint16_t &step, const uint8_t &min_neighbors, const bool aquire_false_positives) {

    std::vector<std::vector<Point>> matrix = create_prediction_matrix(img, step);
    std::vector<std::vector<uint16_t>> found_pixels;
    uint8_t neighbors;

    if (aquire_false_positives) {
        false_positives.clear();
    }

    for (size_t x_idx = 1; x_idx < matrix.size()-1; ++x_idx) {  // ignore outer border
        for (size_t y_idx = 1; y_idx < matrix[0].size()-1; ++y_idx) {  // ignore outer border
            if (matrix.at(x_idx).at(y_idx).valid) {
                neighbors = 0;  // TODO: max value it 2, but should be 8

                if (matrix.at(x_idx-1).at(y_idx-1).valid) {
                    ++neighbors;
                }

                if (matrix.at(x_idx).at(y_idx-1).valid) {
                    ++neighbors;
                }

                if (matrix.at(x_idx+1).at(y_idx-1).valid) {
                    ++neighbors;
                }

                if (matrix.at(x_idx-1).at(y_idx).valid) {
                    ++neighbors;
                }

                if (matrix.at(x_idx+1).at(y_idx).valid) {
                    ++neighbors;
                }

                if (matrix.at(x_idx-1).at(y_idx+1).valid) {
                    ++neighbors;
                }

                if (matrix.at(x_idx).at(y_idx+1).valid) {
                    ++neighbors;
                }

                if (matrix.at(x_idx+1).at(y_idx+1).valid) {
                    ++neighbors;
                }

                if (min_neighbors <= neighbors) {
                    found_pixels.push_back(std::vector<uint16_t>{matrix.at(x_idx).at(y_idx).x, matrix.at(x_idx).at(y_idx).y});
                }
                else if (aquire_false_positives) {
                    false_positives.push_back(std::vector<uint16_t>{matrix.at(x_idx).at(y_idx).x, matrix.at(x_idx).at(y_idx).y});
                }
            }
        }
    }

    return found_pixels;
}


std::vector<int16_t> PixelEvaluator::search_median(const cv::Mat *img, const uint16_t &step, const uint8_t &min_neighbors, const bool aquire_false_positives) {
    std::vector<std::vector<Point>> matrix = create_prediction_matrix(img, step);
    std::vector<uint16_t> x_values;
    std::vector<uint16_t> y_values;
    uint8_t neighbors;

    if (aquire_false_positives) {
        false_positives.clear();
    }

    for (size_t x_idx = 1; x_idx < matrix.size()-1; ++x_idx) {  // ignore outer border
        for (size_t y_idx = 1; y_idx < matrix[0].size()-1; ++y_idx) {  // ignore outer border
            if (matrix.at(x_idx).at(y_idx).valid) {
                neighbors = 0;  // TODO: max value it 2, but should be 8

                if (matrix.at(x_idx-1).at(y_idx-1).valid) {
                    ++neighbors;
                }

                if (matrix.at(x_idx).at(y_idx-1).valid) {
                    ++neighbors;
                }

                if (matrix.at(x_idx+1).at(y_idx-1).valid) {
                    ++neighbors;
                }

                if (matrix.at(x_idx-1).at(y_idx).valid) {
                    ++neighbors;
                }

                if (matrix.at(x_idx+1).at(y_idx).valid) {
                    ++neighbors;
                }

                if (matrix.at(x_idx-1).at(y_idx+1).valid) {
                    ++neighbors;
                }

                if (matrix.at(x_idx).at(y_idx+1).valid) {
                    ++neighbors;
                }

                if (matrix.at(x_idx+1).at(y_idx+1).valid) {
                    ++neighbors;
                }

                if (min_neighbors <= neighbors) {
                    x_values.push_back(matrix.at(x_idx).at(y_idx).x);
                    y_values.push_back(matrix.at(x_idx).at(y_idx).y);
                }
                else if (aquire_false_positives) {
                    false_positives.push_back(std::vector<uint16_t>{matrix.at(x_idx).at(y_idx).x, matrix.at(x_idx).at(y_idx).y});
                }
            }
        }
    }

    std::vector<int16_t> median;

    if (x_values.empty()) {
        median.push_back(-1);
        median.push_back(-1);
    }
    else {
        size_t middle = x_values.size() / 2;
        // ignore average of middle on purpose, because the picture will also be a little unprecise
        std::nth_element(x_values.begin(), x_values.begin() + middle, x_values.end());
        median.push_back(x_values[middle]);
        std::nth_element(y_values.begin(), y_values.begin() + middle, y_values.end());
        median.push_back(y_values[middle]);
    }

    return median;
}

void PixelEvaluator::set_factors(const std::vector<double> &factors) {
    this->factors = factors;
}

std::vector<double> PixelEvaluator::get_factors() const {
    return factors;
}

void PixelEvaluator::set_optimal_values(const std::vector<double> &optimal_values) {
    this->optimal_values = optimal_values;
}

std::vector<double> PixelEvaluator::get_optimal_values() const {
    return optimal_values;
}

std::vector<std::vector<uint16_t>> PixelEvaluator::get_false_positives() const {
    return false_positives;
}

std::vector<std::vector<PixelEvaluator::Point>> PixelEvaluator::create_prediction_matrix(const cv::Mat *img, const uint16_t &step) {
    int16_t err_hue;
    int16_t err_sat;
    int16_t err_val;

    double prediction;

    uint8_t *pixel_ptr = reinterpret_cast<uint8_t*>(img->data);
    const uint16_t X_ENTRIES = img->cols / step;
    const uint16_t Y_ENTRIES = img->rows / step;
    const size_t NBR_PARTICLES = X_ENTRIES * Y_ENTRIES;
    std::vector<uint16_t> particles_x(NBR_PARTICLES);
    std::vector<uint16_t> particles_y(NBR_PARTICLES);

    for (size_t i = 0; i < NBR_PARTICLES; ++i) {
        particles_x[i] = (i*step) % img->cols;
        particles_y[i] = ((i*step) / img->cols) * step;
    }

    std::vector<std::vector<Point>> matrix(
        X_ENTRIES,
        std::vector<Point>(
            Y_ENTRIES
            )
        );

    for (uint32_t idx = 0; idx < NBR_PARTICLES; ++idx) {
        err_hue =
            optimal_values[0] - pixel_ptr[
                                    particles_y.at(idx) * img->cols * img->channels()
                                    + particles_x.at(idx) * img->channels()];

        err_sat =
            optimal_values[1] - pixel_ptr[
                                    particles_y.at(idx) * img->cols * img->channels()
                                    +particles_x.at(idx) * img->channels() + 1];

        err_val =
            optimal_values[2] - pixel_ptr[
                                    particles_y.at(idx) * img->cols * img->channels()
                                    + particles_x.at(idx) * img->channels() + 2];

        prediction = 1
                     / (1
                        + err_hue * err_hue * factors.at(0)
                        + err_sat * err_sat * factors.at(1)
                        + err_val * err_val * factors.at(2));

        if (0.5 <= prediction ) {
            uint16_t x_idx = idx % X_ENTRIES;
            uint16_t y_idx = idx / X_ENTRIES;
            matrix.at(x_idx).at(y_idx).valid = true;
            matrix.at(x_idx).at(y_idx).x = particles_x.at(idx);
            matrix.at(x_idx).at(y_idx).y = particles_y.at(idx);
        }
    }

    return matrix;
}
