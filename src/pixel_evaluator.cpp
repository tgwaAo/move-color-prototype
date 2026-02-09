#include "pixel_evaluator.h"

#include <algorithm>

using namespace mc;

PixelEvaluator::PixelEvaluator(const std::vector<double> &factors,
                               const std::vector<double> &optimal_values) {
    this->factors = factors;
    this->optimal_values = optimal_values;
}


std::vector<std::vector<uint16_t>> PixelEvaluator::search_for_pixel_with_color(const cv::Mat *img, const uint16_t &step) {
    int16_t err_hue;
    int16_t err_sat;
    int16_t err_val;

    double prediction;

    uint8_t *pixel_ptr = reinterpret_cast<uint8_t*>(img->data);
    const uint8_t CHANNELS = img->channels();
    const uint16_t COLS = img->cols;
    const size_t NBR_PIXELS = img->rows * img->cols;
    const size_t NBR_PARTICLES = NBR_PIXELS / step;

    std::vector<std::vector<uint16_t>> found_pixels;
    std::vector<uint16_t> particles_x(NBR_PARTICLES);
    std::vector<uint16_t> particles_y(NBR_PARTICLES);

    for (size_t i = 0; i < NBR_PARTICLES; ++i) {
        particles_x[i] = (i*step) % 640;
        particles_y[i] = (i*step) / 640;
    }


    for (uint32_t idx = 0; idx < particles_x.size(); ++idx) {
        err_hue =
            optimal_values[0] - pixel_ptr[
                      particles_y[idx] * COLS * CHANNELS
                      + particles_x[idx] * CHANNELS];

        err_sat =
            optimal_values[1] - pixel_ptr[
                      particles_y[idx] * COLS * CHANNELS
                      + particles_x[idx] * CHANNELS + 1];

        err_val =
            optimal_values[2] - pixel_ptr[
                      particles_y[idx] * COLS * CHANNELS
                      + particles_x[idx] * CHANNELS + 2];

        prediction = 1
                     / (1
                        + err_hue * err_hue * factors[0]
                        + err_sat * err_sat * factors[1]
                        + err_val * err_val * factors[2]);

        if (0.5 <= prediction ) {
            found_pixels.push_back(std::vector<uint16_t>{particles_x[idx], particles_y[idx]});
        }
    }

    return found_pixels;
}


std::vector<uint16_t> PixelEvaluator::search_median(const cv::Mat *img, const uint16_t &step) {
    int16_t err_hue;
    int16_t err_sat;
    int16_t err_val;

    double prediction;

    uint8_t *pixel_ptr = reinterpret_cast<uint8_t*>(img->data);
    const uint8_t CHANNELS = img->channels();
    const uint16_t COLS = img->cols;
    const size_t NBR_PIXELS = img->rows * img->cols;
    const size_t NBR_PARTICLES = NBR_PIXELS / step;

    std::vector<uint16_t> x_values;
    std::vector<uint16_t> y_values;
    std::vector<uint16_t> particles_x(NBR_PARTICLES);
    std::vector<uint16_t> particles_y(NBR_PARTICLES);

    for (size_t i = 0; i < NBR_PARTICLES; ++i) {
        particles_x[i] = (i*step) % 640;
        particles_y[i] = (i*step) / 640;
    }


    for (uint32_t idx = 0; idx < particles_x.size(); ++idx) {
        err_hue =
            optimal_values[0] - pixel_ptr[
                                    particles_y[idx] * COLS * CHANNELS
                                    + particles_x[idx] * CHANNELS];

        err_sat =
            optimal_values[1] - pixel_ptr[
                                    particles_y[idx] * COLS * CHANNELS
                                    + particles_x[idx] * CHANNELS + 1];

        err_val =
            optimal_values[2] - pixel_ptr[
                                    particles_y[idx] * COLS * CHANNELS
                                    + particles_x[idx] * CHANNELS + 2];

        prediction = 1
                     / (1
                        + err_hue * err_hue * factors[0]
                        + err_sat * err_sat * factors[1]
                        + err_val * err_val * factors[2]);

        if (0.5 <= prediction ) {
            x_values.push_back(particles_x[idx]);
            y_values.push_back(particles_y[idx]);
        }
    }

    std::vector<uint16_t> median;

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


