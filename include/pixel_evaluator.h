#ifndef PIXEL_EVALUATOR_H
#define PIXEL_EVALUATOR_H

#include <math.h>
#include <cstdint>
#include <vector>

#include <opencv2/imgproc.hpp>


namespace mc {
class PixelEvaluator {
  public:
    PixelEvaluator(const std::vector<double> &factors,
                   const std::vector<double> &optimal_values);

    std::vector<std::vector<uint16_t>> search_for_pixel_with_color(const cv::Mat *img, const uint16_t &step, const uint8_t &min_neighbors, const bool aquire_false_positives = false);
    std::vector<int16_t> search_median(const cv::Mat *img, const uint16_t &step, const uint8_t &min_neighbors, const bool aquire_false_positives = false);
    void set_factors(const std::vector<double> &factors);
    std::vector<double> get_factors() const;
    void set_optimal_values(const std::vector<double> &optimal_values);
    std::vector<double> get_optimal_values() const;

    std::vector<std::vector<uint16_t>> get_false_positives() const;

    struct Point {
        bool valid = false;
        uint16_t x = -1;
        uint16_t y = -1;
    };



  private:
    std::vector<std::vector<Point>> create_prediction_matrix(const cv::Mat *img, const uint16_t &step);
    std::vector<std::vector<uint16_t>> false_positives;
    std::vector<double> factors;  // size 3
    std::vector<double> optimal_values;  // size 3

};
}
#endif // PIXEL_EVALUATOR_H
