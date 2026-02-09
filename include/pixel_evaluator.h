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

    std::vector<std::vector<uint16_t>> search_for_pixel_with_color(const cv::Mat *img, const uint16_t &step);
    std::vector<uint16_t> search_median(const cv::Mat *img, const uint16_t &step);

  private:
    std::vector<double> factors;  // size 3
    std::vector<double> optimal_values;  // size 3

};
}
#endif // PIXEL_EVALUATOR_H
