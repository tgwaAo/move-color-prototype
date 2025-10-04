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

#ifndef CODE_INCLUDE_PARTICLEWEIGHTING_H_
#define CODE_INCLUDE_PARTICLEWEIGHTING_H_

#include <cstdint>
#include <math.h>
#include <vector>

/**
 * @class ParticleWeighting
 * @date 09/10/19
 * @file Particle_weighting.h
 * @brief Decide, whether a color is in this area or not via particles.
 */
class ParticleWeighting {
  public:
    /**
     * @brief Constructs a continuous matrix of particles in given area
     * and initializes necessary attributes.
     * @param num_particles Number of particles.
     * @param min_width Where width starts.
     * @param max_width Where width ends.
     * @param min_height Where height starts.
     * @param max_height Where height ends.
     * @param max_weight Weight of a particle without error.
     * @param cols Number of horizontal pixels.
     * @param bound Minimal number of positive particles.
     * @param factors Factors multiplied by squared error.
     * @param optimal_hsv_values HSV values closest to ideal values.
     */
    ParticleWeighting(const uint16_t num_particles,
                      const double min_width,
                      const double max_width,
                      const double min_height,
                      const double max_height,
                      const uint16_t max_weight,
                      const uint16_t cols,
                      const uint32_t bound,
                      const std::vector<double> &factors,
                      const std::vector<double> &optimal_hsv_values);

    /**
     * @brief Decide, wheter area contains color or not.
     * @param pixel_ptr Pointer pointing to current image.
     * @return Decision of containing color.
     */
    bool is_color(const uint8_t *const pixel_ptr);

    /**
     * @brief Update position of particles after changed size.
     */
    void update();

    /**
     * @brief Get values of a particle.
     * @param idx Index of wanted particle.
     * @param back_x X position of particle.
     * @param back_y Y position of particle.
     * @param back_w Weight of particle.
     * @param alright Is alright, if idx_ is in valid range.
     */
    void get_particle(const uint16_t idx,
                     uint16_t &back_x,
                     uint16_t &back_y,
                     uint16_t &back_w,
                     bool &alright) const;

    /**
     * @brief Set values of a particle.
     * @param idx Index of interesting particle.
     * @param change_x Next x position of particle.
     * @param change_y Next y position of particle.
     * @param change_w Next weight of particle.
     */
    void set_particle(const uint16_t idx,
                     const uint16_t change_x,
                     const uint16_t change_y,
                     const uint16_t change_w);

    /**
     * @brief Get sum of weights.
     * @return Sum of weights.
     */
    uint32_t get_sum_weights() const;

    /**
     * @brief Set sum of weights.
     * @param sum_weights New sum of weights.
     */
    void set_sum_weights(const uint32_t sum_weights);

    /**
     * @brief Get weight of no error.
     * @return Weight of no error.
     */
    uint16_t get_max_weight() const;

    /**
     * @brief Set maximum possible weight.
     * @param max_weight Maximum possible weight.
     */
    void set_max_weight(const uint16_t max_weight);

    /**
     * @brief Get horizontal start of area.
     * @return Horizontal start of area.
     */
    uint16_t get_min_width() const;

    /**
     * @brief Set horizontal start of area.
     * @param min_width Horizontal start of area.
     */
    void set_min_width(const uint16_t min_width);

    /**
     * @brief Get vertical start of area.
     * @return Vertical start of area.
     */
    uint16_t get_min_height() const;

    /**
     * @brief Set vertical start of area.
     * @param min_height Vertical start of area.
     */
    void set_min_height(const uint16_t min_height);

    /**
     * @brief Get horizontal end of area.
     * @return Horizontal end of area.
     */
    uint16_t get_max_width() const;

    /**
     * @brief Set horizontal end of area.
     * @param max_width Horizontal end of area.
     */
    void set_max_width(const uint16_t max_width);

    /**
     * @brief Get vertical end of area.
     * @return Vertical end of area.
     */
    uint16_t get_max_height() const;

    /**
     * @brief Set vertical end of area.
     * @param max_height Vertical end of area.
     */
    void set_max_height(const uint16_t max_height);

    /**
     * @brief Get vector of optimal hsv values.
     * @return Vector of optimal hsv values.
     */
    std::vector<double> get_hsv() const;

    /**
     * @brief Set optimal hsv values.
     * @param hsv Optimal hsv values.
     * @return Returns true, if input vector has a length of 3.
     */
    bool set_hsv(const std::vector<double> &hsv);

    /**
     * @brief Get factors of squared errors of hsv values.
     * @return Factors of squared errors of hsv values.
     */
    std::vector<double> get_factors() const;

    /**
     * @brief Set factors of squared errors of hsv values.
     * @param factors Factors of squared errors of hsv values.
     * @return Returns true, if input vector has a length of 3.
     */
    bool set_factors(const std::vector<double> &factors);

  private:
    const uint8_t HSV_CHANNELS = 3;

    std::vector<uint16_t> particles_x;
    std::vector<uint16_t> particles_y;
    std::vector<uint16_t> particles_w;
    uint32_t boundary;
    uint32_t sum_weights;
    uint16_t max_weight;
    uint16_t min_width;
    uint16_t min_height;
    uint16_t max_width;
    uint16_t max_height;
    uint16_t cols;
    uint16_t hue;
    uint16_t sat;
    uint16_t val;
    double factor_h;
    double factor_s;
    double factor_v;
};

#endif // CODE_INCLUDE_PARTICLEWEIGHTING_H_
