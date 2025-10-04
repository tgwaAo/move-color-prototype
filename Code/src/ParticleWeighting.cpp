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
 * ParticleWeighting.cpp
 *
 * Class to calculate decision of containing a color in hsv space or not.
 * This is done via unmoving particles, because the color
 * from one pixel and the one next to it will not change often.
 */

#include "ParticleWeighting.h"


ParticleWeighting::ParticleWeighting(
    const uint16_t num_particles,
    const double min_width,
    const double max_width,
    const double min_height,
    const double max_height,
    const uint16_t max_weight,
    const uint16_t cols,
    const uint32_t bound,
    const std::vector<double> &factors,
    const std::vector<double> &optimal_hsv_values
) {
    this->min_width = min_width;
    this->max_width = max_width;
    this->min_height = min_height;
    this->max_height = max_height;
    this->max_weight = max_weight;

    if (factors.size() == HSV_CHANNELS) {
        factor_h = factors[0];
        factor_s = factors[1];
        factor_v = factors[2];
    }

    if (optimal_hsv_values.size() == HSV_CHANNELS) {
        hue = optimal_hsv_values[0];
        sat = optimal_hsv_values[1];
        val = optimal_hsv_values[2];
    }

    this->cols = cols;
    this->boundary = bound;

    particles_x.resize(num_particles);
    particles_y.resize(num_particles);
    particles_w.resize(num_particles);

    update();
}

bool ParticleWeighting::is_color(const uint8_t *const pixel_ptr) {
    int16_t err_hue;
    int16_t err_sat;
    int16_t err_val;

    double prediction;
    sum_weights = 0;
    uint32_t failure_idx = particles_x.size() - boundary;

    for (uint32_t idx = 0; idx < particles_x.size(); ++idx) {
        // calculate probability of being correct
        err_hue =
            hue - pixel_ptr[
         particles_y[idx] * cols * HSV_CHANNELS
         + particles_x[idx] * HSV_CHANNELS];

        err_sat =
            sat - pixel_ptr[
         particles_y[idx] * cols * HSV_CHANNELS
         + particles_x[idx] * HSV_CHANNELS + 1];

        err_val =
            val - pixel_ptr[
         particles_y[idx] * cols * HSV_CHANNELS
         + particles_x[idx] * HSV_CHANNELS + 2];

        prediction = 1
                     / (1
                        + pow(err_hue, 2) * factor_h
                        + pow(err_sat, 2) * factor_s
                        + pow(err_val, 2) * factor_v);

        if (prediction < 0.5)
            particles_w[idx] = 0;
        else
            particles_w[idx] = max_weight;

        sum_weights += particles_w[idx];

        if (boundary <= sum_weights)
            return true;

        uint32_t left_particles = particles_x.size() - idx;
        uint64_t possible_positives = left_particles * max_weight;
        uint32_t needed_positives = boundary - sum_weights;

        if (possible_positives < needed_positives)
            return false;
    }
    return false;
}

void ParticleWeighting::update() {
    uint16_t diff = max_width - min_width;
    uint32_t num_pixels = diff * (max_height - min_height);
    uint16_t step = num_pixels / particles_x.size();

    for (uint32_t i = 0; i < particles_x.size(); ++i) {
        particles_x[i] = ((i*step) % static_cast<int>(diff)) + min_width;
        particles_y[i] = ((i*step) / diff) + min_height;
        particles_w[i] = 1;
    }
}

std::vector<double> ParticleWeighting::get_hsv() const {
    std::vector<double> hsv(3, 0);
    hsv[0] = hue;
    hsv[1] = sat;
    hsv[2] = val;
    return hsv;
}

bool ParticleWeighting::set_hsv(const std::vector<double> &hsv) {
    if (hsv.size() == HSV_CHANNELS) {
        hue = hsv[0];
        sat = hsv[1];
        val = hsv[2];
        return true;
    } else {
        return false;
    }
}

std::vector<double> ParticleWeighting::get_factors() const {
    return std::vector<double> {factor_h, factor_s, factor_v};
}

bool ParticleWeighting::set_factors(const std::vector<double> &factors) {
    if (factors.size() == HSV_CHANNELS) {
        factor_h = factors[0];
        factor_s = factors[1];
        factor_v = factors[2];
        return true;
    } else {
        return false;
    }
}

void ParticleWeighting::get_particle(
    const uint16_t idx,
    uint16_t& back_x,
    uint16_t& back_y,
    uint16_t& back_w,
    bool& alright) const {
    if (idx < particles_x.size()) {
        back_x = particles_x[idx];
        back_y = particles_y[idx];
        back_w = particles_w[idx];
        alright = true;
    } else {
        back_x = 0;
        back_y = 0;
        back_w = 0;
        alright = false;
    }
}

void ParticleWeighting::set_sum_weights(const uint32_t sum_weights) {
    this->sum_weights = sum_weights;
}

uint32_t ParticleWeighting::get_sum_weights() const {
    return sum_weights;
}

uint16_t ParticleWeighting::get_max_weight() const {
    return max_weight;
}

void ParticleWeighting::set_max_weight(const uint16_t max_weight) {
    this->max_weight = max_weight;
}

uint16_t ParticleWeighting::get_min_width() const {
    return min_width;
}

void ParticleWeighting::set_min_width(const uint16_t min_width) {
    this->min_width = min_width;
}

uint16_t ParticleWeighting::get_min_height() const {
    return min_height;
}

void ParticleWeighting::set_min_height(const uint16_t min_height) {
    this->min_height = min_height;
}

uint16_t ParticleWeighting::get_max_width() const {
    return max_width;
}

void ParticleWeighting::set_max_width(const uint16_t max_width) {
    this->max_width = max_width;
}

uint16_t ParticleWeighting::get_max_height() const {
    return max_height;
}

void ParticleWeighting::set_max_height(const uint16_t max_height) {
    this->max_height = max_height;
}
