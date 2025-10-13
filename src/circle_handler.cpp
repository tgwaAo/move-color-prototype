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
 * CircleHandler.cpp
 * 
 * Handles timers and visualization of circles.
 */

#include "circle_handler.h"

using namespace mc;

CircleHandler::CircleHandler() {
    CircleHandler(10, 10, std::vector<float>(3, 3), 0, 640, 480);
}

CircleHandler::CircleHandler(const uint8_t num_circles,
                             const uint8_t radius,
                             const std::vector<float> &circle_time_states,
                             const cv::Scalar& color,
                             const uint16_t width,
                             const uint16_t height) {
    if (circle_time_states.size() == SIZE_OF_TIME_STATES) {
        this->circle_time_states = circle_time_states;
    } else {
        this->circle_time_states = std::vector<float>(3, 10);
    }

    this->radius = radius;
    this->color = color;

    std::random_device rd;
    eng = std::mt19937(rd());
    // Use an index to select different positions.
    random_left_right_idx =
        std::uniform_int_distribution<uint16_t>(
            0,
            std::round((width - 2 * radius) / (2 * radius)));

    random_up_down_idx =
        std::uniform_int_distribution<uint16_t>(0,
                std::round((height - 2 * radius) / (2 * radius)));

    all_circles.resize(num_circles, Circle {cv::Point(0, 0), clock(), 0});

    std::uniform_int_distribution<uint8_t> random_state(0, 2);

    for (uint8_t i = 0; i < all_circles.size(); ++i) {
        all_circles[i].point.x = radius + 2 * radius * random_left_right_idx(eng);
        all_circles[i].point.y = radius + 2 * radius * random_up_down_idx(eng);
        all_circles[i].status = random_state(eng);
    }
}

CircleHandler::~CircleHandler() {}

void CircleHandler::update_circles(cv::Mat *const img) {
    float left_seconds;
    cv::Scalar used_color;

    for (uint8_t i = 0; i < all_circles.size(); ++i) {
        left_seconds =
            circle_time_states[all_circles[i].status]
            - static_cast<float>(clock() - all_circles[i].timer_start)
            / CLOCKS_PER_SEC;

        if (left_seconds <= 0) {
            if (all_circles[i].status == SIZE_OF_TIME_STATES - 1) {
                new_pos(i);
            } else {
                all_circles[i].status += 1;
                all_circles[i].timer_start = clock();
            }
        }

        if (all_circles[i].status != 0) {
            used_color.val[0] = color.val[0] * 0.5 * all_circles[i].status;
            used_color.val[1] = color.val[1] * 0.5 * all_circles[i].status;
            used_color.val[2] = color.val[2] * 0.5 * all_circles[i].status;
            cv::circle(
                *img,
                all_circles[i].point,
                radius,
                used_color,
                -1);
        }
    }
}

uint8_t CircleHandler::check_hit(const uint16_t x, const uint16_t y) {
    double dist;
    uint16_t hits = 0;

    for (uint8_t i = 0; i < all_circles.size(); ++i) {
        if (all_circles[i].status == 2) {
            dist =
                sqrt(
                    pow(x - all_circles[i].point.x, 2)
                    + pow(y - all_circles[i].point.y, 2));

            if (dist <= radius) {
                ++hits;
                new_pos(i);
            }
        }
    }

    return hits;
}

void CircleHandler::new_pos(uint8_t idx) {
    all_circles[idx].point.x = radius + 2 * radius * random_left_right_idx(eng);
    all_circles[idx].point.y = radius + 2 * radius * random_up_down_idx(eng);
    all_circles[idx].timer_start = clock();
    all_circles[idx].status = 0;
}

void CircleHandler::set_circle_time_states(
    const std::vector<float>& circle_time_states) {
    if (circle_time_states.size() == SIZE_OF_TIME_STATES) {
        this->circle_time_states = circle_time_states;
    }
}

void CircleHandler::set_color(const cv::Scalar& color) {
    this->color = color;
}

void CircleHandler::set_radius(const uint8_t radius) {
    this->radius = radius;
}

std::vector<float> CircleHandler::get_circle_time_states() const {
    return circle_time_states;
}

cv::Scalar CircleHandler::get_color() const {
    return color;
}

uint8_t CircleHandler::get_radius() const {
    return radius;
}
