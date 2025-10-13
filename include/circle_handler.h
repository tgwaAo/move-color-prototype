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

#ifndef CODE_INCLUDE_CIRCLEHANDLER_H_
#define CODE_INCLUDE_CIRCLEHANDLER_H_

#include <opencv2/opencv.hpp>
#include <random>
#include <vector>

namespace mc {
/**
 * @class CircleHandler
 * @date 09/10/19
 * @file Circle_handler.h
 * @brief Class handles circles and enables hits of them (gameplay).
 */
class CircleHandler {
  public:
    /**
     * @brief Default constructor calling other constructor with 10 circles
     *        with a radius of ten. The time of each state is 3 seconds and
     *        initial state is dark (visible, but not active jet). The circles
     *        are blue and created in a window of size 480x640.
     */
    CircleHandler();

    /**
     * @brief Constructor initializing times of states and active state, color
     *        and number of circles and window of creation.
     * @param num_circles Number of used circles.
     * @param radius_ Radius of used circles.
     * @param circle_time_states Time of each state each circle has.
     * @param color Color of active state (brightest).
     * @param width Width of possible positions in window.
     * @param height Height of possible positions in window.
     */
    CircleHandler(const uint8_t num_circles,
                  const uint8_t radius_,
                  const std::vector<float> &circle_time_states,
                  const cv::Scalar &color,
                  const uint16_t width = 640,
                  const uint16_t height = 480);

    /**
     * @brief Destructor of class.
     */
    ~CircleHandler();

    /**
     * @brief Update states and drawings.
     * @param img Used image to draw circles.
     */
    void update_circles(cv::Mat *img);

    /**
     * @brief Check hits of circles at position.
     * @param x Current x-value to check hit.
     * @param y Current y-value to check hit.
     * @return Number of hits.
     */
    uint8_t check_hit(const uint16_t x, const uint16_t y);

    /**
     * @class Circle
     * @date 09/10/19
     * @file Circle_handler.h
     * @brief Struct to use circles with a point for position, a timer to switch
     * states and a current status (invisible, dark = not active, bright =
     * active).
     *
     */
    struct Circle {
        /**
         * @brief point Center of a circle.
         */
        cv::Point point;
        /**
         * @brief timer_start Start of current status.
         */
        clock_t timer_start;
        /**
         * @brief status Current status of circle.
         */
        uint8_t status;
    };

    /**
     * @brief Set times of different states.
     * @param circle_time_states Possible states of the circles
     */
    void set_circle_time_states(const std::vector<float> &circle_time_states);

    /**
     * @brief Set color of shown circles.
     * @param color Color of shown circles.
     */
    void set_color(const cv::Scalar &color);

    /**
     * @brief Set radius of shown circles.
     * @param radius Radius of shown circles.
     */
    void set_radius(const uint8_t radius);

    /**
     * @brief Get time of state at given index.
     * @param idx Index of time state.
     * @return
     */
    std::vector<float> get_circle_time_states() const;

    /**
     * @brief Get color of drawn circles.
     * @return Color of drawn circles.
     */
    cv::Scalar get_color() const;

    /**
     * @brief Get radius of drawn cirlces.
     * @return Radius of drawn circles.
     */
    uint8_t get_radius() const;

  private:
    /**
     * @brief New position, state and new time of a circle.
     * @param idx Index of changed circle.
     */
    void new_pos(uint8_t idx);

    const uint8_t SIZE_OF_TIME_STATES = 3;

    uint8_t radius;
    cv::Scalar color;
    std::vector<float> circle_time_states;
    std::vector<Circle> all_circles;
    std::mt19937 eng;
    std::uniform_int_distribution<uint16_t> random_up_down_idx;
    std::uniform_int_distribution<uint16_t> random_left_right_idx;
};
} // namespace mc
#endif // CODE_INCLUDE_CIRCLEHANDLER_H_
