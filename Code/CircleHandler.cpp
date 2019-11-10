// Modified MIT License
//
// Copyright (c) 2019 tgwaAo
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

#include "CircleHandler.h"

CircleHandler::CircleHandler()
{
    CircleHandler(10, 10, std::vector<float>(3, 3), 0, 640, 480);
}

CircleHandler::CircleHandler(const uint8_t numCircles, const uint8_t radius,
                             const std::vector<float> &circleTimeStates,
                             const cv::Scalar& color,
                             const uint16_t width,
                             const uint16_t height)
{
    if (circleTimeStates.size() == SIZE_OF_TIME_STATES) {
        this->circleTimeStates = circleTimeStates;
    } else {
        this->circleTimeStates = std::vector<float>(3, 10);
    }

    this->radius = radius;
    this->color = color;

    std::random_device rd;
    eng = std::mt19937(rd());
    // Use an index to select different positions.
    randomLeftRightIdx =
        std::uniform_int_distribution<uint16_t>(
            0,
            std::round((width - 2 * radius) / (2 * radius)));
    randomUpDownIdx =
        std::uniform_int_distribution<uint16_t>(0,
                std::round((height - 2 * radius) / (2 * radius)));

    allCircles.resize(numCircles, Circle {cv::Point(0, 0), clock(), 0});

    std::uniform_int_distribution<uint8_t> randomState(0, 2);

    for (uint8_t i = 0; i < allCircles.size(); ++i) {
        allCircles[i].point.x = radius + 2 * radius * randomLeftRightIdx(eng);
        allCircles[i].point.y = radius + 2 * radius * randomUpDownIdx(eng);
        allCircles[i].status = randomState(eng);
    }
}

CircleHandler::~CircleHandler()
{
}

void CircleHandler::updateCircles(cv::Mat *img)
{
    float leftSeconds;
    cv::Scalar usedColor;

    for (uint8_t i = 0; i < allCircles.size(); ++i) {
        leftSeconds =
            circleTimeStates[allCircles[i].status]
            - static_cast<float>(clock() - allCircles[i].timerStart)
            / CLOCKS_PER_SEC;

        if (leftSeconds <= 0) {
            if (allCircles[i].status == SIZE_OF_TIME_STATES - 1) {
                newPos(i);
            } else {
                allCircles[i].status += 1;
                allCircles[i].timerStart = clock();
            }
        }

        if (allCircles[i].status != 0) {
            usedColor.val[0] = color.val[0] * 0.5 * allCircles[i].status;
            usedColor.val[1] = color.val[1] * 0.5 * allCircles[i].status;
            usedColor.val[2] = color.val[2] * 0.5 * allCircles[i].status;
            cv::circle(
                *img,
                allCircles[i].point,
                radius,
                usedColor,
                -1);
        }
    }
}

uint8_t CircleHandler::checkHit(const uint16_t &x, const uint16_t &y)
{
    double dist;
    uint16_t hits = 0;

    for (uint8_t i = 0; i < allCircles.size(); ++i) {
        if (allCircles[i].status == 2) {
            dist =
                sqrt(
                    pow(x - allCircles[i].point.x, 2)
                    + pow(y - allCircles[i].point.y, 2));

            if (dist <= radius) {
                ++hits;
                newPos(i);
            }
        }
    }

    return hits;
}

void CircleHandler::newPos(uint8_t idx)
{
    allCircles[idx].point.x = radius + 2 * radius * randomLeftRightIdx(eng);
    allCircles[idx].point.y = radius + 2 * radius * randomUpDownIdx(eng);
    allCircles[idx].timerStart = clock();
    allCircles[idx].status = 0;
}

void CircleHandler::setCircleTimeStates(
    const std::vector<float>& circleTimeStates)
{
    if (circleTimeStates.size() == SIZE_OF_TIME_STATES) {
        this->circleTimeStates = circleTimeStates;
    }
}

void CircleHandler::setColor(const cv::Scalar& color)
{
    this->color = color;
}

void CircleHandler::setRadius(const uint8_t radius)
{
    this->radius = radius;
}

std::vector<float> CircleHandler::getCircleTimeStates() const
{
    return circleTimeStates;
}

cv::Scalar CircleHandler::getColor() const
{
    return color;
}

uint8_t CircleHandler::getRadius() const
{
    return radius;
}
