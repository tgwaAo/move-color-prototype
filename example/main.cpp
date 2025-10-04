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
 * Start of a little game. The goal is to hit the bright
 * green circles and avoid the bright red circles.
 * The dark green and red circles are not active and
 * will turn bright later.
 */

#include <sys/stat.h>
#include <dirent.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "CalibrationHandler.h"
#include "CircleHandler.h"
#include "ParticleWeighting.h"

/**
 * @brief A single play.
 * @param cap Camera for acquisition of pictures.
 * @param mirror Image for visualization.
 * @param title Title of game play.
 * @param corner_2_center Distance from center of particle weighting to its start.
 * @param pos_handler Handler of positive circles.
 * @param neg_handler Handler of negative circles.
 * @param max_distance Distance of one particle weighting.
 * @param weighting_matrix Matrix of particle weightings.
 * @return Code of user input. 0 for replay, 1 for end of game and 2 for calibration.
 */
uint8_t gameplay(
    cv::VideoCapture *cap,
    cv::Mat *mirror,
    const std::string &title,
    const uint8_t corner_2_center,
    CircleHandler *pos_handler,
    CircleHandler *neg_handler,
    const uint8_t max_distance,
    std::vector<std::vector
    <ParticleWeighting>> *weighting_matrix);

/**
* @brief Take a photo after 3 seconds.
* @param cap Camera to shot photo.
* @param image Image to save picture.
* @param title Title shown on counting down.
* @param key_abort Key to abort timer.
* @return False after abort.
*/
bool photo_with_timer(
    cv::VideoCapture *cap,
    cv::Mat *image,
    const std::string &title,
    int16_t key_abort);

/**
* @brief Count down and display left seconds.
* @param cap Camera to shot photo.
* @param image Image to store the picture.
* @param title Title shown on counting down.
* @param key_abort Key to abort timer.
* @param seconds_to_wait Seconds before function returns.
* @return False after abort and true after countdown.
*/
bool wait_and_show_seconds(
    cv::VideoCapture *cap,
    cv::Mat *image,
    const std::string &title,
    int16_t key_abort,
    const uint8_t &seconds_to_wait);


/**
 * @brief Load last highscore and return current highscore.
 * @param filename Name of highscore file.
 * @param hits Number of hits in actual game play.
 * @return Actual highscore.
 */
int16_t load_highscore(const std::string &filename, int16_t hits);

/**
 * @brief Save calibration in a txt file.
 * @param filename Name of the calibration file.
 * @param hsv_color Vector of bright optimal values.
 * @param factors_color Vector of bright factors.
 */
void save_calibration(
    const std::string &filename,
    const std::vector<double> &hsv_color,
    const std::vector<double> &factors_color);

int main() {
    /*******************************************************************
     * Set camera up and initialize variables.
     * ****************************************************************/
    struct stat buffer;
    const std::string CAM_FILENAME = "cam.txt";
    int cam_nbr;

    if (stat(CAM_FILENAME.c_str(), &buffer) == 0) {
        std::ifstream cam_file;
        cam_file.open(CAM_FILENAME);

        cam_file >> cam_nbr;

        cam_file.close();
    } else {
        cam_nbr = 0;
    }

    std::cout << "cam_nbr:" << cam_nbr << std::endl;
    std::unique_ptr<cv::VideoCapture> cap(new cv::VideoCapture(cam_nbr));

    if (cap->isOpened() == false) {
        cv::Mat error_image{cv::Size(640, 480), CV_8UC3, cv::Scalar(0, 0, 0)};
        cv::putText(
            error_image,
            "Camera " +  std::to_string(cam_nbr) + " could not be opened.",
            cv::Point(10, error_image.rows - 10),
            cv::FONT_HERSHEY_SIMPLEX,
            1.2,
            cv::Scalar(255, 255, 0));
        cv::imshow("Error", error_image);
        cv::waitKey(0);
        cv::destroyAllWindows();
        return -1;
    }

    const double WIDTH = 640;
    const double HEIGHT = 480;

    cap->set(cv::CAP_PROP_FRAME_WIDTH, WIDTH);
    cap->set(cv::CAP_PROP_FRAME_HEIGHT, HEIGHT);

    const std::string TITLE = "Calibration";
    cv::namedWindow(TITLE, cv::WINDOW_AUTOSIZE);
    cv::resizeWindow(TITLE, WIDTH, HEIGHT);

    // Just wrong values for an aborted calibration at first start.
    std::vector<double> optimal_hsv_color(3, 0);
    std::vector<double> factors_color(3, 0);

    const std::string SETTINGS_FILENAME = "hsv.txt";
    const int16_t key_abort = 27;
    CalibrationHandler calibrator(TITLE);
    std::unique_ptr<cv::Mat> mirror(new cv::Mat);

    /*********************************************************************
     * Load configuration or calibrate a new.
     * ******************************************************************/
    if (stat(SETTINGS_FILENAME.c_str(), &buffer) == 0) {
        std::ifstream settings_file;
        settings_file.open(SETTINGS_FILENAME);

        settings_file >> optimal_hsv_color[0];
        settings_file >> optimal_hsv_color[1];
        settings_file >> optimal_hsv_color[2];
        settings_file >> factors_color[0];
        settings_file >> factors_color[1];
        settings_file >> factors_color[2];

        settings_file.close();
    } else {
        if (photo_with_timer(cap.get(), mirror.get(), TITLE, key_abort)) {
            if (calibrator.calibrate(
                    mirror.get(),
                    &optimal_hsv_color,
                    &factors_color)) {
                save_calibration(
            SETTINGS_FILENAME,
            optimal_hsv_color,
            factors_color);
            }
        }
    }

    /************************************************************
     * Setup matrices to find colors.
     * *********************************************************/
    const uint16_t NUM_PARTICLES = 30;
    const uint8_t MAX_WEIGHT = 1;  // TODO: check possible removal
    const uint8_t MAX_DISTANCE = 10;
    uint16_t mat_size = HEIGHT / MAX_DISTANCE;
    std::unique_ptr<std::vector<std::vector<ParticleWeighting>
    > > weighting_matrix(
        new std::vector<std::vector<ParticleWeighting>>(
            mat_size,
            std::vector
            <ParticleWeighting>(WIDTH / MAX_DISTANCE,
                                ParticleWeighting(
                                    NUM_PARTICLES,
                                    0,
                                    10,
                                    0,
                                    10,
                                    MAX_WEIGHT,
                                    WIDTH,
                                    NUM_PARTICLES * MAX_WEIGHT * 4 / 10,
                                    factors_color,
                                    optimal_hsv_color))));

    for (int i = 0; i < HEIGHT / MAX_DISTANCE; ++i) {
        for (int j = 0; j < WIDTH / MAX_DISTANCE; ++j) {
            (*weighting_matrix)[i][j].set_min_height(i * MAX_DISTANCE);
            (*weighting_matrix)[i][j].set_max_height((i + 1) * MAX_DISTANCE);
            (*weighting_matrix)[i][j].set_min_width(j * MAX_DISTANCE);
            (*weighting_matrix)[i][j].set_max_width((j + 1) * MAX_DISTANCE);
            (*weighting_matrix)[i][j].update();
        }
    }

    /******************************************************************
     * Define circle data.
     * ***************************************************************/
    uint8_t target_radius = 40;

    /********************************************************
     * Circle and timer (Circles have timers)
     * *****************************************************/
    std::vector<float> state_times(3, 1);
    state_times[2] = 3;

    std::unique_ptr<CircleHandler> pos_handler(
        new CircleHandler(
            7,
            target_radius,
            state_times,
            cv::Scalar(0, 254, 0),
            WIDTH,
            HEIGHT));

    for (uint8_t i = 0; i < state_times.size(); ++i)
        state_times[i] = 5;

    target_radius = 30;
    std::unique_ptr<CircleHandler> neg_handler(
        new CircleHandler(5,
                          target_radius,
                          state_times,
                          cv::Scalar(0, 0, 254),
                          WIDTH,
                          HEIGHT));

    /*******************************************************
     * Let the game start.
     * ****************************************************/
    uint8_t key_code;
    const uint8_t CODE_END = 1;
    const uint8_t CODE_CALIBRATE = 2;
    bool run = true;
    const uint8_t CORNER_2_CENTER = MAX_DISTANCE / 2;

    while (run) {
        key_code = gameplay(
                      cap.get(),
                      mirror.get(),
                      TITLE,
                      CORNER_2_CENTER,
                      pos_handler.get(),
                      neg_handler.get(),
                      MAX_DISTANCE,
                      weighting_matrix.get());

        switch (key_code) {
        case CODE_END:
            run = false;
            break;
        case CODE_CALIBRATE:
            if (photo_with_timer(
                    cap.get(),
                    mirror.get(),
                    TITLE,
                    key_abort))
                if (calibrator.calibrate(
                        mirror.get(),
                        &optimal_hsv_color,
                        &factors_color)) {
                    save_calibration(
                        SETTINGS_FILENAME,
                        optimal_hsv_color,
                        factors_color);

                    for (int i = 0; i < HEIGHT / MAX_DISTANCE; ++i) {
                        for (int j = 0; j < WIDTH / MAX_DISTANCE; ++j) {
                            (*weighting_matrix)[i][j].set_hsv(optimal_hsv_color);
                            (*weighting_matrix)[i][j].set_factors(factors_color);
                        }
                    }
                }
            break;
        }
    }

    cap->release();
    cv::destroyAllWindows();
    return 0;
}

uint8_t gameplay(
    cv::VideoCapture *cap,
    cv::Mat *mirror,
    const std::string &title,
    const uint8_t corner_2_center,
    CircleHandler *pos_handler,
    CircleHandler *neg_handler,
    const uint8_t max_distance,
    std::vector<std::vector
    <ParticleWeighting>> *weighting_matrix) {
    cv::Mat frame;
    cv::Mat hsv;
    uint8_t *pixel_ptr_hsv;
    cv::Scalar color;
    const cv::Scalar BAD_COLOR(0, 0, 255);
    const cv::Scalar GOOD_COLOR(255, 0, 0);
    int16_t key;
    bool good_area;
    float left_seconds;
    int16_t hits = 0;

    const int16_t KEY_ESC = 27;
    const int16_t KEY_C = 99;
    const int16_t KEY_D = 100;
    const int16_t KEY_R = 114;
    const uint8_t game_time = 60;
    std::ostringstream time_message;

    if (!wait_and_show_seconds(cap, mirror, title, KEY_ESC, 5)) {
        return 1;
    }

    clock_t time_start = clock();

    while (true) {
        *cap >> frame;
        cv::flip(frame, *mirror, 1);
        cv::cvtColor(*mirror, hsv, cv::COLOR_BGR2HSV);
        pixel_ptr_hsv = reinterpret_cast<uint8_t*>(hsv.data);

        pos_handler->update_circles(mirror);
        neg_handler->update_circles(mirror);

        for (uint32_t i = 0; i < (*weighting_matrix).size(); ++i) {
            for (uint32_t j = 0; j < (*weighting_matrix)[i].size(); ++j) {
                good_area = (*weighting_matrix)[i][j].is_color(pixel_ptr_hsv);

                if (good_area) {
                    color = GOOD_COLOR;
                    cv::line(
                        *mirror,
                        cv::Point(
                            j * max_distance + corner_2_center,
                            i * max_distance + corner_2_center),
                        cv::Point(
                            j * max_distance + corner_2_center,
                            i * max_distance + corner_2_center),
                        color,
                        10,
                        10);

                    hits += pos_handler->check_hit(
                                j * max_distance + corner_2_center,
                                i * max_distance + corner_2_center);
                    hits -= 5 * neg_handler->check_hit(
                                j * max_distance + corner_2_center,
                                i * max_distance + corner_2_center);
                }
            }
        }

        cv::putText(
            *mirror,
            "Hits= " + std::to_string(hits),
            cv::Point(10, mirror->rows - 10),
            cv::FONT_HERSHEY_SIMPLEX,
            1.2,
            cv::Scalar(255, 255, 0));

        left_seconds = game_time - static_cast<float>(clock() - time_start)
                      / CLOCKS_PER_SEC;

        time_message << "Time left= " << std::fixed << std::setprecision(2) << left_seconds;

        cv::putText(
            *mirror,
            time_message.str(),
            cv::Point(200, mirror->rows - 10),
            cv::FONT_HERSHEY_SIMPLEX,
            1.2,
            cv::Scalar(255, 255, 0));

        time_message.str("");

        if (cv::getWindowProperty(title, cv::WND_PROP_VISIBLE)) {
            cv::imshow(title, *mirror);
            key = cv::waitKey(1);

            if (key == KEY_R) {
                return 0;
            } else if (key == KEY_ESC) {
                break;
            } else if (key == KEY_C) {
                return 2;
            } else if (key == KEY_D) {
                cv::imwrite("debug_image.jpg", *mirror);
                std::ofstream save_file;
                save_file.open("debug_hsv.txt");

                save_file << (*weighting_matrix)[0][0].get_hsv()[0] << std::endl;
                save_file << (*weighting_matrix)[0][0].get_hsv()[1] << std::endl;
                save_file << (*weighting_matrix)[0][0].get_hsv()[2] << std::endl;
                save_file <<
                    (*weighting_matrix)[0][0].get_factors()[0]
                    << std::endl;
                save_file <<
                    (*weighting_matrix)[0][0].get_factors()[1]
                    << std::endl;
                save_file <<
                    (*weighting_matrix)[0][0].get_factors()[2]
                    << std::endl;

                save_file.close();
            }
        }
        if (left_seconds <= 0) break;
    }

    /******************************************************
     * Get actual highscore and show it.
     * ***************************************************/
    const std::string HIGHSCORE_FILENAME = "highscore.txt";
    int16_t highscore = load_highscore(HIGHSCORE_FILENAME, hits);

    cv::putText(
        *mirror,
        "Finished",
        cv::Point(20, mirror->rows / 2),
        cv::FONT_HERSHEY_SIMPLEX,
        1.2,
        cv::Scalar(255, 255, 0));
    cv::putText(
        *mirror,
        "Highscore = " + std::to_string(highscore),
        cv::Point(20, 30 + mirror->rows / 2),
        cv::FONT_HERSHEY_SIMPLEX,
        1.2,
        cv::Scalar(255, 255, 0));
    cv::imshow(title, *mirror);

    while (true) {
        if (cv::getWindowProperty(title, cv::WND_PROP_VISIBLE)) {
            key = cv::waitKey(100);

            if (key != -1) {
                if (key == KEY_R)
                    return 0;
                else if (key == KEY_C)
                    return 2;
                else
                    break;
            }
        } else {
            break;
        }
    }
    return 1;
}


bool wait_and_show_seconds(
    cv::VideoCapture *cap,
    cv::Mat *image,
    const std::string &title,
    int16_t key_abort,
    const uint8_t &seconds_to_wait) {
    cv::Mat frame;
    int16_t key;

    double time_start = cv::getTickCount();
    int8_t left_seconds = seconds_to_wait - (cv::getTickCount() - time_start)
                         / cv::getTickFrequency();

    while (left_seconds > 0) {
        *cap >> frame;
        cv::flip(frame, *image, 1);

        left_seconds = seconds_to_wait - (cv::getTickCount() - time_start)
                      / cv::getTickFrequency();

        cv::putText(
            *image,
            "countdown= " + std::to_string(left_seconds),
            cv::Point(10, image->rows - 10),
            cv::FONT_HERSHEY_SIMPLEX,
            1.2,
            cv::Scalar(255, 255, 0),
            2);

        cv::imshow(title, *image);
        key = cv::waitKey(1);

        if (key == key_abort)
            return false;
    }
    return true;
}


// Take photo after 3 seconds.
bool photo_with_timer(
    cv::VideoCapture *cap,
    cv::Mat *image,
    const std::string &title,
    int16_t key_abort) {
    const uint8_t min_time_passed = 5;
    cv::Mat frame;

    if (!wait_and_show_seconds(cap, image, title, key_abort, min_time_passed)) {
        return false;
    }

    *cap >> frame;
    cv::flip(frame, *image, 1);
    return true;
}

int16_t load_highscore(const std::string &filename, int16_t hits) {
    struct stat buffer;
    int16_t highscore;

    bool already_exits = stat(filename.c_str(), &buffer) == 0;

    if (already_exits) {
        std::ifstream read_highscore_file;
        read_highscore_file.open(filename);

        read_highscore_file >> highscore;

        read_highscore_file.close();

        if (highscore < hits) {
            std::ofstream write_highscore_file;
            write_highscore_file.open(filename);

            write_highscore_file << hits;

            write_highscore_file.close();
            return hits;
        } else {
            return highscore;
        }
    } else {
        std::ofstream write_highscore_file;
        write_highscore_file.open(filename);

        write_highscore_file << hits;

        write_highscore_file.close();

        return hits;
    }
}

void save_calibration(
    const std::string &filename,
    const std::vector<double> &hsv_color,
    const std::vector<double> &factors_color) {
    std::ofstream save_file;
    save_file.open(filename);

    save_file << unsigned(hsv_color[0]) << std::endl;
    save_file << unsigned(hsv_color[1]) << std::endl;
    save_file << unsigned(hsv_color[2]) << std::endl;
    save_file << factors_color[0] << std::endl;
    save_file << factors_color[1] << std::endl;
    save_file << factors_color[2] << std::endl;

    save_file.close();
    std::cout << "Saved" << std::endl;
}
