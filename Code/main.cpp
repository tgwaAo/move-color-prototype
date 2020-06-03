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
 * Start of a little game. The goal is to hit the bright
 * green circles and avoid the bright red circles.
 * The dark green and red circles are not active and
 * will turn bright later.
 */

#include <sys/stat.h>
#include <vector>
#include <fstream>
#include <iomanip>
#include <random>
#include <opencv2/opencv.hpp>

#include "CalibrationHandler.h"
#include "CircleHandler.h"
#include "ParticleWeighting.h"
 
/**
 * @brief A single play.
 * @param cap Camera for acquisition of pictures.
 * @param mirror Image for visualization.
 * @param title Title of game play.
 * @param corner2center Distance from center of particle weighting to its start.
 * @param posHandler Handler of positive circles.
 * @param negHandler Handler of negative circles.
 * @param maxDistance Distance of one particle weighting.
 * @param weightingMatrix Matrix of particle weightings.
 * @return Code of user input. 0 for replay, 1 for end of game and 2 for calibration.
 */
uint8_t gameplay(
    cv::VideoCapture *cap,
    cv::Mat *mirror,
    const std::string &title,
    const uint8_t corner2center,
    CircleHandler *posHandler,
    CircleHandler *negHandler,
    const uint8_t maxDistance,
    std::vector<std::vector
    <ParticleWeighting>> *weightingMatrix);

/**
* @brief Take a photo after 3 seconds.
* @param cap Camera to shot photo.
* @param image Image to save picture.
* @param title Title shown on counting down.
* @param keyAbort Key to abort timer.
* @return False after abort.
*/
bool photoWithTimer(
    cv::VideoCapture *cap,
    cv::Mat *image,
    const std::string &title,
    int16_t keyAbort);

/**
 * @brief Load last highscore and return current highscore.
 * @param filename Name of highscore file.
 * @param hits Number of hits in actual game play.
 * @return Actual highscore.
 */
int16_t loadHighscore(const std::string &filename, int16_t hits);

/**
 * @brief Save calibration in a txt file.
 * @param filename Name of the calibration file.
 * @param hsvColor Vector of bright optimal values.
 * @param factorsColor Vector of bright factors.
 */
void saveCalibration(
    const std::string &filename,
    const std::vector<uint16_t> &hsvColor,
    const std::vector<double> &factorsColor);

int main()
{
    /*******************************************************************
     * Set camera up and initialize variables.
     * ****************************************************************/
    struct stat buffer;
    const std::string CAM_FILENAME = "cam.txt";
    int camNbr;

    if (stat(CAM_FILENAME.c_str(), &buffer) == 0) {
        std::ifstream cam_file;
        cam_file.open(CAM_FILENAME);

        cam_file >> camNbr;

        cam_file.close();
    } else {
        camNbr = 0;
    } 


    std::unique_ptr<cv::VideoCapture> cap(new cv::VideoCapture(camNbr));

    if (!cap->isOpened()) {
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
    std::vector<uint16_t> hsvColor(3, 0);
    std::vector<double> factorsColor(3, 1000000);

    const std::string SETTINGS_FILENAME = "hsv.txt";
    const int16_t keyAbort = 27;
    CalibrationHandler calibrator(TITLE);
    cv::Mat *mirror = new cv::Mat;

    /*********************************************************************
     * Load configuration or calibrate a new.
     * ******************************************************************/
    if (stat(SETTINGS_FILENAME.c_str(), &buffer) == 0) {
        std::ifstream settings_file;
        settings_file.open(SETTINGS_FILENAME);

        settings_file >> hsvColor[0];
        settings_file >> hsvColor[1];
        settings_file >> hsvColor[2];
        settings_file >> factorsColor[0];
        settings_file >> factorsColor[1];
        settings_file >> factorsColor[2];

        settings_file.close();
    } else {
        if (photoWithTimer(cap.get(), mirror, TITLE, keyAbort))
            if (calibrator.calibrate(mirror, &hsvColor, &factorsColor))
                saveCalibration(SETTINGS_FILENAME, hsvColor, factorsColor);
    }

    /************************************************************
     * Setup matrices to find colors.
     * *********************************************************/
    const uint16_t NUM_PARTICLES = 30;
    const uint8_t MAX_WEIGHT = 30;
    const uint8_t MAX_DISTANCE = 10;
    uint16_t mat_size = HEIGHT / MAX_DISTANCE;
    std::unique_ptr<std::vector<std::vector<ParticleWeighting>
    > > weightingMatrix(
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
                                    factorsColor,
                                    hsvColor))));

    for (int i = 0; i < HEIGHT / MAX_DISTANCE; ++i) {
        for (int j = 0; j < WIDTH / MAX_DISTANCE; ++j) {
            (*weightingMatrix)[i][j].setMinHeight(i * MAX_DISTANCE);
            (*weightingMatrix)[i][j].setMaxHeight((i + 1) * MAX_DISTANCE);
            (*weightingMatrix)[i][j].setMinWidth(j * MAX_DISTANCE);
            (*weightingMatrix)[i][j].setMaxWidth((j + 1) * MAX_DISTANCE);
            (*weightingMatrix)[i][j].update();
        }
    }

    /******************************************************************
     * Define circle data.
     * ***************************************************************/
    uint8_t targetRadius = 40;

    /********************************************************
     * Circle and timer stuff. (Circles have timers)
     * *****************************************************/
    std::vector<float> stateTimes(3, 1);
    stateTimes[2] = 3;

    std::unique_ptr<CircleHandler> posHandler(
        new CircleHandler(
            7,
            targetRadius,
            stateTimes,
            cv::Scalar(0, 254, 0),
            WIDTH,
            HEIGHT));

    for (uint8_t i = 0; i < stateTimes.size(); ++i)
        stateTimes[i] = 5;

    targetRadius = 30;
    std::unique_ptr<CircleHandler> negHandler(
        new CircleHandler(5,
                          targetRadius,
                          stateTimes,
                          cv::Scalar(0, 0, 254),
                          WIDTH,
                          HEIGHT));

    /*******************************************************
     * Let the game start.
     * ****************************************************/
    uint8_t keyCode;
    const uint8_t CODE_END = 1;
    const uint8_t CODE_CALIBRATE = 2;
    bool run = true;
    const uint8_t CORNER_2_CENTER = MAX_DISTANCE / 2;

    while (run) {
        keyCode = gameplay(
                      cap.get(),
                      mirror,
                      TITLE,
                      CORNER_2_CENTER,
                      posHandler.get(),
                      negHandler.get(),
                      MAX_DISTANCE,
                      weightingMatrix.get());

        switch (keyCode) {
        case CODE_END:
            run = false;
            break;
        case CODE_CALIBRATE:
            if (photoWithTimer(
                    cap.get(),
                    mirror,
                    TITLE,
                    keyAbort))
                if (calibrator.calibrate(mirror, &hsvColor, &factorsColor)) {
                    saveCalibration(SETTINGS_FILENAME, hsvColor, factorsColor);

                    for (int i = 0; i < HEIGHT / MAX_DISTANCE; ++i) {
                        for (int j = 0; j < WIDTH / MAX_DISTANCE; ++j) {
                            (*weightingMatrix)[i][j].setHsv(hsvColor);
                            (*weightingMatrix)[i][j].setFactors(factorsColor);
                        }
                    }
                }
            break;
        }
    }

    cv::destroyAllWindows();
    delete mirror;

    return 0;
}

uint8_t gameplay(
    cv::VideoCapture *cap,
    cv::Mat *mirror,
    const std::string &title,
    const uint8_t corner2center,
    CircleHandler *posHandler,
    CircleHandler *negHandler,
    const uint8_t maxDistance,
    std::vector<std::vector
    <ParticleWeighting>> *weightingMatrix)
{
    cv::Mat frame;
    cv::Mat hsv;
    uint8_t *pixelPtr_hsv;
    cv::Scalar color;
    const cv::Scalar BAD_COLOR(0, 0, 255);
    const cv::Scalar GOOD_COLOR(255, 0, 0);
    std::random_device rd;
    std::mt19937 eng(rd());
    int16_t key;
    bool goodArea;
    float leftSeconds;
    int16_t hits = 0;

    const int16_t KEY_ESC = 27;
    const int16_t KEY_C = 99;
    const int16_t KEY_R = 114;
    const uint8_t gameTime = 60;

    clock_t timeStart = clock();

    while (true) {
        *cap >> frame;
        cv::flip(frame, *mirror, 1);
        cvtColor(*mirror, hsv, cv::COLOR_BGR2HSV);
        pixelPtr_hsv = reinterpret_cast<uint8_t*>(hsv.data);

        posHandler->updateCircles(std::move(mirror));
        negHandler->updateCircles(std::move(mirror));

        for (uint32_t i = 0; i < (*weightingMatrix).size(); ++i) {
            for (uint32_t j = 0; j < (*weightingMatrix)[i].size(); ++j) {
                goodArea = (*weightingMatrix)[i][j].isColor(pixelPtr_hsv);

                if (goodArea) {
                    color = GOOD_COLOR;
                    cv::line(
                        *mirror,
                        cv::Point(
                            j * maxDistance + corner2center,
                            i * maxDistance + corner2center),
                        cv::Point(
                            j * maxDistance + corner2center,
                            i * maxDistance + corner2center),
                        color,
                        10,
                        10);

                    hits += posHandler->checkHit(
                                j * maxDistance + corner2center,
                                i * maxDistance + corner2center);
                    hits -= 5 * negHandler->checkHit(
                                j * maxDistance + corner2center,
                                i * maxDistance + corner2center);
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

        leftSeconds = gameTime - static_cast<float>(clock() - timeStart)
                      / CLOCKS_PER_SEC;

        cv::putText(
            *mirror,
            "Time left= " + std::to_string(leftSeconds),
            cv::Point(200, mirror->rows - 10),
            cv::FONT_HERSHEY_SIMPLEX,
            1.2,
            cv::Scalar(255,255,0));

        if (cv::getWindowProperty(title, cv::WND_PROP_VISIBLE)) {
            cv::imshow(title, *mirror);
            key = cv::waitKey(1);

            if (key == KEY_R)
                return 0;
            else if (key == KEY_ESC)
                break;
            else if (key == KEY_C)
                return 2;
        }
        if (leftSeconds <= 0) break;
    }

    /******************************************************
     * Get actual highscore and show it.
     * ***************************************************/
    const std::string HIGHSCORE_FILENAME = "highscore.txt";
    int16_t highscore = loadHighscore(HIGHSCORE_FILENAME, hits);

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

// Take photo after 3 seconds.
bool photoWithTimer(
    cv::VideoCapture *cap,
    cv::Mat *image,
    const std::string &title,
    int16_t keyAbort)
{
    const uint8_t minTimePassed = 3;
    cv::Mat frame;
    int16_t key;

    double timeStart = cv::getTickCount();
    int8_t leftSeconds = minTimePassed - (cv::getTickCount() - timeStart)
                         / cv::getTickFrequency();

    while (leftSeconds > 0) {
        *cap >> frame;
        cv::flip(frame, *image, 1);

        leftSeconds = minTimePassed - (cv::getTickCount() - timeStart)
                      / cv::getTickFrequency();

        cv::putText(
            *image,
            "Countdown= " + std::to_string(leftSeconds),
            cv::Point(10, image->rows - 10),
            cv::FONT_HERSHEY_SIMPLEX,
            1.2,
            cv::Scalar(255, 255, 0),
            2);

        cv::imshow(title, *image);
        key = cv::waitKey(1);

        if (key == keyAbort)
            return false;
    }

    *cap >> frame;
    cv::flip(frame, *image, 1);
    return true;
}

int16_t loadHighscore(const std::string &filename, int16_t hits)
{
    struct stat buffer;
    int16_t highscore;

    bool alreadyExits = stat(filename.c_str(), &buffer) == 0;

    if (alreadyExits) {
        std::ifstream readHighscoreFile;
        readHighscoreFile.open(filename);

        readHighscoreFile >> highscore;

        readHighscoreFile.close();

        if (highscore < hits) {
            std::ofstream writeHighscoreFile;
            writeHighscoreFile.open(filename);

            writeHighscoreFile << hits;

            writeHighscoreFile.close();
            return hits;
        } else {
            return highscore;
        }
    } else {
        std::ofstream writeHighscoreFile;
        writeHighscoreFile.open(filename);

        writeHighscoreFile << hits;

        writeHighscoreFile.close();

        return hits;
    }
}

void saveCalibration(
    const std::string &filename,
    const std::vector<uint16_t> &hsvColor,
    const std::vector<double> &factorsColor)
{
    std::ofstream save_file;
    save_file.open(filename);

    save_file << unsigned(hsvColor[0]) << std::endl;
    save_file << unsigned(hsvColor[1]) << std::endl;
    save_file << unsigned(hsvColor[2]) << std::endl;
    save_file << factorsColor[0] << std::endl;
    save_file << factorsColor[1] << std::endl;
    save_file << factorsColor[2] << std::endl;

    save_file.close();
    std::cout << "Saved" << std::endl;
}
