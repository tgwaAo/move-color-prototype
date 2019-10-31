/**
* Start of a little game. The goal is to hit the bright
* green circles and avoid the bright red circles.
* The dark green and red circles are not active and
* will turn bright later.
*/


#include <opencv2/opencv.hpp>
#include <sstream>
#include <sys/stat.h>
#include <vector>
#include <random>

#include "ParticleWeighting.h"
#include "CalibrationHandler.h"
#include "CircleHandler.h"


/**
 * @brief Take a photo after 5 seconds.
 * @param cap Camera to shot photo.
 * @param image Image to save picture.
 * @param title Title shown on conting down.
 */
void photoWithTimer(cv::VideoCapture &cap, cv::Mat &image, const std::string &title);

/**
 * @brief Save calibration in a txt file.
 * @param filename Name of the calibration file.
 * @param hsvColor Vector of bright optimal values.
 * @param factorsColor Vector of bright factors.
 */
void saveCalibration(const std::string &filename,
                     const std::vector<uint16_t> &hsvColor,
                     const std::vector<double> &factorsColor);

int main()
{
    /*******************************************************************
     * Set camera up and initialize variables.
     * ****************************************************************/
    cv::VideoCapture cap(1);

    if (!cap.isOpened()) {
        return -1;
    }

    const double WIDTH = 640;
    const double HEIGHT = 480;

    cap.set(cv::CAP_PROP_FRAME_WIDTH,WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,HEIGHT);

    const std::string TITLE = "calibration";
    cv::namedWindow(TITLE,cv::WINDOW_AUTOSIZE);
    cv::resizeWindow(TITLE, WIDTH,HEIGHT);

    std::vector<uint16_t> hsvColor(3,0);
    std::vector<double> factorsColor(3,1000000);

    struct stat buffer;
    const std::string SETTINGS_FILENAME = "hsv.txt";

    /*********************************************************************
     * Load configuration or calibrate a new.
     * ******************************************************************/
    if(stat (SETTINGS_FILENAME.c_str(), &buffer) == 0) {
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
        cv::Mat image;
        photoWithTimer(cap, image, TITLE);
        CalibrationHandler calibrator(TITLE);
        calibrator.calibrate(image, hsvColor, factorsColor);
        saveCalibration(SETTINGS_FILENAME, hsvColor,factorsColor);
    }

    /************************************************************
     * Setup matrices to find colors.
     * *********************************************************/
    const uint16_t NUM_PARTICLES = 30;
    const uint8_t MAX_WEIGHT = 30;
    const uint8_t MAX_DISTANCE = 10;
    uint16_t mat_size = HEIGHT/MAX_DISTANCE;
    std::vector<std::vector<ParticleWeighting> > weightingMatrix(mat_size,
            std::vector<ParticleWeighting>(WIDTH/MAX_DISTANCE,ParticleWeighting(NUM_PARTICLES,0,10,0,10,
                                           MAX_WEIGHT,WIDTH,NUM_PARTICLES*MAX_WEIGHT*4/10, factorsColor,hsvColor)));

    for (int i = 0; i < HEIGHT/MAX_DISTANCE; ++i) {
        for (int j = 0; j < WIDTH/MAX_DISTANCE; ++j) {
            weightingMatrix[i][j].setMinHeight(i*MAX_DISTANCE);
            weightingMatrix[i][j].setMaxHeight((i+1)*MAX_DISTANCE);
            weightingMatrix[i][j].setMinWidth(j*MAX_DISTANCE);
            weightingMatrix[i][j].setMaxWidth((j+1)*MAX_DISTANCE);
            weightingMatrix[i][j].update();
        }
    }

    /******************************************************************
     * Define circle data.
     * ***************************************************************/
    uint8_t targetRadius = 30;
    std::uniform_int_distribution<uint16_t> randomUpDown(targetRadius,HEIGHT-targetRadius);
    std::uniform_int_distribution<uint16_t> randomLeftRight(targetRadius,WIDTH-targetRadius);

    /********************************************************
     * Circle and timer stuff. (Circles have timers)
     * *****************************************************/
    std::vector<float> stateTimes(3,2);
    CircleHandler posHandler(7, targetRadius, stateTimes, cv::Scalar(0,254,0), WIDTH, HEIGHT);

    for (uint8_t i = 0; i < stateTimes.size(); ++i)
        stateTimes[i] = 10;

    targetRadius = 20;
    CircleHandler negHandler(14, targetRadius, stateTimes, cv::Scalar(0,0,254), WIDTH, HEIGHT);


    /*******************************************************
     * Let the game start.
     * ****************************************************/
    cv::Mat frame;
    cv::Mat mirror;
    cv::Mat hsv;
    uint8_t* pixelPtr_hsv;
    cv::Scalar color;
    const cv::Scalar BAD_COLOR(0,0,255);
    const cv::Scalar GOOD_COLOR(255,0,0);
    const uint8_t corner2Center = MAX_DISTANCE/2;
    std::random_device rd;
    std::mt19937 eng(rd());

    int key;
    bool goodArea;

    float leftSeconds;
    const uint8_t gameTime = 60;
    int16_t hits = 0;
    clock_t timeStart = clock();

    while (true) {
        cap >> frame;
        cv::flip(frame,mirror,1);
        cvtColor(mirror,hsv,cv::COLOR_BGR2HSV);
        pixelPtr_hsv= (uint8_t*)hsv.data;

        posHandler.updateCircles(mirror);
        negHandler.updateCircles(mirror);

        for (int i = 0; i < HEIGHT/MAX_DISTANCE; ++i) {
            for (int j = 0; j < WIDTH/MAX_DISTANCE; ++j) {
                goodArea = weightingMatrix[i][j].is_color(pixelPtr_hsv);

                if (goodArea) {
                    color = GOOD_COLOR;
                    cv::line(mirror,cv::Point(j*MAX_DISTANCE+corner2Center,i*MAX_DISTANCE+corner2Center),
                             cv::Point(j*MAX_DISTANCE+corner2Center,i*MAX_DISTANCE+corner2Center),color,10,10);
                    hits += posHandler.checkHit(j*MAX_DISTANCE+corner2Center,i*MAX_DISTANCE+corner2Center);
                    hits -= negHandler.checkHit(j*MAX_DISTANCE+corner2Center,i*MAX_DISTANCE+corner2Center);
                }
            }
        }

        putText(mirror,"Hits= " + std::to_string(hits),cv::Point(10,mirror.rows-10),
                cv::FONT_HERSHEY_SIMPLEX,1.2,cv::Scalar(255,255,0));

        if (cv::getWindowProperty(TITLE,cv::WND_PROP_VISIBLE)) {
            cv::imshow(TITLE,mirror);
            key = cv::waitKey(1);

            if (key == 27) // esc
                break;
            else if (key == 99) { // 'c'
                cv::Mat image;
                photoWithTimer(cap, image, TITLE);
                CalibrationHandler calibrator(TITLE);
                calibrator.calibrate(image,hsvColor,factorsColor);
                saveCalibration(SETTINGS_FILENAME, hsvColor,factorsColor);

                for (int i = 0; i < HEIGHT/MAX_DISTANCE; ++i) {
                    for (int j = 0; j < WIDTH/MAX_DISTANCE; ++j) {
                        weightingMatrix[i][j].setHsv(hsvColor);
                        weightingMatrix[i][j].setFactors(factorsColor);
                    }
                }

                timeStart = clock(); // New start of game timer.
                hits = 0; // Reset points gained in game.
            }
        }

        leftSeconds = gameTime - (float)(clock() - timeStart)/CLOCKS_PER_SEC;

        if (leftSeconds <= 0)
            break;
    }

    /******************************************************
     * Get actual highscore and show it.
     * ***************************************************/
    cv::putText(mirror, "Finished",cv::Point(20,mirror.rows/2),cv::FONT_HERSHEY_SIMPLEX,
                1.2,cv::Scalar(255,255,0));

    const std::string HIGHTSCORE_FILENAME = "highscore.txt";
    int16_t highscore;
    bool alreadyExits = stat (HIGHTSCORE_FILENAME.c_str(), &buffer) == 0;

    if (alreadyExits) {
        std::ifstream readHighscoreFile;
        readHighscoreFile.open(HIGHTSCORE_FILENAME);

        readHighscoreFile >> highscore;

        readHighscoreFile.close();

        if (highscore < hits) {
            highscore = hits;

            std::ofstream writeHighscoreFile;
            writeHighscoreFile.open(HIGHTSCORE_FILENAME);

            writeHighscoreFile << hits;

            writeHighscoreFile.close();
        }
    } else {
        highscore = hits;

        std::ofstream writeHighscoreFile;
        writeHighscoreFile.open(HIGHTSCORE_FILENAME);

        writeHighscoreFile << hits;

        writeHighscoreFile.close();
    }

    cv::putText(mirror, "Highscore = " + std::to_string(highscore),cv::Point(20,30+mirror.rows/2),
                cv::FONT_HERSHEY_SIMPLEX,1.2,cv::Scalar(255,255,0));
    cv::imshow(TITLE,mirror);

    while (true) {
        if (cv::getWindowProperty(TITLE,cv::WND_PROP_VISIBLE)) {
            if (cv::waitKey(100) != -1)
                break;
        } else
            break;
    }

    cv::destroyAllWindows();

    return 0;
}

// Take photo after 5 seconds.
void photoWithTimer(cv::VideoCapture &cap, cv::Mat &image, const std::string &title)
{
    /***********************************************************
    *  Wait a few seconds to give the user time to get in position.
    * ********************************************************/
    const uint8_t minTimePassed = 5;
    clock_t timeStart = clock();
    int8_t leftSeconds = minTimePassed - (float)(clock() - timeStart)/CLOCKS_PER_SEC;
    cv::Mat frame;

    while (leftSeconds > 0) {
        cap >> frame;
        cv::flip(frame,image,1);
        leftSeconds = minTimePassed - (float)(clock() - timeStart)/CLOCKS_PER_SEC;
        cv::putText(image,"Countdown= " + std::to_string(leftSeconds),
                    cv::Point(10,image.rows-10),cv::FONT_HERSHEY_SIMPLEX,1.2,cv::Scalar(255,255,0),2);
        cv::imshow(title,image);
        cv::waitKey(1);
    }

    cap >> frame;
    cv::flip(frame,image,1);
}

void saveCalibration(const std::string &filename,
                     const std::vector<uint16_t> &hsvColor,
                     const std::vector<double> &factorsColor)
{
    std::ofstream save_file;
    save_file.open(filename);

    save_file << unsigned(hsvColor[0]) << std::endl;
    save_file << unsigned(hsvColor[1]) << std::endl;
    save_file << unsigned(hsvColor[2]) << std::endl;
    save_file << factorsColor[0] << std::endl;
    save_file << factorsColor[1]<< std::endl;
    save_file << factorsColor[2] << std::endl;

    save_file.close();
    std::cout << "Saved" << std::endl;
}
