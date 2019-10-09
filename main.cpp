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


using namespace std;
using namespace cv;


const string settings_filename = "hsv.txt";
const uint16_t NUM_PARTICLES = 30;
const double WIDTH = 640;
const double HEIGHT = 480;
const uint8_t MAX_WEIGHT = 30;
const uint8_t MAX_DISTANCE = 10;

/**
 * @brief Take a photo after 5 seconds.
 * @param cap Camera to shot photo.
 * @param image Image to save picture.
 * @param title Title shown on conting down.
 */
void photoWithTimer(cv::VideoCapture &cap, cv::Mat &image, const std::string &title);


int main()
{
    /*******************************************************************
     * Set camera up and initialize variables.
     * ****************************************************************/
    VideoCapture cap(0);

    if (!cap.isOpened()) {
        return -1;
    }

    cap.set(CAP_PROP_FRAME_WIDTH,WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT,HEIGHT);

    const string TITLE = "calibration";
    namedWindow(TITLE,WINDOW_AUTOSIZE);
    resizeWindow(TITLE, WIDTH,HEIGHT);

    vector<uint16_t> hsvBright(3,0);
    vector<double> factorsBright(3,1000000);
    vector<uint16_t> hsvDark(3,0);
    vector<double> factorsDark(3,1000000);

    struct stat buffer;
    bool exists = stat (settings_filename.c_str(), &buffer) == 0;
    CalibrationHandler calibrator(TITLE);
    cv::Mat image;

    /*********************************************************************
     * Load configuration or calibrate a new.
     * ******************************************************************/
    if(exists) {
        ifstream settings_file;
        settings_file.open(settings_filename);

        settings_file >> hsvBright[0];
        settings_file >> hsvBright[1];
        settings_file >> hsvBright[2];
        settings_file >> factorsBright[0];
        settings_file >> factorsBright[1];
        settings_file >> factorsBright[2];
        settings_file >> hsvDark[0];
        settings_file >> hsvDark[1];
        settings_file >> hsvDark[2];
        settings_file >> factorsDark[0];
        settings_file >> factorsDark[1];
        settings_file >> factorsDark[2];

        settings_file.close();
    } else {
        photoWithTimer(cap, image, TITLE);
        calibrator.calibrate(image, hsvBright, factorsBright, hsvDark, factorsDark);

        std::ofstream save_file;
        save_file.open(settings_filename);

        save_file << unsigned(hsvBright[0]) << std::endl;
        save_file << unsigned(hsvBright[1]) << std::endl;
        save_file << unsigned(hsvBright[2]) << std::endl;
        save_file << factorsBright[0] << std::endl;
        save_file << factorsBright[1]<< std::endl;
        save_file << factorsBright[2] << std::endl;
        save_file << unsigned(hsvDark[0]) << std::endl;
        save_file << unsigned(hsvDark[1]) << std::endl;
        save_file << unsigned(hsvDark[2]) << std::endl;
        save_file << factorsDark[0] << std::endl;
        save_file << factorsDark[1]<< std::endl;
        save_file << factorsDark[2] << std::endl;

        save_file.close();
        std::cout << "Saved" << std::endl;
    }

    /************************************************************
     * Setup matrices to find colors.
     * *********************************************************/
    uint16_t mat_size = HEIGHT/MAX_DISTANCE;
    vector<vector<ParticleWeighting> > weightingMatrixBright(mat_size,
            vector<ParticleWeighting>(WIDTH/MAX_DISTANCE,ParticleWeighting(NUM_PARTICLES,0,10,0,10,
                                      MAX_WEIGHT,WIDTH,NUM_PARTICLES*MAX_WEIGHT*4/10, factorsBright,hsvBright)));
    vector<vector<ParticleWeighting> > weightingMatrixDark(mat_size,
            vector<ParticleWeighting>(WIDTH/MAX_DISTANCE,ParticleWeighting(NUM_PARTICLES,0,10,0,10,
                                      MAX_WEIGHT,WIDTH,NUM_PARTICLES*MAX_WEIGHT*4/10, factorsDark,hsvDark)));

    for (int i = 0; i < HEIGHT/MAX_DISTANCE; ++i) {
        for (int j = 0; j < WIDTH/MAX_DISTANCE; ++j) {
            weightingMatrixBright[i][j].setMinHeight(i*MAX_DISTANCE);
            weightingMatrixBright[i][j].setMaxHeight((i+1)*MAX_DISTANCE);
            weightingMatrixBright[i][j].setMinWidth(j*MAX_DISTANCE);
            weightingMatrixBright[i][j].setMaxWidth((j+1)*MAX_DISTANCE);
            weightingMatrixBright[i][j].update();

            weightingMatrixDark[i][j].setMinHeight(i*MAX_DISTANCE);
            weightingMatrixDark[i][j].setMaxHeight((i+1)*MAX_DISTANCE);
            weightingMatrixDark[i][j].setMinWidth(j*MAX_DISTANCE);
            weightingMatrixDark[i][j].setMaxWidth((j+1)*MAX_DISTANCE);
            weightingMatrixDark[i][j].update();
        }
    }

    /******************************************************************
     * Define circle data.
     * ***************************************************************/
    int key;
    Point target(300,200);
    uint8_t targetRadius = 30;
    bool goodAreaBright;
    bool goodAreaDark;
    Mat hsv;
    Mat frame;
    uint8_t* pixelPtr_hsv;
    Scalar color;
    Scalar BAD_COLOR(0,0,255);
    Scalar GOOD_COLOR(255,0,0);
    uint8_t hits = 0;
    uint8_t corner2Center = MAX_DISTANCE/2;
    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<uint16_t> randomUpDown(targetRadius,HEIGHT-targetRadius);
    std::uniform_int_distribution<uint16_t> randomLeftRight(targetRadius,WIDTH-targetRadius);
    cv::Mat mirror;

    /********************************************************
     * Circle and timer stuff. (Circles have timers)
     * *****************************************************/
    const uint8_t minTimePassed = 60;
    float leftSeconds;
    std::vector<float> stateTimes(3,3);
    CircleHandler posHandler(7, targetRadius, stateTimes, 1, WIDTH, HEIGHT);

    for (uint8_t i = 0; i < stateTimes.size(); ++i)
        stateTimes[i] = 10;

    targetRadius = 20;
    
    CircleHandler negHandler(10, targetRadius, stateTimes, 2, WIDTH, HEIGHT);

    clock_t timeStart = clock();

    /*******************************************************
     * Let the game start.
     * ****************************************************/
    while (true) {
        cap >> frame;
        cv::flip(frame,mirror,1);
        cvtColor(mirror,hsv,COLOR_BGR2HSV);
        pixelPtr_hsv= (uint8_t*)hsv.data;

        posHandler.updateCircles(mirror);
        negHandler.updateCircles(mirror);

        for (int i = 0; i < HEIGHT/MAX_DISTANCE; ++i) {
            for (int j = 0; j < WIDTH/MAX_DISTANCE; ++j) {
                goodAreaBright = weightingMatrixBright[i][j].is_color(pixelPtr_hsv);
                goodAreaDark = weightingMatrixDark[i][j].is_color(pixelPtr_hsv);
                if (goodAreaBright || goodAreaDark) {
                    color = GOOD_COLOR;
                    line(mirror,Point(j*MAX_DISTANCE+corner2Center,i*MAX_DISTANCE+corner2Center),
                         Point(j*MAX_DISTANCE+corner2Center,i*MAX_DISTANCE+corner2Center),color,10,10);
                    hits += posHandler.checkHit(j*MAX_DISTANCE+corner2Center,i*MAX_DISTANCE+corner2Center);
                    hits -= negHandler.checkHit(j*MAX_DISTANCE+corner2Center,i*MAX_DISTANCE+corner2Center);
                }
            }
        }

        putText(mirror,"Hits= " + to_string(hits),Point(10,mirror.rows-10),FONT_HERSHEY_SIMPLEX,1.2,Scalar(255,255,0));

        if (cv::getWindowProperty(TITLE,cv::WND_PROP_VISIBLE)) {
            cv::imshow(TITLE,mirror);
            key = cv::waitKey(1);

            if (key == 27) // esc
                break;
            else if (key == 99) { // 'c'
                photoWithTimer(cap, image, TITLE);
                calibrator.calibrate(image,hsvBright,factorsBright,hsvDark,factorsDark);

                for (int i = 0; i < HEIGHT/MAX_DISTANCE; ++i) {
                    for (int j = 0; j < WIDTH/MAX_DISTANCE; ++j) {
                        weightingMatrixBright[i][j].setHsv(hsvBright);
                        weightingMatrixBright[i][j].setFactors(factorsBright);
                        weightingMatrixDark[i][j].setHsv(hsvDark);
                        weightingMatrixDark[i][j].setFactors(factorsDark);
                    }
                }
                
                timeStart = clock(); // New start of game timer.
                hits = 0; // Reset points gained in game.
            }
        }

        leftSeconds = minTimePassed - (float)(clock() - timeStart)/CLOCKS_PER_SEC;

        if (leftSeconds <= 0)
            break;
    }

    // Show result.
    cv::putText(mirror, "Finished",cv::Point(20,mirror.rows/2),cv::FONT_HERSHEY_SIMPLEX,1.2,cv::Scalar(255,255,0));
    cv::imshow(TITLE,mirror);
    
    while (true) {
        if (cv::getWindowProperty(TITLE,cv::WND_PROP_VISIBLE)) {
            if (cv::waitKey(100) != -1)
                break;
        } else
            break;
    }

    destroyAllWindows();

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
                    cv::Point(10,image.rows-10),cv::FONT_HERSHEY_SIMPLEX,1.2,Scalar(255,255,0),2);
        cv::imshow(title,image);
        cv::waitKey(1);
    }

    cap >> frame;
    cv::flip(frame,image,1);
}
