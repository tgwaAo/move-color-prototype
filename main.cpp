#include <opencv2/opencv.hpp>
#include <sstream>
#include <sys/stat.h>
#include <vector>
#include <random>

#include "ParticleWeighting.h"
#include "Calibrate.h"
#include "CircleHandler.h"

using namespace std;
using namespace cv;

const string settings_filename = "hsv.txt";
const uint16_t NUM_PARTICLES = 30;
const double WIDTH = 640;
const double HEIGHT = 480;
const uint8_t MAX_WEIGHT = 30;
const uint8_t MAX_DISTANCE = 10;

bool check_hit(int is_x, int is_y, Point goal, uint8_t goal_radius);

int main()
{
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
//        return -2;
        calibrate(cap, TITLE, WIDTH, HEIGHT,settings_filename,
                  hsvBright, factorsBright, hsvDark, factorsDark);
    }




    //particle filter
    uint16_t mat_size = HEIGHT/MAX_DISTANCE;
    vector<vector<ParticleWeighting> > weightingMatrixBright(mat_size,
            vector<ParticleWeighting>(WIDTH/MAX_DISTANCE,ParticleWeighting(NUM_PARTICLES,0,10,0,10,
                                      MAX_WEIGHT,WIDTH,NUM_PARTICLES*MAX_WEIGHT*4/10, factorsBright,hsvBright)));
    vector<vector<ParticleWeighting> > weightingMatrixDark(mat_size,
            vector<ParticleWeighting>(WIDTH/MAX_DISTANCE,ParticleWeighting(NUM_PARTICLES,0,10,0,10,
                                      MAX_WEIGHT,WIDTH,NUM_PARTICLES*MAX_WEIGHT*4/10, factorsDark,hsvDark)));

    for (int i = 0; i < HEIGHT/MAX_DISTANCE; ++i) {
        for (int j = 0; j < WIDTH/MAX_DISTANCE; ++j) {
            weightingMatrixBright[i][j].setMin_height(i*MAX_DISTANCE);
            weightingMatrixBright[i][j].setMax_height((i+1)*MAX_DISTANCE);
            weightingMatrixBright[i][j].setMin_width(j*MAX_DISTANCE);
            weightingMatrixBright[i][j].setMax_width((j+1)*MAX_DISTANCE);
            weightingMatrixBright[i][j].update();

            weightingMatrixDark[i][j].setMin_height(i*MAX_DISTANCE);
            weightingMatrixDark[i][j].setMax_height((i+1)*MAX_DISTANCE);
            weightingMatrixDark[i][j].setMin_width(j*MAX_DISTANCE);
            weightingMatrixDark[i][j].setMax_width((j+1)*MAX_DISTANCE);
            weightingMatrixDark[i][j].update();
        }
    }

    int key;
    Point target(300,200);
    uint8_t targetRadius = 40;
    bool goodAreaBright;
    bool goodAreaDark;
//    bool hit;
    bool run = true;
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
     * Timer stuff.
     * *****************************************************/
    const uint8_t minTimePassed = 10;
    float leftSeconds;
    std::vector<float> stateTimes(3,2);
    CircleHandler posHandler(10, targetRadius, stateTimes, 1, 1, WIDTH, HEIGHT);

    for (uint8_t i = 0; i < stateTimes.size(); ++i)
        stateTimes[i] = 4;

    CircleHandler negHandler(10, targetRadius, stateTimes, 2, 2, WIDTH, HEIGHT);

    clock_t timeStart = clock();


    while (run) {
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
//                    hit = check_hit(j*MAX_DISTANCE+corner2Center, i*MAX_DISTANCE+corner2Center, target, targetRadius);
                    hits += posHandler.checkHit(j*MAX_DISTANCE+corner2Center,i*MAX_DISTANCE+corner2Center);
                    hits -= negHandler.checkHit(j*MAX_DISTANCE+corner2Center,i*MAX_DISTANCE+corner2Center);
//                    if (hit) {
//                        ++hits;
//                        target.x = randomLeftRight(eng);
//                        target.y = randomUpDown(eng);
//                    }
                }
            }
        }

//        circle(mirror,target,targetRadius,BAD_COLOR,-1);
        putText(mirror,"Hits= " + to_string(hits),Point(10,mirror.rows-10),FONT_HERSHEY_SIMPLEX,1.2,Scalar(255,255,0));

        if (cv::getWindowProperty(TITLE,cv::WND_PROP_VISIBLE)) {
            cv::imshow(TITLE,mirror);
            key = cv::waitKey(1);

            if (key == 27) // esc
                break;
            else if (key == 99) { // 'c'
                calibrate(cap, TITLE, WIDTH, HEIGHT,settings_filename,
                          hsvBright, factorsBright, hsvDark, factorsDark);
                for (int i = 0; i < HEIGHT/MAX_DISTANCE; ++i) {
                    for (int j = 0; j < WIDTH/MAX_DISTANCE; ++j) {
                        weightingMatrixBright[i][j].setHsv(hsvBright);
                        weightingMatrixBright[i][j].setFactors(factorsBright);
                        weightingMatrixDark[i][j].setHsv(hsvDark);
                        weightingMatrixDark[i][j].setFactors(factorsDark);
                    }
                }
            }
        }

        leftSeconds = minTimePassed - (float)(clock() - timeStart)/CLOCKS_PER_SEC;

        if (leftSeconds <= 0)
            break;
    }

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

//    bool check_hit(int is_x, int is_y, Point goal, uint8_t goal_radius) {
//        double dist = sqrt( pow(is_x-goal.x,2)+pow(is_y-goal.y,2) ) ;
//
//        if (dist <= goal_radius)
//            return true;
//        else
//            return false;
//    }
