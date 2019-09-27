#include "CircleHandler.h"

CircleHandler::CircleHandler(const uint8_t &numCircles, const uint8_t &radius_,
                             const std::vector<float> &circleTimeStates_,
                             const uint8_t &colorIdx_,
                             uint8_t status, const uint16_t width,
                             const uint16_t height)
{
    std::random_device rd;
    eng = std::mt19937(rd());
    randomLeftRight = std::uniform_int_distribution<uint16_t>(radius_, width - radius_);
    randomUpDown = std::uniform_int_distribution<uint16_t>(radius_, height - radius_);

    allCircles.resize(numCircles,Circle {cv::Point(0,0),clock(),status});

    std::uniform_int_distribution<uint8_t> randomState(0,2);
    
    for (uint8_t i = 0; i < allCircles.size(); ++i) {
        allCircles[i].point.x = randomLeftRight(eng);
        allCircles[i].point.y = randomUpDown(eng);
        allCircles[i].status = randomState(eng);
    }

    circleTimeStates = circleTimeStates_;
    radius = radius_;
    colorIdx = colorIdx_;
}

CircleHandler::~CircleHandler()
{
}

void CircleHandler::updateCircles(cv::Mat &img)
{
    float leftSeconds;
    cv::Scalar color(0,0,0);
    for (uint8_t i = 0; i < allCircles.size(); ++i) {
        leftSeconds = circleTimeStates[allCircles[i].status] - (float)(clock() - allCircles[i].timerStart)/CLOCKS_PER_SEC;

        if (leftSeconds <= 0) {
            if(allCircles[i].status == 2)
                newPos(i);
            else {
                allCircles[i].status += 1;
                allCircles[i].timerStart = clock();
            }
        }

        if (allCircles[i].status != 0) {
            color.val[colorIdx] = 127*allCircles[i].status;
            cv::circle(img,allCircles[i].point,radius,
                       color,-1);
        }
    }
}

uint8_t CircleHandler::checkHit(const uint16_t &x, const uint16_t &y)
{
    double dist;
    uint16_t hits = 0;

    for (uint8_t i = 0; i < allCircles.size(); ++i) {
        if (allCircles[i].status == 2) {
            dist = sqrt( pow(x-allCircles[i].point.x,2)+pow(y-allCircles[i].point.y,2) ) ;

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
    allCircles[idx].point.x = randomLeftRight(eng);
    allCircles[idx].point.y = randomUpDown(eng);
    allCircles[idx].timerStart = clock();
    allCircles[idx].status = 0;
}
