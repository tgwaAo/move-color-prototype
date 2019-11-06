#include "CircleHandler.h"

CircleHandler::CircleHandler()
{
    CircleHandler(10, 10, std::vector<float>(3,3), 0, 640, 480);
}

CircleHandler::CircleHandler(const uint8_t numCircles, const uint8_t radius,
                             const std::vector<float> &circleTimeStates,
                             const cv::Scalar& color,
                             const uint16_t width,
                             const uint16_t height)
{
    if (circleTimeStates.size() == sizeOfTimeStates) {
        circleTimeStates_ = circleTimeStates;
    } else {
        circleTimeStates_ = std::vector<float>(3,10);
    }

    radius_ = radius;
    color_ = color;

    std::random_device rd;
    eng = std::mt19937(rd());
    // Use an index to select different positions.
    randomLeftRightIdx = std::uniform_int_distribution<uint16_t>(0, std::round((width-2*radius_) / (2*radius_)));
    randomUpDownIdx = std::uniform_int_distribution<uint16_t>(0, std::round((height-2*radius_) / (2*radius_)));

    allCircles.resize(numCircles,Circle {cv::Point(0,0),clock(),0});

    std::uniform_int_distribution<uint8_t> randomState(0,2);

    for (uint8_t i = 0; i < allCircles.size(); ++i) {
        allCircles[i].point.x = radius_ + 2*radius_*randomLeftRightIdx(eng);
        allCircles[i].point.y = radius_ + 2*radius_*randomUpDownIdx(eng);
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
        leftSeconds = circleTimeStates_[allCircles[i].status] - (float)(clock() - allCircles[i].timerStart)/CLOCKS_PER_SEC;

        if (leftSeconds <= 0) {
            if(allCircles[i].status == 2)
                newPos(i);
            else {
                allCircles[i].status += 1;
                allCircles[i].timerStart = clock();
            }
        }

        if (allCircles[i].status != 0) {
            usedColor.val[0] = color_.val[0] * 0.5 * allCircles[i].status;
            usedColor.val[1] = color_.val[1] * 0.5 * allCircles[i].status;
            usedColor.val[2] = color_.val[2] * 0.5 * allCircles[i].status;
            cv::circle(*img,allCircles[i].point,radius_,
                       usedColor,-1);
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

            if (dist <= radius_) {
                ++hits;
                newPos(i);
            }
        }
    }

    return hits;
}

void CircleHandler::newPos(uint8_t idx)
{
    allCircles[idx].point.x = radius_ + 2*radius_*randomLeftRightIdx(eng);
    allCircles[idx].point.y = radius_ + 2*radius_*randomUpDownIdx(eng);
    allCircles[idx].timerStart = clock();
    allCircles[idx].status = 0;
}

void CircleHandler::setCircleTimeStates(const std::vector<float>& circleTimeStates)
{
    if (circleTimeStates.size() == sizeOfTimeStates) {
        circleTimeStates_ = circleTimeStates;
    }
}

void CircleHandler::setColor(const cv::Scalar& color)
{
    color_ = color;
}

void CircleHandler::setRadius(const uint8_t radius)
{
    radius_ = radius;
}

float CircleHandler::getCircleTimeState(uint8_t idx) const
{
    return circleTimeStates_[idx];
}

cv::Scalar CircleHandler::getColor() const
{
    return color_;
}

uint8_t CircleHandler::getRadius() const
{
    return radius_;
}
