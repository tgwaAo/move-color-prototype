#ifndef CIRCLEHANDLER_H
#define CIRCLEHANDLER_H

#include <opencv2/opencv.hpp>
#include <random>

class updateCircles;
class CircleHandler
{
public:
    CircleHandler(const uint8_t &numCircles, const uint8_t &radius_,
                  const std::vector<float> &circleTimeStates,
                  const uint8_t &colorIdx_,
                  uint8_t status = 0, const uint16_t width = 640,
                  const uint16_t height = 480);
    ~CircleHandler();
    void updateCircles(cv::Mat &img);
    uint8_t checkHit(const uint16_t &x, const uint16_t &y);
    struct Circle {
        cv::Point point;
        clock_t timerStart;
        uint8_t status;
    };
    
private:
    uint8_t radius;
    uint8_t colorIdx;
    std::mt19937 eng;
    std::uniform_int_distribution<uint16_t> randomUpDown;
    std::uniform_int_distribution<uint16_t> randomLeftRight;
    std::vector<Circle> allCircles;
    std::vector<float> circleTimeStates;
    
    void newPos(uint8_t idx);
};

#endif // CIRCLEHANDLER_H
