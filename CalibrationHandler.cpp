/**
* CalibrationHandler.cpp
*
* Take picture after a short time,
* drag and drop over it and calibrate.
*/

#include "CalibrationHandler.h"

CalibrationHandler::CalibrationHandler(std::string title, uint16_t distanceText2Border,
                                       uint8_t font, float textScale,
                                       cv::Scalar textColor, uint8_t textThickness)
    : title_ (title), distanceText2Border_(distanceText2Border),
      font_(font),textScale_(textScale), textColor_(textColor),
      textThickness_(textThickness)
{
    hsvColor_ = {0,0,0};
    factorsColor_ = {1000000,1000000,1000000};

    hsvPtr = 0;

    negDist = 2;
    posDist = 1;

    badColor = cv::Scalar(0,0,255);
    goodColor = cv::Scalar(0,255,0);

    startValueFactors = 0.0003;
    minError = 50;
    maxIteration = 100;


    hsvChannels = 3; // Hsv has always 3 channels!
}

CalibrationHandler::~CalibrationHandler()
{
}

void CalibrationHandler::calibrate(cv::Mat img,
                                   std::vector<uint16_t> &hsvColor,
                                   std::vector<double> &factorsColor)
{
    /***************************************************************
     * Let user select positives.
     * ************************************************************/
    img.copyTo(imgCopy);
    cv::setMouseCallback(title_, clickAndCrop, this);

    cv::putText(imgCopy,"Select ONLY good color", cv::Point(distanceText2Border_,imgCopy.rows-distanceText2Border_),
                font_, textScale_, textColor_, textThickness_);
    int key;
    std::vector<cv::Point> true_positive_square;


    while (true) {
        cv::imshow(title_, imgCopy);
        key = cv::waitKey(1);

        if (key == 114) { // r
            uint8_t max_i = square_points.size();

            for (int i = 0; i < max_i; ++i) {
                square_points.pop_back();
            }
            img.copyTo(imgCopy);
            cv::putText(imgCopy,"Select ONLY good color",
                        cv::Point(distanceText2Border_,imgCopy.rows-distanceText2Border_),
                        font_, textScale_, textColor_, textThickness_);
            cv::imshow(title_, imgCopy);
        } else if (key == 13 && square_points.size() == 2) { // enter
            true_positive_square = square_points;
            break;
        } else if (key == 27) { // esc
            return;
        }
    }

    /******************************************************************
     * Let user select non-negatives (more than all positives).
     * ***************************************************************/
    square_points.pop_back();
    square_points.pop_back();
    img.copyTo(imgCopy);
    cv::putText(imgCopy,"Select MORE than good color!",
                cv::Point(distanceText2Border_,imgCopy.rows-distanceText2Border_),
                font_, textScale_, textColor_, textThickness_);

    while (true) {
        cv::imshow(title_, imgCopy);
        key = cv::waitKey(1);

        if (key == 114) { // r
            uint8_t max_i = square_points.size();
            for (int i = 0; i < max_i; ++i) {
                square_points.pop_back();
            }
            img.copyTo(imgCopy);
            cv::putText(imgCopy,"Select MORE than good color!",
                        cv::Point(distanceText2Border_,imgCopy.rows-distanceText2Border_),
                        font_, textScale_, textColor_, textThickness_);
        } else if (key == 13 && square_points.size() == 2) { // enter
            break;
        } else if (key == 27) { // esc
            return;
        }
    }

    /*************************************************************
     * Calculate sizes for preallocation
     * **********************************************************/
    img.copyTo(imgCopy);
    cv::putText(imgCopy,"Calibrating ...",
                cv::Point(distanceText2Border_,imgCopy.rows-distanceText2Border_),
                font_, textScale_, textColor_, textThickness_);
    uint16_t smallest_x, biggest_x, smallest_y, biggest_y;

    findMinMaxXY(square_points, smallest_x, biggest_x, smallest_y, biggest_y);

    // Above square
    uint16_t num_x = ceil(imgCopy.cols/negDist);
    uint16_t num_y = ceil(smallest_y/(float)negDist);
    uint64_t numAll = num_x*num_y;

    // Below square
    num_y = ceil( (imgCopy.rows-biggest_y)/(float)negDist );
    numAll += num_x*num_y;

    // Left next to square
    num_x = ceil(smallest_x/(float)negDist);
    num_y = ceil( (biggest_y-smallest_y)/(float)negDist );
    numAll += num_x*num_y;

    // Right next to square
    num_x = ceil( (imgCopy.cols-biggest_x)/(float)negDist );
    numAll += num_x*num_y;

    // Only good values
    uint16_t g_smallest_x, g_smallest_y, g_biggest_x, g_biggest_y;
    findMinMaxXY(true_positive_square, g_smallest_x, g_biggest_x, g_smallest_y, g_biggest_y);
    true_positive_square[0].x = g_smallest_x;
    true_positive_square[1].x = g_biggest_x;
    true_positive_square[0].y = g_smallest_y;
    true_positive_square[1].y = g_biggest_y;
    num_x = ceil( (g_biggest_x-g_smallest_x)/(float)posDist);
    num_y = ceil((g_biggest_y-g_smallest_y )/(float)posDist);
    uint32_t numAllGoodValues = num_x*num_y;
    uint16_t mult = numAll/(numAllGoodValues);
    numAll += mult*numAllGoodValues;

    /**************************************************************
     * Fill matrices with wrong colors.
     * ***********************************************************/
    hsvValues = Matrix8u(numAll,3);

    uint64_t counter = 0;
    img.copyTo(imgCopy);
    cv::Mat hsv;
    cv::cvtColor(imgCopy,hsv,cv::COLOR_BGR2HSV);
    hsvPtr = (uint8_t*)hsv.data;

    for (int col = 0; col < imgCopy.cols; col+=negDist) {
        // Above square
        for (int row = 0; row < smallest_y; row+=negDist) {
            fillMatricesWithBadValues(imgCopy.cols, row, col, counter);
        }

        // Below square
        for (int row = biggest_y; row < imgCopy.rows; row += negDist) {
            fillMatricesWithBadValues(imgCopy.cols, row, col, counter);
        }
    }

    // Left next to square
    for (int row = smallest_y; row < biggest_y; row += negDist) {
        for (int col = 0; col < smallest_x; col += negDist) {
            fillMatricesWithBadValues(imgCopy.cols, row, col, counter);
        }

        // Right next to square
        for (int col = biggest_x; col < imgCopy.cols; col += negDist) {
            fillMatricesWithBadValues(imgCopy.cols, row, col, counter);
        }
    }

    /********************************************************
     * Collect positive values and create histograms.
     * *****************************************************/
    uint64_t goodValuesStart = counter;
    results = Eigen::VectorXd(numAll);
    Eigen::VectorXi hueHist = Eigen::VectorXi::Zero(256);
    Eigen::VectorXi satHist = Eigen::VectorXi::Zero(256);
    Eigen::VectorXi valHist = Eigen::VectorXi::Zero(256);

    for (uint32_t col = g_smallest_x; col < g_biggest_x; ++col) {
        for (uint32_t row = g_smallest_y; row < g_biggest_y; ++row) {
            line(imgCopy,cv::Point(col,row),cv::Point(col,row),goodColor);

            // Fill histograms.
            ++hueHist(hsvPtr[row*imgCopy.cols*hsvChannels+col*hsvChannels]);
            ++satHist(hsvPtr[row*imgCopy.cols*hsvChannels+col*hsvChannels+1]);
            ++valHist(hsvPtr[row*imgCopy.cols*hsvChannels+col*hsvChannels+2]);

            for (uint16_t m = 0; m < mult; ++m) {
                hsvValues(counter, 0) = hsvPtr[row*imgCopy.cols*hsvChannels+col*hsvChannels];
                hsvValues(counter, 1) = hsvPtr[row*imgCopy.cols*hsvChannels+col*hsvChannels + 1];
                hsvValues(counter, 2) = hsvPtr[row*imgCopy.cols*hsvChannels+col*hsvChannels + 2];
                results(counter) = 1;
                ++counter;
            }
        }
    }

    cv::imshow(title_,imgCopy);
    key = cv::waitKey(0);

    if (key == 27) { // esc or q
        return;
    }

    cv::putText(imgCopy,"Calculating ...",
                cv::Point(distanceText2Border_,imgCopy.rows-distanceText2Border_),
                font_, textScale_, textColor_, textThickness_);
    cv::imshow(title_,imgCopy);
    cv::waitKey(1); // Needed to show img.

    /**********************************************************
     * Find optimal values using histograms.
     * *******************************************************/
    int maxHue = 0;
    int maxSat = 0;
    int maxVal = 0;
    
    for (int i = 0; i < hueHist.size(); ++i) {
        if (maxHue < hueHist(i)) {
            hsvColor_(0) = i;
            maxHue = hueHist(i);
        }

        if (maxSat < satHist(i)) {
            hsvColor_(1) = i;
            maxSat = satHist(i);
        }

        if (maxVal < valHist(i)) {
            hsvColor_(2) = i;
            maxVal = valHist(i);
        }
    }

std::cout << "hsv = " << hsvColor_(0) << " "
          << hsvColor_(1) << " " << hsvColor_(2) << std::endl;

/*************************************************************
 * Start calibration for factors (tolerances) of colors
 * **********************************************************/
std::cout << "Starting calibration" << std::endl;

calculate(goodValuesStart);
std::cout << "Factors for bright = " << std::endl << factorsColor_(0)
          << " " << factorsColor_(1) << " " << factorsColor_(2) << std::endl;

/***************************************************************
 * Visualisation of bright color search and optional save.
 * ************************************************************/

img.copyTo(imgCopy);
bool acceptValues =  visualizeResult();

if (acceptValues)
{
    hsvColor[0] = hsvColor_(0);
    hsvColor[1] = hsvColor_(1);
    hsvColor[2] = hsvColor_(2);
    factorsColor[0] = factorsColor_[0];
    factorsColor[1] = factorsColor_[1];
    factorsColor[2] = factorsColor_[2];
}

*hsvPtr = 0;
}

void CalibrationHandler::clickAndCrop(int event, int x, int y, int flags, void *userdata)
{
    if (userdata != 0) {
        CalibrationHandler* handler = reinterpret_cast<CalibrationHandler*>(userdata);
        handler->clickAndCrop(event, x, y);
    }
}

void CalibrationHandler::clickAndCrop(int event, int x, int y)
{
    if (event == cv::EVENT_LBUTTONDOWN) {
        square_points.push_back(cv::Point(x,y));
    } else if (event == cv::EVENT_LBUTTONUP) {
        square_points.push_back(cv::Point(x,y));
        rectangle(imgCopy,square_points[0],square_points[1],cv::Scalar(0,0,0),2);
    }
}


void CalibrationHandler::findMinMaxXY(const std::vector<cv::Point> &square, uint16_t &smallestX,
                                      uint16_t &biggestX, uint16_t &smallestY, uint16_t &biggestY)
{
    if (square[0].x < square[1].x) {
        smallestX = square[0].x;
        biggestX = square[1].x;
    } else {
        smallestX = square[1].x;
        biggestX = square[0].x;
    }

    if (square[0].y < square[1].y) {
        smallestY = square[0].y;
        biggestY = square[1].y;
    } else {
        smallestY = square[1].y;
        biggestY = square[0].y;
    }
}

double CalibrationHandler::getProbability(const double &err2,const double &factor)
{
    return (1/(1+factor*err2));
}

double CalibrationHandler::getPrediction(const uint16_t &row, const uint16_t &col)
{
    if (hsvPtr != 0) {
        int16_t v_h = hsvColor_(0) - hsvPtr[row*imgCopy.cols*hsvChannels + col*hsvChannels];
        int16_t v_s = hsvColor_(1) - hsvPtr[row*imgCopy.cols*hsvChannels + col*hsvChannels + 1];
        int16_t v_v = hsvColor_(2) - hsvPtr[row*imgCopy.cols*hsvChannels + col*hsvChannels + 2];
        return getProbability(pow(v_h,2)*factorsColor_(0) + pow(v_s,2)*factorsColor_(1)
                              + pow(v_v,2)*factorsColor_(2),1);
    } else {
        return -1;
    }
}

void CalibrationHandler::calculate(const uint64_t &startGoodValues)
{
    factorsColor_ = {startValueFactors,startValueFactors,startValueFactors};

    double v_h;
    double v_s;
    double v_v;
    Eigen::MatrixXd A(hsvValues.rows(),hsvValues.cols());
    double sum_denominator;
    Eigen::BDCSVD<Eigen::MatrixXd> svd_solver;
    Eigen::Vector3d dx;
    uint16_t iterations = 0;

    while (iterations < maxIteration) {
        for (int i = 0; i < hsvValues.rows(); ++i) {
            v_h = hsvColor_[0] - hsvValues(i,0);
            v_s = hsvColor_[1] - hsvValues(i,1);
            v_v = hsvColor_[2] - hsvValues(i,2);
            sum_denominator = 1 + pow(v_h,2)*factorsColor_(0)
                              + pow(v_s,2)*factorsColor_(1) + pow(v_v,2)*factorsColor_(2);
            A(i,0) = -pow(v_h,2)/pow(sum_denominator,2);
            A(i,1) = -pow(v_s,2)/pow(sum_denominator,2);
            A(i,2) = -pow(v_v,2)/pow(sum_denominator,2);
        }

        dx = svd_solver.compute(A,Eigen::ComputeThinU|Eigen::ComputeThinV).solve(results);
        factorsColor_ += dx;

        if (dx(0) < 0) dx(0) *= -1;
        if (dx(1) < 0) dx(1) *= -1;
        if (dx(2) < 0) dx(2) *= -1;


        if (getFalsePositives(startGoodValues) < minError) {
            break;
        }

        ++iterations;
    }

    std::cout << "Used iterations for calculation of factors = " << unsigned(iterations) << std::endl;
}


bool CalibrationHandler::visualizeResult()
{
    uint64_t falsePositives = 0;

    for (uint16_t col = 0; col < imgCopy.cols; col += negDist) {
        // Above
        for (uint16_t row = 0; row < square_points[0].y; row += negDist) {
            addPointInImg(row, col);
            ++falsePositives;

        }

        // Below
        for (int row = square_points[1].y; row < imgCopy.rows; row += negDist) {
            addPointInImg(row, col);
            ++falsePositives;

        }
    }

    // Left
    for (int row = square_points[0].y; row < square_points[1].y; row += negDist) {
        for (int col = 0; col < square_points[0].x; col += negDist) {
            addPointInImg(row, col);
            ++falsePositives;
        }

        // Right
        for (int col = square_points[1].x; col < imgCopy.cols; col += negDist) {
            addPointInImg(row, col);
            ++falsePositives;
        }
    }

    // Good and more values
    double prediction;

    for (int col = square_points[0].x; col < square_points[1].x; col += posDist) {
        for (int row = square_points[0].y; row < square_points[1].y; row += posDist) {
            prediction = getPrediction(row,col);

            if (prediction < 0.5)
                line(imgCopy,cv::Point(col,row),cv::Point(col,row),badColor);
            else {
                line(imgCopy,cv::Point(col,row),cv::Point(col,row),goodColor);
            }
        }
    }

    std::cout << "False positives in image = " << falsePositives << std::endl;
    cv::imshow(title_,imgCopy);

    int key;

    while (true) {
        key = cv::waitKey(0);

        if (key == 115) // s
            return true;
        else if (key == 27)
            return false;

    }
}

uint64_t CalibrationHandler::getFalsePositives(const uint64_t &startGoodValues)
{
    uint64_t falsePositives = 0;
    double prediction;
    int16_t v_h;
    int16_t v_s;
    int16_t v_v;

    for (uint64_t i = 0; i < startGoodValues; ++i) {
        v_h = hsvColor_(0) - hsvValues(i,0);
        v_s = hsvColor_(1) - hsvValues(i,1);
        v_v = hsvColor_(2) - hsvValues(i,2);
        prediction = getProbability(pow(v_h,2)*factorsColor_(0)
                                    + pow(v_s,2)*factorsColor_(1)
                                    + pow(v_v,2)*factorsColor_(2),
                                    1);

        if (prediction >= 0.5)
            ++falsePositives;
    }

    return falsePositives;
}

void CalibrationHandler::fillMatricesWithBadValues(const uint16_t &hsvCols, const uint16_t &row,
        const uint16_t &col, uint64_t &counter)
{
    hsvValues(counter,0) = hsvPtr[row*hsvCols*hsvChannels+col*hsvChannels];
    hsvValues(counter,1) = hsvPtr[row*hsvCols*hsvChannels+col*hsvChannels+1];
    hsvValues(counter,2) = hsvPtr[row*hsvCols*hsvChannels+col*hsvChannels+2];

    cv::line(imgCopy,cv::Point(col,row),cv::Point(col,row),badColor);
    ++counter;

}

void CalibrationHandler::addPointInImg(const uint16_t &row, const uint16_t &col)
{
    double prediction = getPrediction(row,col);

    if (prediction < 0.5)
        line(imgCopy,cv::Point(col,row),cv::Point(col,row),badColor);
    else {
        line(imgCopy,cv::Point(col,row),cv::Point(col,row),goodColor);
    }
}
