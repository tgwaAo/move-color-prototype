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
    hsvColor_ = {0,0,0}; // Just some really wrong values.
    factorsColor_ = {1000000,1000000,1000000}; // Just some really wrong values.

    hsvPtr = 0;

    negDist = 2;
    posDist = 1;

    badColor = cv::Scalar(0,0,255);
    goodColor = cv::Scalar(0,255,0);

    startValueFactors = 0.0003;
    minError = 50;
    maxIteration = 100;
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
    cv::setMouseCallback(title_, clickAndCrop, this);

    img.copyTo(imgCopy);
    cv::putText(imgCopy,"Select ONLY good color", cv::Point(distanceText2Border_,imgCopy.rows-distanceText2Border_),
                font_, textScale_, textColor_, textThickness_);
    int key;
    std::vector<cv::Point> truePositiveSquare;
    const uint8_t WAIT_TIME = 20;

    while (true) {
        cv::imshow(title_, imgCopy);

        if (squarePoints.size() == POINTS_OF_RECTANGLE) { // Draw rectangle.
            img.copyTo(imgCopy);
            cv::putText(imgCopy,"Select ONLY good color", cv::Point(distanceText2Border_,imgCopy.rows-distanceText2Border_),
                        font_, textScale_, textColor_, textThickness_);
            cv::rectangle(imgCopy, squarePoints[0], squarePoints[1],cv::Scalar(0,0,0),2);
        }

        key = cv::waitKey(WAIT_TIME);

        if (key == KEY_R) {
            for (int i = 0; i < POINTS_OF_RECTANGLE; ++i) {
                squarePoints.pop_back();
            }
            img.copyTo(imgCopy);
            cv::putText(imgCopy,"Select ONLY good color",
                        cv::Point(distanceText2Border_,imgCopy.rows-distanceText2Border_),
                        font_, textScale_, textColor_, textThickness_);
        } else if (key == KEY_ACCEPT) {
            truePositiveSquare = squarePoints;
            break;
        } else if (key == KEY_ESC) {
            return;
        }
    }

    /******************************************************************
     * Let user select non-negatives (more than all positives).
     * ***************************************************************/
    squarePoints.pop_back();
    squarePoints.pop_back();
    img.copyTo(imgCopy);
    cv::putText(imgCopy,"Select MORE than good color!",
                cv::Point(distanceText2Border_,imgCopy.rows-distanceText2Border_),
                font_, textScale_, textColor_, textThickness_);

    while (true) {
        cv::imshow(title_, imgCopy);

        if (squarePoints.size() == POINTS_OF_RECTANGLE) {
            img.copyTo(imgCopy);
            cv::putText(imgCopy,"Select MORE than good color!",
                        cv::Point(distanceText2Border_,imgCopy.rows-distanceText2Border_),
                        font_, textScale_, textColor_, textThickness_);
            cv::rectangle(imgCopy, squarePoints[0],squarePoints[1],cv::Scalar(0,0,0),2);
        }

        key = cv::waitKey(WAIT_TIME);

        if (key == KEY_R) {
            for (int i = 0; i < POINTS_OF_RECTANGLE; ++i) {
                squarePoints.pop_back();
            }
            img.copyTo(imgCopy);
            cv::putText(imgCopy,"Select MORE than good color!",
                        cv::Point(distanceText2Border_,imgCopy.rows-distanceText2Border_),
                        font_, textScale_, textColor_, textThickness_);
        } else if (key == KEY_ACCEPT) {
            break;
        } else if (key == KEY_ESC) {
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
    uint16_t smallestX, biggestX, smallestY, biggestY;

    findMinMaxXY(squarePoints, smallestX, biggestX, smallestY, biggestY);

    // Above square
    uint16_t numX = ceil(imgCopy.cols/negDist);
    uint16_t numY = ceil(smallestY/(float)negDist);
    uint64_t numAll = numX * numY;

    // Below square
    numY = ceil( (imgCopy.rows-biggestY)/(float)negDist );
    numAll += numX * numY;

    // Left next to square
    numX = ceil(smallestX/(float)negDist);
    numY = ceil( (biggestY-smallestY)/(float)negDist );
    numAll += numX * numY;

    // Right next to square
    numX = ceil( (imgCopy.cols-biggestX)/(float)negDist );
    numAll += numX * numY;

    // Only good values
    uint16_t gSmallestX, gSmallestY, gBiggestX, gBiggestY;
    findMinMaxXY(truePositiveSquare, gSmallestX, gSmallestY, gBiggestX, gBiggestY);
    truePositiveSquare[0].x = gSmallestX;
    truePositiveSquare[1].x = gBiggestX;
    truePositiveSquare[0].y = gSmallestY;
    truePositiveSquare[1].y = gBiggestY;
    numX = ceil( (gBiggestX - gSmallestX)/(float)posDist);
    numY = ceil((gBiggestY - gSmallestY )/(float)posDist);
    uint32_t numAllGoodValues = numX * numY;
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
        for (int row = 0; row < smallestY; row+=negDist) {
            fillMatricesWithBadValues(imgCopy.cols, row, col, counter);
        }

        // Below square
        for (int row = biggestY; row < imgCopy.rows; row += negDist) {
            fillMatricesWithBadValues(imgCopy.cols, row, col, counter);
        }
    }

    // Left next to square
    for (int row = smallestY; row < biggestY; row += negDist) {
        for (int col = 0; col < smallestX; col += negDist) {
            fillMatricesWithBadValues(imgCopy.cols, row, col, counter);
        }

        // Right next to square
        for (int col = biggestX; col < imgCopy.cols; col += negDist) {
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

    for (uint32_t col = gSmallestX; col < gBiggestX; ++col) {
        for (uint32_t row = gSmallestY; row < gBiggestY; ++row) {
            line(imgCopy,cv::Point(col,row),cv::Point(col,row),goodColor);

            // Fill histograms.
            ++hueHist(hsvPtr[row*imgCopy.cols*HSV_CHANNELS+col*HSV_CHANNELS]);
            ++satHist(hsvPtr[row*imgCopy.cols*HSV_CHANNELS+col*HSV_CHANNELS+1]);
            ++valHist(hsvPtr[row*imgCopy.cols*HSV_CHANNELS+col*HSV_CHANNELS+2]);

            for (uint16_t m = 0; m < mult; ++m) {
                hsvValues(counter, 0) = hsvPtr[row*imgCopy.cols*HSV_CHANNELS+col*HSV_CHANNELS];
                hsvValues(counter, 1) = hsvPtr[row*imgCopy.cols*HSV_CHANNELS+col*HSV_CHANNELS + 1];
                hsvValues(counter, 2) = hsvPtr[row*imgCopy.cols*HSV_CHANNELS+col*HSV_CHANNELS + 2];
                results(counter) = 1;
                ++counter;
            }
        }
    }

    cv::imshow(title_,imgCopy);
    key = cv::waitKey(WAIT_TIME);

    if (key == KEY_ESC) {
        return;
    }

    cv::putText(imgCopy,"Calculating ...",
                cv::Point(distanceText2Border_,imgCopy.rows-distanceText2Border_),
                font_, textScale_, textColor_, textThickness_);
    cv::imshow(title_,imgCopy);
    cv::waitKey(WAIT_TIME);

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

    if (acceptValues) {
        hsvColor[0] = hsvColor_(0);
        hsvColor[1] = hsvColor_(1);
        hsvColor[2] = hsvColor_(2);
        factorsColor[0] = factorsColor_[0];
        factorsColor[1] = factorsColor_[1];
        factorsColor[2] = factorsColor_[2];
    }

    *hsvPtr = 0;
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

double CalibrationHandler::getProbability(const double err2)
{
    return (1 / (1 + err2) );
}

double CalibrationHandler::getPrediction(const uint16_t &row, const uint16_t &col)
{
    if (hsvPtr != 0) {
        int16_t errHue = hsvColor_(0) - hsvPtr[row*imgCopy.cols*HSV_CHANNELS + col*HSV_CHANNELS];
        int16_t errSat = hsvColor_(1) - hsvPtr[row*imgCopy.cols*HSV_CHANNELS + col*HSV_CHANNELS + 1];
        int16_t errVal = hsvColor_(2) - hsvPtr[row*imgCopy.cols*HSV_CHANNELS + col*HSV_CHANNELS + 2];
        return getProbability(pow(errHue,2)*factorsColor_(0) + pow(errSat,2)*factorsColor_(1)
                              + pow(errVal,2)*factorsColor_(2));
    } else {
        return -1;
    }
}

void CalibrationHandler::calculate(const uint64_t &startGoodValues)
{
    factorsColor_ = {startValueFactors,startValueFactors,startValueFactors};

    double errHue;
    double errSat;
    double errVal;
    Eigen::MatrixXd A(hsvValues.rows(),hsvValues.cols());
    double sum_denominator;
    Eigen::BDCSVD<Eigen::MatrixXd> svd_solver;
    Eigen::Vector3d dx;
    uint16_t iterations = 0;

    while (iterations < maxIteration) {
        for (int i = 0; i < hsvValues.rows(); ++i) {
            errHue = hsvColor_[0] - hsvValues(i,0);
            errSat = hsvColor_[1] - hsvValues(i,1);
            errVal = hsvColor_[2] - hsvValues(i,2);
            sum_denominator = 1 + pow(errHue,2)*factorsColor_(0)
                              + pow(errSat,2)*factorsColor_(1) + pow(errVal,2)*factorsColor_(2);
            A(i,0) = -pow(errHue,2)/pow(sum_denominator,2);
            A(i,1) = -pow(errSat,2)/pow(sum_denominator,2);
            A(i,2) = -pow(errVal,2)/pow(sum_denominator,2);
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
        for (uint16_t row = 0; row < squarePoints[0].y; row += negDist) {
            addPointInImg(row, col);
            ++falsePositives;

        }

        // Below
        for (int row = squarePoints[1].y; row < imgCopy.rows; row += negDist) {
            addPointInImg(row, col);
            ++falsePositives;

        }
    }

    // Left
    for (int row = squarePoints[0].y; row < squarePoints[1].y; row += negDist) {
        for (int col = 0; col < squarePoints[0].x; col += negDist) {
            addPointInImg(row, col);
            ++falsePositives;
        }

        // Right
        for (int col = squarePoints[1].x; col < imgCopy.cols; col += negDist) {
            addPointInImg(row, col);
            ++falsePositives;
        }
    }

    // Good and more values
    double prediction;

    for (int col = squarePoints[0].x; col < squarePoints[1].x; col += posDist) {
        for (int row = squarePoints[0].y; row < squarePoints[1].y; row += posDist) {
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

        if (key == KEY_S)
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
                                    + pow(v_v,2)*factorsColor_(2));

        if (prediction >= 0.5)
            ++falsePositives;
    }

    return falsePositives;
}

void CalibrationHandler::fillMatricesWithBadValues(const uint16_t &hsvCols, const uint16_t &row,
        const uint16_t &col, uint64_t &counter)
{
    hsvValues(counter,0) = hsvPtr[row*hsvCols*HSV_CHANNELS+col*HSV_CHANNELS];
    hsvValues(counter,1) = hsvPtr[row*hsvCols*HSV_CHANNELS+col*HSV_CHANNELS+1];
    hsvValues(counter,2) = hsvPtr[row*hsvCols*HSV_CHANNELS+col*HSV_CHANNELS+2];

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

void CalibrationHandler::clickAndCrop(int event, int x, int y, int flags, void *userdata)
{
    if (userdata != 0) {
        CalibrationHandler* handler = reinterpret_cast<CalibrationHandler*>(userdata);
        handler->clickAndCrop(event, x, y);
    }
}

void CalibrationHandler::clickAndCrop(int event, int x, int y)
{
    if (event == cv::EVENT_LBUTTONDOWN && squarePoints.size() < 2) {
        squarePoints.push_back(cv::Point(x,y));
        squarePoints.push_back(cv::Point(x,y));
        drawActive = true;
    } else if (event == cv::EVENT_LBUTTONUP) {
        drawActive = false;
    }

    if (event == cv::EVENT_MOUSEMOVE && drawActive) {
        if (squarePoints.size() == 2) {
            squarePoints[1] = cv::Point(x,y);
        }
    }
}
