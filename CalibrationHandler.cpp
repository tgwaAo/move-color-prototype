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
    hsvBright_ = {0,0,0};
    hsvDark_ = {0,0,0};
    factorsBright_ = {1000000,1000000,1000000};
    factorsDark_ = {1000000,1000000,1000000};

    hsvPtr = 0;

    negDist = 2;
    posDist = 1;

    badColor = cv::Scalar(0,0,255);
    goodColor = cv::Scalar(0,255,0);

    startValueFactors = 0.0005;
    minError = 50;
    maxIteration = 100;


    hsvChannels = 3; // Hsv has always 3 channels!
}

CalibrationHandler::~CalibrationHandler()
{
}

void CalibrationHandler::calibrate(cv::Mat img,
                                   std::vector<uint16_t> &hsvBright, std::vector<double> &factorsBright,
                                   std::vector<uint16_t> &hsvDark, std::vector<double> &factorsDark)
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
    square_points[0].x = smallest_x;
    square_points[1].x = biggest_x;
    square_points[0].y = smallest_y;
    square_points[1].y = biggest_y;

    // Above square
    uint16_t num_x = ceil(imgCopy.cols/negDist);
    uint16_t num_y = ceil(smallest_y/(float)negDist);
    uint64_t numAllBrightAndDark = num_x*num_y;

    // Below square
    num_y = ceil( (imgCopy.rows-biggest_y)/(float)negDist );
    numAllBrightAndDark += num_x*num_y;

    // Left next to square
    num_x = ceil(smallest_x/(float)negDist);
    num_y = ceil( (biggest_y-smallest_y)/(float)negDist );
    numAllBrightAndDark += num_x*num_y;

    // Right next to square
    num_x = ceil( (imgCopy.cols-biggest_x)/(float)negDist );
    numAllBrightAndDark += num_x*num_y;

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
    uint32_t offsetBrightAndDark = numAllGoodValues/2;
    uint32_t numUpperAndLowerPart = offsetBrightAndDark*0.8;
    uint16_t mult = numAllBrightAndDark/(numUpperAndLowerPart);
    numAllBrightAndDark += mult*numUpperAndLowerPart;

    /**************************************************************
     * Fill matrices with wrong colors and collect right colors.
     * ***********************************************************/
    hsvValuesBright = Matrix8u(numAllBrightAndDark,3);
    hsvValuesDark =  Matrix8u(numAllBrightAndDark,3);

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

    // Collect right colors separately.
    uint64_t goodValuesStart = counter;
    counter = 0;
    allGoodValues.resize(numAllGoodValues,Matrix8u(1,hsvChannels));

    // Get all good values
    for (int row = g_smallest_y; row < g_biggest_y; row += posDist) {
        for (int col = g_smallest_x; col < g_biggest_x; col += posDist) {
            line(imgCopy,cv::Point(col,row),cv::Point(col,row),goodColor);
            allGoodValues[counter](0,0) = hsvPtr[row*hsv.cols*hsv.channels()+col*hsv.channels()];
            allGoodValues[counter](0,1) = hsvPtr[row*hsv.cols*hsv.channels()+col*hsv.channels()+1];
            allGoodValues[counter](0,2) = hsvPtr[row*hsv.cols*hsv.channels()+col*hsv.channels()+2];
            ++counter;
        }
    }

    cv::imshow(title_,imgCopy);
    key = cv::waitKey(0);

    if (key == 27 || key == 113) { // esc or q
        return;
    }

    /*********************************************************************
     * Sort good values with respect to brightness.
     * Leave brightest and darkest values out and fill matrices.
     * *******************************************************************/
    cv::putText(imgCopy,"Calculating ...",
                cv::Point(distanceText2Border_,imgCopy.rows-distanceText2Border_),
                font_, textScale_, textColor_, textThickness_);
    cv::imshow(title_,imgCopy);
    cv::waitKey(1); // Needed to show img.

    std::sort(allGoodValues.begin(), allGoodValues.end(),[](const Matrix8u& lhs, const Matrix8u& rhs) {
        return lhs(0,2) < rhs(0,2);
    });
    counter = goodValuesStart;
    results = Eigen::VectorXd(numAllBrightAndDark);


    for (uint32_t row = 0; row < numUpperAndLowerPart; ++row) {
        for (uint16_t m = 0; m < mult; ++m) {
            hsvValuesBright.row(counter) = allGoodValues[offsetBrightAndDark+row];
            hsvValuesDark.row(counter) = allGoodValues[offsetBrightAndDark-row];
            results(counter) = 1;
            ++counter;
        }
    }

    /**********************************************************
     * Find median values to have a refenrence color.
     * *******************************************************/
    getMedianValues(offsetBrightAndDark,numUpperAndLowerPart);

    std::cout << "Bright hsv= " << hsvBright_(0) << " "
              << hsvBright_(1) << " " << hsvBright_(2) << std::endl;
    std::cout << "Dark hsv= " << hsvDark_(0)
              << " " << hsvDark_(1) << " " << hsvDark_(2) << std::endl;

    /*************************************************************
     * Start calibration for factors (tolerances) of colors
     * **********************************************************/
    std::cout << "Starting calibration" << std::endl;

    factorsBright_ = {startValueFactors,startValueFactors,startValueFactors};
    calculate(hsvBright_,factorsBright_, hsvValuesBright,
              results, "bright", goodValuesStart);
    std::cout << "Factors for bright = " << std::endl << factorsBright_(0)
              << factorsBright_(1) << factorsBright_(2) << std::endl;

    factorsDark_ = {startValueFactors,startValueFactors,startValueFactors};
    calculate(hsvDark_,factorsDark_,hsvValuesDark,
              results, "dark", goodValuesStart);
    std::cout << "Factors for dark = " << std::endl << factorsDark_(0)
              << factorsDark_(1) << factorsDark_(2) << std::endl;

    /***************************************************************
     * Visualisation of bright color search and optional save.
     * ************************************************************/

    img.copyTo(imgCopy);
    bool acceptValues =  visualizeResult();

    if (acceptValues) {
        hsvBright[0] = hsvBright_(0);
        hsvBright[1] = hsvBright_(1);
        hsvBright[2] = hsvBright_(2);
        factorsBright[0] = factorsBright_[0];
        factorsBright[1] = factorsBright_[1];
        factorsBright[2] = factorsBright_[2];
        hsvDark[0] = hsvDark_(0);
        hsvDark[1] = hsvDark_(1);
        hsvDark[2] = hsvDark_(2);
        factorsDark[0] = factorsDark_(0);
        factorsDark[1] = factorsDark_(1);
        factorsDark[2] = factorsDark_(2);
    }

    *hsvPtr = 0;
}


int CalibrationHandler::median(Eigen::VectorXi &v)
{
    size_t n = v.size() / 2;
    std::nth_element(v.data(), v.data()+n, v.data()+v.size());
    return v(n);
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

        int16_t v_h = hsvValuesBright(0) - hsvPtr[row*imgCopy.cols*hsvChannels + col*hsvChannels];
        int16_t v_s = hsvValuesBright(1) - hsvPtr[row*imgCopy.cols*hsvChannels + col*hsvChannels + 1];
        int16_t v_v = hsvValuesBright(2) - hsvPtr[row*imgCopy.cols*hsvChannels + col*hsvChannels + 2];
        double trustBright = getProbability(pow(v_h,2)*factorsBright_(0) + pow(v_s,2)*factorsBright_(1)
                                            + pow(v_v,2)*factorsBright_(2),1);

        v_h = hsvValuesDark(0) - hsvPtr[row*imgCopy.cols*hsvChannels + col*hsvChannels];
        v_s = hsvValuesDark(1) - hsvPtr[row*imgCopy.cols*hsvChannels + col*hsvChannels + 1];
        v_v = hsvValuesDark(2) - hsvPtr[row*imgCopy.cols*hsvChannels + col*hsvChannels + 2];
        double trustDark = getProbability(pow(v_h,2)*factorsDark_(0) + pow(v_s,2)*factorsDark_(1)
                                          + pow(v_v,2)*factorsDark_(2),1);

        if (trustBright > trustDark)
            return trustBright;
        else
            return trustDark;
    } else {
        return -1;
    }
}

void CalibrationHandler::getMedianValues(const uint32_t &offset, const uint32_t &numElements)
{
    Eigen::VectorXi colorVec(numElements);

    // Bright hue
    for (int i = 0; i < colorVec.size(); ++i) {
        colorVec(i) = allGoodValues[offset+i](0,0);
    }

    hsvBright_(0) = median(colorVec);

    // Bright saturation
    for (int i = 0; i < colorVec.size(); ++i) {
        colorVec(i) = allGoodValues[offset+i](0,1);
    }

    hsvBright_(1) = median(colorVec);

    // Bright value
    for (int i = 0; i < colorVec.size(); ++i) {
        colorVec(i) = allGoodValues[offset+i](0,2);
    }

    hsvBright_(2) = median(colorVec);

    // Dark hue
    for (int i = 0; i < colorVec.size(); ++i) {
        colorVec(i) = allGoodValues[offset-i](0,0);
    }

    hsvDark_(0) = median(colorVec);

    // Dark saturation
    for (int i = 0; i < colorVec.size(); ++i) {
        colorVec(i) = allGoodValues[offset-i](0,1);
    }

    hsvDark_(1) = median(colorVec);

    // Dark value
    for (int i = 0; i < colorVec.size(); ++i) {
        colorVec(i) = allGoodValues[offset-i](0,2);
    }

    hsvDark_(2) = median(colorVec);
}

void CalibrationHandler::calculate(const Eigen::Vector3i &hsvBrightOrDark, Eigen::Vector3d &x0,
                                   const Matrix8u &hsvValues, const Eigen::VectorXd &results,
                                   const std::string &description, const uint64_t &startGoodValues)
{
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
            v_h = hsvBrightOrDark[0] - hsvValues(i,0);
            v_s = hsvBrightOrDark[1] - hsvValues(i,1);
            v_v = hsvBrightOrDark[2] - hsvValues(i,2);
            sum_denominator = 1 + pow(v_h,2)*x0(0)
                              + pow(v_s,2)*x0(1) + pow(v_v,2)*x0(2);
            A(i,0) = -pow(v_h,2)/pow(sum_denominator,2);
            A(i,1) = -pow(v_s,2)/pow(sum_denominator,2);
            A(i,2) = -pow(v_v,2)/pow(sum_denominator,2);
        }

        dx = svd_solver.compute(A,Eigen::ComputeThinU|Eigen::ComputeThinV).solve(results);
        x0 += dx;

        if (dx(0) < 0) dx(0) *= -1;
        if (dx(1) < 0) dx(1) *= -1;
        if (dx(2) < 0) dx(2) *= -1;


        if (getFalsePositives(hsvBrightOrDark,x0,hsvValues,startGoodValues) < minError) {
            break;
        }

        ++iterations;
    }

    std::cout << "Used iterations for " << description << "= " << unsigned(iterations) << std::endl;
}


bool CalibrationHandler::visualizeResult()
{
    uint64_t falsePositives = 0;

    for (uint16_t col = 0; col < imgCopy.cols; col += negDist) {
        // Above
        for (uint16_t row = 0; row < square_points[0].y; row += negDist) {
            addNegPointInImg(row, col, falsePositives);
        }

        // Below
        for (int row = square_points[1].y; row < imgCopy.rows; row += negDist) {
            addNegPointInImg(row, col, falsePositives);
        }
    }

    // Left
    for (int row = square_points[0].y; row < square_points[1].y; row += negDist) {
        for (int col = 0; col < square_points[0].x; col += negDist) {
            addNegPointInImg(row, col, falsePositives);
        }

        // Right
        for (int col = square_points[1].x; col < imgCopy.cols; col += negDist) {
            addNegPointInImg(row, col, falsePositives);
        }
    }

    // Good Values
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
    int key = cv::waitKey(0);

    if (key == 115) // s
        return true;
    else
        return false;
}

uint64_t CalibrationHandler::getFalsePositives(const Eigen::Vector3i &hsv,const Eigen::Vector3d &factors,
        const Matrix8u &hsvValues,const uint64_t &startGoodValues)
{
    uint64_t falsePositives = 0;
    double prediction;
    int16_t v_h;
    int16_t v_s;
    int16_t v_v;

    for (uint64_t i = 0; i < startGoodValues; ++i) {
        v_h = hsv(0) - hsvValues(i,0);
        v_s = hsv(1) - hsvValues(i,1);
        v_v = hsv(2) - hsvValues(i,2);
        prediction = getProbability(pow(v_h,2)*factors(0) + pow(v_s,2)*factors(1) + pow(v_v,2)*factors(2),1);

        if (prediction >= 0.5)
            ++falsePositives;
    }

    return falsePositives;
}

void CalibrationHandler::fillMatricesWithBadValues(const uint16_t &hsvCols, const uint16_t &row,
        const uint16_t &col, uint64_t &counter)
{
    hsvValuesBright(counter,0) = hsvPtr[row*hsvCols*hsvChannels+col*hsvChannels];
    hsvValuesBright(counter,1) = hsvPtr[row*hsvCols*hsvChannels+col*hsvChannels+1];
    hsvValuesBright(counter,2) = hsvPtr[row*hsvCols*hsvChannels+col*hsvChannels+2];

    hsvValuesDark(counter,0) = hsvPtr[row*hsvCols*hsvChannels+col*hsvChannels];
    hsvValuesDark(counter,1) = hsvPtr[row*hsvCols*hsvChannels+col*hsvChannels+1];
    hsvValuesDark(counter,2) = hsvPtr[row*hsvCols*hsvChannels+col*hsvChannels+2];

    cv::line(imgCopy,cv::Point(col,row),cv::Point(col,row),badColor);
    ++counter;

}

void CalibrationHandler::addNegPointInImg(const uint16_t &row, const uint16_t &col, uint64_t &falsePositives)
{
    double prediction = getPrediction(row,col);

    if (prediction < 0.5)
        line(imgCopy,cv::Point(col,row),cv::Point(col,row),badColor);
    else {
        line(imgCopy,cv::Point(col,row),cv::Point(col,row),goodColor);
        ++falsePositives;
    }
}
