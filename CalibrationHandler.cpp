/**
* Take picture after a short time,
* drag and drop over it and calibrate.
*/

#include "CalibrationHandler.h"

CalibrationHandler::CalibrationHandler(std::string title = "Calibration", uint16_t distanceText2Border = 10,
                       uint8_t font = cv::FONT_HERSHEY_SIMPLEX, float textScale = 1.2,
                       cv::Scalar textColor(255,255,0), uint8_t textThickness = 2)
{
    title_ = title;
    distanceText2Border_ = distanceText2Border;
    font_ = font;
    textScale_ = textScale;
    textColor_ = textColor;
    textThickness_ = textThickness;
}

CalibrationHandler::~CalibrationHandler()
{
    // do not forget delete!!
}

void CalibrationHandler::calibrate(cv::Mat img,
                           const std::string &settings_filename,
                           std::vector<uint16_t> &hsvBright, std::vector<double> &factorsBright,
                           std::vector<uint16_t> &hsvDark, std::vector<double> &factorsDark)
{
    /***************************************************************
     * Let user select positives.
     * ************************************************************/
    cv::Mat mirror_copy;
    img.copyTo(mirror_copy);
    img.copyTo(mirror);
    cv::setMouseCallback(title,click_and_crop);

    cv::putText(mirror,"Select ONLY good color",cv::Point(10,mirror.rows-10),
                cv::FONT_HERSHEY_SIMPLEX,1.2,textColor,2);
    int key;
    std::vector<cv::Point> true_positive_square;


    while (true) {
        cv::imshow(title, mirror);
        key = cv::waitKey(1);

        if (key == 114) { // r
            uint8_t max_i = square_points.size();

            for (int i = 0; i < max_i; ++i) {
                square_points.pop_back();
            }
            mirror_copy.copyTo(mirror);
            cv::putText(mirror,"Select ONLY good color",cv::Point(10,mirror.rows-10),
                        cv::FONT_HERSHEY_SIMPLEX,1.2,cv::Scalar(255,255,0));
            cv::imshow(title, mirror);
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
    mirror_copy.copyTo(mirror);
    cv::putText(mirror,"Select MORE than good color!",cv::Point(10,mirror.rows-10),
                cv::FONT_HERSHEY_SIMPLEX,1.2,textColor,2);

    while (true) {
        cv::imshow(title, mirror);
        key = cv::waitKey(1);

        if (key == 114) { // r
            uint8_t max_i = square_points.size();
            for (int i = 0; i < max_i; ++i) {
                square_points.pop_back();
            }
            mirror_copy.copyTo(mirror);
            cv::putText(mirror,"Select MORE than good color!",
                        cv::Point(10,mirror.rows-10),cv::FONT_HERSHEY_SIMPLEX,1.2,textColor,2);
        } else if (key == 13 && square_points.size() == 2) { // enter
            break;
        } else if (key == 27) { // esc
            return;
        }
    }


    /*************************************************************
     * Calculate sizes for preallocation
     * **********************************************************/
    mirror_copy.copyTo(mirror);
    cv::putText(mirror,"Calibrating ...",cv::Point(10,mirror.rows-10),
                cv::FONT_HERSHEY_SIMPLEX,1.2,textColor,2);
    uint16_t smallest_x, biggest_x, smallest_y, biggest_y;

    find_minmax_xy(square_points, smallest_x, biggest_x, smallest_y, biggest_y);
    square_points[0].x = smallest_x;
    square_points[1].x = biggest_x;
    square_points[0].y = smallest_y;
    square_points[1].y = biggest_y;
    square_points.push_back(cv::Point(step_x,step_y));
    // Above square
    uint16_t num_x = ceil(mirror.cols/step_x);
    uint16_t num_y = ceil(smallest_y/(float)step_y);
    uint64_t numAllBrightAndDark = num_x*num_y;
    // Below square
    num_y = ceil( (mirror.rows-biggest_y)/(float)step_y );
    numAllBrightAndDark += num_x*num_y;
    // Left next to square
    num_x = ceil(smallest_x/(float)step_x);
    num_y = ceil( (biggest_y-smallest_y)/(float)step_y );
    numAllBrightAndDark += num_x*num_y;
    // Right next to square
    num_x = ceil( (mirror.cols-biggest_x)/(float)step_x );
    numAllBrightAndDark += num_x*num_y;

    // Only good values
    uint16_t g_smallest_x, g_smallest_y, g_biggest_x, g_biggest_y;
    find_minmax_xy(true_positive_square, g_smallest_x, g_biggest_x, g_smallest_y, g_biggest_y);
    true_positive_square[0].x = g_smallest_x;
    true_positive_square[1].x = g_biggest_x;
    true_positive_square[0].y = g_smallest_y;
    true_positive_square[1].y = g_biggest_y;
    num_x = ceil( (g_biggest_x-g_smallest_x)/(float)POS_DIST);
    num_y = ceil((g_biggest_y-g_smallest_y )/(float)POS_DIST);
    uint32_t numAllGoodValues = num_x*num_y;
    uint32_t offsetBrightAndDark = numAllGoodValues/2;
    uint32_t numUpperAndLowerPart = offsetBrightAndDark*0.8;
    uint16_t mult = numAllBrightAndDark/(numUpperAndLowerPart);
    numAllBrightAndDark += mult*numUpperAndLowerPart;

    /**************************************************************
     * Fill matrices with wrong colors and collect right colors.
     * ***********************************************************/
    hsvValuesBright = Matrix8u(numAllBrightAndDark,3);
    hsvValuesDark = Matrix8u(numAllBrightAndDark,3);
    uint64_t counter = 0;
    mirror_copy.copyTo(mirror);
    cv::Mat hsv;
    cv::cvtColor(mirror,hsv,cv::COLOR_BGR2HSV);
    *hsv_ptr = (uint8_t*)hsv.data;
    hsvChannels = hsv.channels();

    for (int col = 0; col < mirror.cols; col+=step_x) {
        // Above square
        for (int row = 0; row < smallest_y; row+=step_y) {
            fillAllBadMatrices(mirror.cols, row, col, counter);
        }

        // Below square
        for (int row = biggest_y; row < mirror.rows; row += step_y) {
            fillAllBadMatrices(mirror.cols, row, col, counter);
        }
    }

    // Left next to square
    for (int row = smallest_y; row < biggest_y; row += step_y) {
        for (int col = 0; col < smallest_x; col += step_x) {
            fillAllBadMatrices(mirror.cols, row, col, counter);
        }

        // Right next to square
        for (int col = biggest_x; col < mirror.cols; col += step_x) {
            fillAllBadMatrices(mirror.cols, row, col, counter);
        }
    }

    // Collect right colors separately.
    uint64_t goodValuesStart = counter;
    counter = 0;
    allGoodValues.resize(numAllGoodValues,Matrix8u(1,hsvChannels));

    // Get all good values
    for (int row = g_smallest_y; row < g_biggest_y; row += POS_DIST) {
        for (int col = g_smallest_x; col < g_biggest_x; col += POS_DIST) {
            line(mirror,cv::Point(col,row),cv::Point(col,row),goodColorBright);
            allGoodValues[counter](0,0) = hsv_ptr[row*hsv.cols*hsv.channels()+col*hsv.channels()];
            allGoodValues[counter](0,1) = hsv_ptr[row*hsv.cols*hsv.channels()+col*hsv.channels()+1];
            allGoodValues[counter](0,2) = hsv_ptr[row*hsv.cols*hsv.channels()+col*hsv.channels()+2];
            ++counter;
        }
    }

    cv::imshow(title,mirror);
    key = cv::waitKey(0);

    if (key == 27 || key == 113) { // esc or q
        return;
    }

    cv::putText(mirror,"Calculating ...",cv::Point(10,mirror.rows-10),
                cv::FONT_HERSHEY_SIMPLEX,1.2,cv::Scalar(255,255,0),2);
    cv::imshow(title,mirror);
    cv::waitKey(1); // Needed to show img.

    /*********************************************************************
     * Sort good values with respect to brightness.
     * Leave brightest and darkest values out and fill matrices.
     * *******************************************************************/
    std::sort(allGoodValues.begin(), allGoodValues.end(), &compareBrightness);
    counter = goodValuesStart;
    Eigen::VectorXd results(numAllBrightAndDark);


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
    getMedianValues(hsvBrightIntern,hsvValuesBright,goodValuesStart,counter);
    std::cout << "Bright hsv= " << hsvBrightIntern(0) << " "
              << hsvBrightIntern(1) << " " << hsvBrightIntern(2) << std::endl;

    getMedianValues(hsvDarkIntern,hsvValuesDark,goodValuesStart,counter);
    std::cout << "Dark hsv= " << hsvDarkIntern(0)
              << " " << hsvDarkIntern(1) << " " << hsvDarkIntern(2) << std::endl;

    /*************************************************************
     * Start calibration for factors (tolerances) of colors
     * **********************************************************/
    std::cout << "Starting calibration" << std::endl;

    factorsBrightIntern = {START_VALUE_FACTORS,START_VALUE_FACTORS,START_VALUE_FACTORS};
    calculate(hsvDarkIntern,factorsBrightIntern,hsvValuesBright, "bright",
              results,MIN_ERROR,MAX_ITERATION,goodValuesStart);
    std::cout << "Factors for bright = " << std::endl << factorsBrightIntern(0)
              << factorsBrightIntern(1) << factorsBrightIntern(2) << std::endl;

    factorsDarkIntern = {START_VALUE_FACTORS,START_VALUE_FACTORS,START_VALUE_FACTORS};
    calculate(hsvDarkIntern,factorsDarkIntern,hsvValuesDark, "dark",
              results,MIN_ERROR,MAX_ITERATION,goodValuesStart);
    std::cout << "Factors for dark = " << std::endl << factorsDarkIntern(0)
              << factorsDarkIntern(1) << factorsDarkIntern(2) << std::endl;

    /***************************************************************
     * Visualisation of bright color search and optional save.
     * ************************************************************/

    mirror_copy.copyTo(mirror);
    bool shouldSave =  visualizeResult(mirror);

    if (shouldSave) {
        hsvBright = hsvBrightIntern;
        factorsBright = factorsBrightIntern;
        hsvDark = hsvDarkIntern;
        factorsDark = factorsDarkIntern;

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
}


int CalibrationHandler::median(Eigen::VectorXi &v)
{
    size_t n = v.size() / 2;
    std::nth_element(v.data(), v.data()+n, v.data()+v.size());
    return v(n);
}

void CalibrationHandler::click_and_crop(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN) {
        square_points.push_back(cv::Point(x,y));
    } else if (event == cv::EVENT_LBUTTONUP) {
        square_points.push_back(cv::Point(x,y));
        rectangle(mirror,square_points[0],square_points[1],cv::Scalar(0,0,0),2);
    }
}

void CalibrationHandler::find_minmax_xy(const std::vector<cv::Point> &square, uint16_t &smallest_x,
                                uint16_t &biggest_x, uint16_t &smallest_y, uint16_t &biggest_y)
{
    if (square[0].x < square[1].x) {
        smallest_x = square[0].x;
        biggest_x = square[1].x;
    } else {
        smallest_x = square[1].x;
        biggest_x = square[0].x;
    }

    if (square[0].y < square[1].y) {
        smallest_y = square[0].y;
        biggest_y = square[1].y;
    } else {
        smallest_y = square[1].y;
        biggest_y = square[0].y;
    }
}

double CalibrationHandler::get_trust(const double &v,const double &factor)
{
    return (1/(1+factor*v));
}

double CalibrationHandler::getPrediction(std::vector<uint16_t> &hsvValuesBright,
                                 std::vector<double> &factors, const uint8_t *hsv_ptr,
                                 const uint16_t &row, const uint16_t &col,
                                 const uint8_t &hsvChannels, const int &mirror.cols)
{
    int16_t v_h = hsvValuesBright[0] - hsv_ptr[row*mirror.cols*hsvChannels + col*hsvChannels];
    int16_t v_s = hsvValuesBright[1] - hsv_ptr[row*mirror.cols*hsvChannels + col*hsvChannels + 1];
    int16_t v_v = hsvValuesBright[2] - hsv_ptr[row*mirror.cols*hsvChannels + col*hsvChannels + 2];
    double trustBright = get_trust(pow(v_h,2)*factorsBrightIntern(0) + pow(v_s,2)*factorsBrightIntern(1)
                                   + pow(v_v,2)*factorsBrightIntern(2),1);

    v_h = hsvValuesBright[0] - hsv_ptr[row*mirror.cols*hsvChannels + col*hsvChannels];
    v_s = hsvValuesBright[1] - hsv_ptr[row*mirror.cols*hsvChannels + col*hsvChannels + 1];
    v_v = hsvValuesBright[2] - hsv_ptr[row*mirror.cols*hsvChannels + col*hsvChannels + 2];
    double trustDark = get_trust(pow(v_h,2)*factorsDarkIntern(0) + pow(v_s,2)*factorsDarkIntern(1)
                                 + pow(v_v,2)*factorsDarkIntern(2),1);

    if (trustBright > trustDark)
        return trustBright;
    else
        return trustDark;
}

bool CalibrationHandler::compareBrightness(const Matrix8u& lhs, const Matrix8u& rhs)
{
    return lhs(0,2) < rhs(0,2);
}

void CalibrationHandler::getMedianValues(Eigen::Vector3i &hsvBrightOrDark,
                                 const Matrix8u &hsvValues,const uint64_t &startIdx,const uint64_t &endIdx)
{
    Eigen::VectorXi colorVec(endIdx-startIdx);

    for (int i = 0; i < colorVec.size(); ++i) {
        colorVec(i) = hsvValues(i+startIdx,0);
    }

    hsvBrightOrDark(0) = median(colorVec);

    for (int i = 0; i < colorVec.size(); ++i) {
        colorVec(i) = hsvValues(i+startIdx,1);
    }

    hsvBrightOrDark(1) = median(colorVec);

    for (int i = 0; i < colorVec.size(); ++i) {
        colorVec(i) = hsvValues(i+startIdx,2);
    }

    hsvBrightOrDark(2) = median(colorVec);
}

void CalibrationHandler::calculate(const Eigen::Vector3d &hsvBrightOrDark, Eigen::Vector3d &x0,
                           std::string brightOrDarkText, const Matrix8u &hsvValues,
                           const Eigen::VectorXd &results,const double &minErrorBound,
                           const uint16_t &maxIteration, const uint64_t &startGoodValues)
{
    double v_h;
    double v_s;
    double v_v;
    Eigen::MatrixXd A(hsvValues.rows(),hsvValues.cols());
    double sum_denominator;
    Eigen::BDCSVD<Eigen::MatrixXd> svd_solver;
    Eigen::Vector3d dx;
    bool run = true;
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


        if (getError(hsvBrightOrDark,factorsBrightOrDark,hsvValues,startGoodValues) < minErrorBound) {
            break;
        }

        ++iterations;
    }

    std::cout << "Used iterations for " << brightOrDarkText << unsigned(iterations) << std::endl;
}


bool CalibrationHandler::visualizeResult()
{
    uint64_t falsePositives = 0;

    for (uint16_t col = 0; col < mirror.cols; col+=square_points[2].x) {
        // Above
        for (uint16_t row = 0; row < square_points[0].y; row+=square_points[2].y) {
            addNegativePoint(row, col, falsePositives);
        }

        // Below
        for (int row = square_points[1].y; row < mirror.rows; row += square_points[2].y) {
            addNegativePoint(row, col, falsePositives);
        }
    }

    // Left
    for (int row = square_points[0].y; row < square_points[1].y; row += square_points[2].y) {
        for (int col = 0; col < square_points[0].x; col += square_points[2].x) {
            addNegativePoint(row, col, falsePositives);
        }

        // Right
        for (int col = square_points[1].x; col < mirror.cols; col += square_points[2].x) {
            addNegativePoint(row, col, falsePositives);
        }
    }

    // Good Values
    double prediction;

    for (int col = square_points[0].x; col < square_points[1].x; ++col) {
        for (int row = square_points[0].y; row < square_points[1].y; ++row) {
            prediction = getPrediction(hsv,factors,hsv_ptr,row,col,3,mirror.cols);

            if (prediction < 0.5)
                line(mirror,cv::Point(col,row),cv::Point(col,row),badColor);
            else {
                line(mirror,cv::Point(col,row),cv::Point(col,row),goodColor);
            }
        }
    }

    std::cout << "False positives in image = " << falsePositives << std::endl;
    cv::imshow(title,mirror);
    int key = cv::waitKey(0);

    if (key == 115) // s
        return true;
    else
        return false;
}

uint64_t CalibrationHandler::getError(const std::vector<uint16_t> &hsv,const std::vector<double> &factors,
                              const Matrix8u &hsvValues,const uint64_t &startGoodValues)
{
    uint64_t falsePositives = 0;
    double prediction;
    int16_t v_h;
    int16_t v_s;
    int16_t v_v;

    for (uint64_t i = 0; i < startGoodValues; ++i) {
        v_h= hsv[0] - hsvValues(i,0);
        v_s = hsv[1] - hsvValues(i,1);
        v_v = hsv[2] - hsvValues(i,2);
        prediction = get_trust(pow(v_h,2)*factors[0] + pow(v_s,2)*factors[1] + pow(v_v,2)*factors[2],1);

        if (prediction >= 0.5) ++falsePositives;
    }

    return falsePositives;
}

void CalibrationHandler::fillAllBadMatrices(const uint16_t &hsvCols, const uint16_t &row,
                                    const uint16_t &col, uint64_t &counter)
{
    hsvValuesBright(counter,0) = hsv_ptr[row*hsvCols*hsvChannels+col*hsvChannels];
    hsvValuesBright(counter,1) = hsv_ptr[row*hsvCols*hsvChannels+col*hsvChannels+1];
    hsvValuesBright(counter,2) = hsv_ptr[row*hsvCols*hsvChannels+col*hsvChannels+2];

    hsvValuesDark(counter,0) = hsv_ptr[row*hsvCols*hsvChannels+col*hsvChannels];
    hsvValuesDark(counter,1) = hsv_ptr[row*hsvCols*hsvChannels+col*hsvChannels+1];
    hsvValuesDark(counter,2) = hsv_ptr[row*hsvCols*hsvChannels+col*hsvChannels+2];

    cv::line(mirror,cv::Point(col,row),cv::Point(col,row),badColor);
    ++counter;

}

void CalibrationHandler::addNegativePoint(const uint16_t &row, const uint16_t &col, uint64_t &falsePositive)
{
    double prediction = getPrediction(row,col);

    if (prediction < 0.5)
        line(mirror,cv::Point(col,row),cv::Point(col,row),badColor);
    else {
        line(mirror,cv::Point(col,row),cv::Point(col,row),goodColor);
        ++falsePositive;
    }
}
