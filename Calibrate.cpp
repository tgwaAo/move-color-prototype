/**
* Take picture after a short time,
* drag and drop over it and calibrate.
*/

#include "Calibrate.h"


Calibrator::Calibrator()
{
    // do stuff
}

Calibrator::Calibrator(const ParticleWeighting &pw)
{
    // better but jet undone
}

Calibrator::~Calibrator()
{
    // do not forget delete!!
}

void Calibrator::calibrate(cv::VideoCapture &cap, const std::string &title, const double &WIDTH, const double &HEIGHT,
               const std::string &settings_filename,
               std::vector<uint16_t> &hsvBright, std::vector<double> &factorsBright,
               std::vector<uint16_t> &hsvDark, std::vector<double> &factorsDark)
{
    /***********************************************************
     *  Wait a few seconds to give the user time to get in position.
     * ********************************************************/
    const uint8_t minTimePassed = 5;
    clock_t timeStart = clock();
    int8_t leftSeconds = minTimePassed - (float)(clock() - timeStart)/CLOCKS_PER_SEC;
    cv::Mat mirror_copy;

    while (leftSeconds > 0) {
        cap >> mirror_copy;
        cv::flip(mirror_copy,mirror,1);
        leftSeconds = minTimePassed - (float)(clock() - timeStart)/CLOCKS_PER_SEC;
        cv::putText(mirror,"Countdown= " + std::to_string(leftSeconds),
                    cv::Point(10,mirror.rows-10),cv::FONT_HERSHEY_SIMPLEX,1.2,textColor,2);
        cv::imshow(title,mirror);
        cv::waitKey(1);
    }

    /***************************************************************
     * Let user select positives.
     * ************************************************************/
    cap >> mirror_copy;
    cv::flip(mirror_copy,mirror,1);
    mirror.copyTo(mirror_copy);
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
    const uint8_t NEG_DIST = 2; // for 480*640
    const uint8_t POS_DIST = 1;
    uint8_t step_x = NEG_DIST*mirror.cols/WIDTH;
    uint8_t step_y = NEG_DIST*mirror.rows/HEIGHT;
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
     * Fill matrices for a database for calculation.
     * ***********************************************************/
    Matrix8u hsvValuesBright(numAllBrightAndDark,3);
    Matrix8u hsvValuesDark(numAllBrightAndDark,3);
    uint64_t counter = 0;
    cv::Scalar badColor(0,0,255);
    cv::Scalar goodColorBright(255,0,0);
    cv::Scalar goodColorDark(0,255,0);
    mirror_copy.copyTo(mirror);
    cv::Mat hsv;
    cv::cvtColor(mirror,hsv,cv::COLOR_BGR2HSV);
    *hsv_ptr = (uint8_t*)hsv.data;
    hsvChannels = hsv.channels();


////////////////to be in separate method
    for (int col = 0; col < mirror.cols; col+=step_x) {
        // Above square
        for (int row = 0; row < smallest_y; row+=step_y) {
            fill_matrix(hsvValuesBright,hsvChannels,hsv.cols,hsv_ptr,row,col,counter);
            fill_matrix(hsvValuesDark,hsvChannels,hsv.cols,hsv_ptr,row,col,counter);
            cv::line(mirror,cv::Point(col,row),cv::Point(col,row),badColor);
            ++counter;
        }

        // Below square
        for (int row = biggest_y; row < mirror.rows; row += step_y) {
            fill_matrix(hsvValuesBright,hsvChannels,hsv.cols,hsv_ptr,row,col,counter);
            fill_matrix(hsvValuesDark,hsvChannels,hsv.cols,hsv_ptr,row,col,counter);
            cv::line(mirror,cv::Point(col,row),cv::Point(col,row),badColor);
            ++counter;
        }
    }

    // Left next to square
    for (int row = smallest_y; row < biggest_y; row += step_y) {
        for (int col = 0; col < smallest_x; col += step_x) {
            fill_matrix(hsvValuesBright,hsvChannels,hsv.cols,hsv_ptr,row,col,counter);
            fill_matrix(hsvValuesDark,hsvChannels,hsv.cols,hsv_ptr,row,col,counter);
            cv::line(mirror,cv::Point(col,row),cv::Point(col,row),badColor);
            ++counter;

        }

        // Right next to square
        for (int col = biggest_x; col < mirror.cols; col += step_x) {
            fill_matrix(hsvValuesBright,hsvChannels,hsv.cols,hsv_ptr,row,col,counter);
            fill_matrix(hsvValuesDark,hsvChannels,hsv.cols,hsv_ptr,row,col,counter);
            cv::line(mirror,cv::Point(col,row),cv::Point(col,row),badColor);
            ++counter;
        }
    }

    // Sort good hsv values in relation to value,
    // split values into bright and dark, and fill matrices
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
    cv::waitKey(1); // to show img

    // Sort vector and position matrix in relation to vector (sort matrix in relation to third column).
    std::sort(allGoodValues.begin(), allGoodValues.end(), &compare_head);
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
    std::vector<uint16_t> hsvBrightFirstWritten(3,0);
    getMedianValues(hsvBrightFirstWritten,hsvValuesBright,goodValuesStart,counter);

    std::cout << "Bright hsv= " << unsigned(hsvBrightFirstWritten[0])
              << " " << unsigned(hsvBrightFirstWritten[1]) << " " << unsigned(hsvBrightFirstWritten[2]) << std::endl;

    std::vector<uint16_t> hsvDarkFirstWritten(3,0);
    getMedianValues(hsvDarkFirstWritten,hsvValuesDark,goodValuesStart,counter);

    std::cout << "Dark hsv= " << unsigned(hsvDarkFirstWritten[0])
              << " " << unsigned(hsvDarkFirstWritten[1]) << " " << unsigned(hsvDarkFirstWritten[2]) << std::endl;

    /*************************************************************
     * Start calibration for factors (tolerances) of colors
     * **********************************************************/
    std::cout << "Starting calibration" << std::endl;

    const double START_VALUE_FACTORS = 0.0005;
    const double MIN_ERROR = 50;
    const uint16_t MAX_ITERATION = 100;
    uint16_t iterations = 0;
    std::vector<double> factorsBrightFirstWritten(3,0);
    preparedCalibration(hsvDarkFirstWritten,factorsBrightFirstWritten,iterations,hsvValuesBright,
                        START_VALUE_FACTORS,results,MIN_ERROR,MAX_ITERATION,goodValuesStart);

    std::cout << "Used iterations for bright = "<< unsigned(iterations) << std::endl;
    std::cout << "Factors for bright = " << std::endl << factorsBrightFirstWritten[0]
              << factorsBrightFirstWritten[1] << factorsBrightFirstWritten[2] << std::endl;

    std::vector<double> factorsDarkFirstWritten(3,0);
    preparedCalibration(hsvDarkFirstWritten,factorsDarkFirstWritten,iterations,hsvValuesDark,
                        START_VALUE_FACTORS,results,MIN_ERROR,MAX_ITERATION,goodValuesStart);

    std::cout << "Used iterations for dark = "<< unsigned(iterations) << std::endl;
    std::cout << "Factors for dark = " << std::endl << factorsDarkFirstWritten[0]
              << factorsDarkFirstWritten[1] << factorsDarkFirstWritten[2] << std::endl;

    /***************************************************************
     * Visualisation of bright color search and optional save.
     * ************************************************************/
     
     ///////////////combine them ////////////////////////////7
    mirror_copy.copyTo(mirror);
    bool shouldSaveBright =  visualize(mirror,title,badColor,goodColorBright,
                                       hsvBrightFirstWritten,factorsBrightFirstWritten,
                                       hsv_ptr,square_points,true_positive_square,"bright");
    bool shouldSaveDark =  visualize(mirror,title,badColor,goodColorDark,
                                     hsvDarkFirstWritten,factorsDarkFirstWritten,
                                     hsv_ptr,square_points,true_positive_square,"dark");

    if (shouldSaveBright || shouldSaveDark) {
        if (shouldSaveBright) {
            hsvBright = hsvBrightFirstWritten;
            factorsBright = factorsBrightFirstWritten;
        }

        if (shouldSaveDark) {
            hsvDark = hsvDarkFirstWritten;
            factorsDark = factorsDarkFirstWritten;
        }

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


int Calibrator::median(Eigen::VectorXi &v)
{
    size_t n = v.size() / 2;
    std::nth_element(v.data(), v.data()+n, v.data()+v.size());
    return v(n);
}

void Calibrator::click_and_crop(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN) {
        square_points.push_back(cv::Point(x,y));
    } else if (event == cv::EVENT_LBUTTONUP) {
        square_points.push_back(cv::Point(x,y));
        rectangle(mirror,square_points[0],square_points[1],cv::Scalar(0,0,0),2);
    }
}

void Calibrator::fill_matrix(Matrix8u &mx,const uint8_t &hsvChannels, const int &hsvCols, uint8_t *hsv_ptr,
                 const int &row, const int &col, uint64_t &counter)
{
    mx(counter,0) = hsv_ptr[row*hsvCols*hsvChannels+col*hsvChannels];
    mx(counter,1) = hsv_ptr[row*hsvCols*hsvChannels+col*hsvChannels+1];
    mx(counter,2) = hsv_ptr[row*hsvCols*hsvChannels+col*hsvChannels+2];
}

void Calibrator::find_minmax_xy(const std::vector<cv::Point> &square, uint16_t &smallest_x,
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

double Calibrator::get_trust(const double &v,const double &factor)
{
    return (1/(1+factor*v));
}

double Calibrator::getPrediction(std::vector<uint16_t> &hsvBrightOrDark,
                     std::vector<double> &factors, const uint8_t *hsv_ptr,
                     const uint16_t &row, const uint16_t &col,
                     const uint8_t &channels, const int &hsv_cols)
{
    int16_t v_h = hsvBrightOrDark[0] - hsv_ptr[row*hsv_cols*channels + col*channels];
    int16_t v_s = hsvBrightOrDark[1] - hsv_ptr[row*hsv_cols*channels + col*channels + 1];
    int16_t v_v = hsvBrightOrDark[2] - hsv_ptr[row*hsv_cols*channels + col*channels + 2];
    return get_trust(pow(v_h,2)*factors[0] + pow(v_s,2)*factors[1] + pow(v_v,2)*factors[2],1);
}

bool Calibrator::compare_head(const Matrix8u& lhs, const Matrix8u& rhs)
{
    return lhs(0,2) < rhs(0,2);
}

void Calibrator::getMedianValues(std::vector<uint16_t> &hsvBrightOrDark,
                     const Matrix8u &hsvValues,const uint64_t &startIdx,const uint64_t &endIdx)
{
    Eigen::VectorXi colorVec(endIdx-startIdx);

    for (int i = 0; i < colorVec.size(); ++i) {
        colorVec(i) = hsvValues(i+startIdx,0);
    }

    hsvBrightOrDark[0] = median(colorVec);

    for (int i = 0; i < colorVec.size(); ++i) {
        colorVec(i) = hsvValues(i+startIdx,1);
    }

    hsvBrightOrDark[1] = median(colorVec);

    for (int i = 0; i < colorVec.size(); ++i) {
        colorVec(i) = hsvValues(i+startIdx,2);
    }

    hsvBrightOrDark[2] = median(colorVec);
}

void Calibrator::preparedCalibration(const std::vector<uint16_t> &hsvBrightOrDark,
                         std::vector<double> &factorsBrightOrDark,uint16_t &iter,
                         const Matrix8u &hsvValues, const double &factorsStart,
                         const Eigen::VectorXd &results,const double &minErrorBound,
                         const uint16_t &maxIteration, const uint64_t &startGoodValues)
{
    double v_h;
    double v_s;
    double v_v;
    Eigen::MatrixXd A(hsvValues.rows(),hsvValues.cols());
    double sum_denominator;
    Eigen::Vector3d x0 = {factorsStart,factorsStart,factorsStart};
    Eigen::BDCSVD<Eigen::MatrixXd> svd_solver;
    iter = 0;
    Eigen::Vector3d dx;
    bool run = true;

    // Bright calibration
    while (iter < maxIteration && run) {
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
            run = false;
        }

        ++iter;
    }

    factorsBrightOrDark[0] = x0(0);
    factorsBrightOrDark[1] = x0(1);
    factorsBrightOrDark[2] = x0(2);
}


bool Calibrator::visualize(cv::Mat &mirror, const std::string title,const cv::Scalar &badColor,
               const cv::Scalar &goodColor,std::vector<uint16_t> &hsv,
               std::vector<double> &factors,const uint8_t *hsv_ptr,
               std::vector<cv::Point> squareAllGoodValues,
               std::vector<cv::Point> squareOnlyGoodValues, std::string textBrightOrDark)
{
    cv::Scalar color;
    uint64_t falsePositives = 0;
    double prediction;

    for (uint16_t col = 0; col < mirror.cols; col+=squareAllGoodValues[2].x) {
        // Above
        for (uint16_t row = 0; row < squareAllGoodValues[0].y; row+=squareAllGoodValues[2].y) {
            prediction = getPrediction(hsv,factors,hsv_ptr,row,col,3,mirror.cols);
            visualizePoint(mirror,prediction,0.5,row,col,badColor,goodColor,falsePositives);
        }

        // Below
        for (int row = squareAllGoodValues[1].y; row < mirror.rows; row += squareAllGoodValues[2].y) {
            prediction = getPrediction(hsv,factors,hsv_ptr,row,col,3,mirror.cols);
            visualizePoint(mirror,prediction,0.5,row,col,badColor,goodColor,falsePositives);
        }
    }

    // Left
    for (int row = squareAllGoodValues[0].y; row < squareAllGoodValues[1].y; row += squareAllGoodValues[2].y) {
        for (int col = 0; col < squareAllGoodValues[0].x; col += squareAllGoodValues[2].x) {
            prediction = getPrediction(hsv,factors,hsv_ptr,row,col,3,mirror.cols);
            visualizePoint(mirror,prediction,0.5,row,col,badColor,goodColor,falsePositives);
        }

        // Right
        for (int col = squareAllGoodValues[1].x; col < mirror.cols; col += squareAllGoodValues[2].x) {
            prediction = getPrediction(hsv,factors,hsv_ptr,row,col,3,mirror.cols);
            visualizePoint(mirror,prediction,0.5,row,col,badColor,goodColor,falsePositives);
        }
    }

    // Good Values
    for (int col = squareAllGoodValues[0].x; col < squareAllGoodValues[1].x; ++col) {
        for (int row = squareAllGoodValues[0].y; row < squareAllGoodValues[1].y; ++row) {
            prediction = getPrediction(hsv,factors,hsv_ptr,row,col,3,mirror.cols);

            if (prediction < 0.5)
                line(mirror,cv::Point(col,row),cv::Point(col,row),badColor);
            else {
                line(mirror,cv::Point(col,row),cv::Point(col,row),goodColor);
            }
        }
    }

    std::cout << "False positives in " << textBrightOrDark
              << " = " << falsePositives << std::endl;
    cv::imshow(title,mirror);
    int key = cv::waitKey(0);

    if (key == 115) // s
        return true;
    else
        return false;
}

void Calibrator::visualizePoint(cv::Mat &mirror,const double &prediction,
                    const double &bound,const int &row,const int &col,
                    const cv::Scalar &badColor,const cv::Scalar &goodColor,
                    uint64_t &falsePositive)
{
    if (prediction < 0.5)
        line(mirror,cv::Point(col,row),cv::Point(col,row),badColor);
    else {
        line(mirror,cv::Point(col,row),cv::Point(col,row),goodColor);
        ++falsePositive;
    }
}

uint64_t Calibrator::getError(const std::vector<uint16_t> &hsv,const std::vector<double> &factors,
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
