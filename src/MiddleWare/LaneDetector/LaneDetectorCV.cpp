#include "LaneDetectorCV.hpp"
#include <sys/time.h>
#include <iostream>

using namespace cv;
using namespace std;
using namespace zenoh;

LaneDetectorCV::LaneDetectorCV(const std::string& pipeline, std::shared_ptr<zenoh::Session> session)
    : cap(pipeline, cv::CAP_GSTREAMER),
      session_(session),
      prevLeftLine(0,0,0,0),
      prevRightLine(0,0,0,0), 
      prevMidLine(0,0,0,0),
      laneWidthEstimate(0.0),
      firstFrame(true),
      frame_count(0),
      FRAME_SKIP(2)
{
    if(!cap.isOpened()) {
        throw std::runtime_error("Error opening video stream");
    }
    
    // Set camera buffer size
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    
    // Initialize the lane detector publisher
    publisher_ = std::make_shared<LaneDetectorPublisher>(session_);
}

LaneDetectorCV::~LaneDetectorCV()
{
    cap.release();
    destroyAllWindows();
}

Mat LaneDetectorCV::regionOfInterest(const Mat &img, const vector<Point>& vertices) {
    Mat mask = Mat::zeros(img.size(), img.type());
    fillPoly(mask, vector<vector<Point>>{vertices}, Scalar(255));
    Mat masked;
    bitwise_and(img, mask, masked);
    return masked;
}

// Polynomial fitting using OpenCV
Mat LaneDetectorCV::polyfit(const Mat& y_vals, const Mat& x_vals, int degree) {
    // Check if we have enough points for the requested degree
    if (y_vals.rows < degree + 1) {
        // Not enough points - reduce degree or return empty matrix
        if (y_vals.rows < 2) {
            return Mat();
        }
        // Adjust degree based on available points
        degree = y_vals.rows - 1;
    }
    
    // Ensure data is in the right format (CV_64F)
    Mat y_vals_64f, x_vals_64f;
    y_vals.convertTo(y_vals_64f, CV_64F);
    x_vals.convertTo(x_vals_64f, CV_64F);
    
    // Create the design matrix with appropriate dimensions
    Mat A = Mat::zeros(y_vals_64f.rows, degree + 1, CV_64F);
    
    // Fill the design matrix
    for (int i = 0; i < y_vals_64f.rows; i++) {
        for (int j = 0; j <= degree; j++) {
            A.at<double>(i, j) = pow(y_vals_64f.at<double>(i), degree - j);
        }
    }
    
    // Check for invalid values (NaN, Inf)
    for (int i = 0; i < A.rows; i++) {
        for (int j = 0; j < A.cols; j++) {
            if (cvIsNaN(A.at<double>(i,j)) || cvIsInf(A.at<double>(i,j))) {
                A.at<double>(i,j) = 0;
            }
        }
    }
    
    // Solve the system using SVD for better stability
    Mat coeffs;
    try {
        solve(A, x_vals_64f, coeffs, DECOMP_SVD);
    } catch (const cv::Exception& e) {
        std::cerr << "Error in polyfit: " << e.what() << std::endl;
        return Mat();
    }
    
    return coeffs;
}


double LaneDetectorCV::getCurrentTime() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}

// Extrapolate a polynomial curve from lane line segments
vector<Point> LaneDetectorCV::extrapolatePolynomialCurve(const vector<Vec4i>& laneLines) {
    // Extract all points from line segments
    vector<Point2f> points;
    for (auto line : laneLines) {
        points.push_back(Point2f(line[0], line[1]));
        points.push_back(Point2f(line[2], line[3]));
    }

    if (points.empty() || points.size() < 3) // Need at least 3 points for quadratic
        return vector<Point>();

    // Convert to vectors for polynomial fitting
    vector<float> x_vals, y_vals;
    for (auto& pt : points) {
        x_vals.push_back(pt.x);
        y_vals.push_back(pt.y);
    }

    // Convert to Mat for polyfit
    Mat x_mat(x_vals), y_mat(y_vals);
    int degree = std::min(2, (int)points.size() - 1); // Adjust degree based on points available
    Mat coeffs = polyfit(y_mat, x_mat, degree);

    // Check if coeffs is valid
    if (coeffs.empty() || coeffs.rows < degree + 1)
        return vector<Point>();

    // Generate points along the curve
    vector<Point> curvePoints;
    int height = cap.get(CAP_PROP_FRAME_HEIGHT);
    for (int y = height; y >= height/3; y -= 5) {
        // Evaluate polynomial based on degree
        double x = 0;
        if (degree == 2 && coeffs.rows >= 3) {
            // Quadratic: x = a*y^2 + b*y + c
            x = coeffs.at<double>(0)*y*y + coeffs.at<double>(1)*y + coeffs.at<double>(2);
        } else if (degree == 1 && coeffs.rows >= 2) {
            // Linear: x = a*y + b
            x = coeffs.at<double>(0)*y + coeffs.at<double>(1);
        } else {
            continue;
        }

        // Check for valid x value before adding point
        if (!std::isnan(x) && !std::isinf(x)) {
            curvePoints.push_back(Point(round(x), y));
        }
    }

    return curvePoints;
}

Vec4i LaneDetectorCV::extrapolateLine(const vector<Vec4i>& laneLines) {
    double slopeSum = 0, interceptSum = 0;
    int count = 0;
    int height = cap.get(CAP_PROP_FRAME_HEIGHT);
    
    for(auto line : laneLines){
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        double slope = (double)(y2 - y1) / (x2 - x1);
        double intercept = y1 - slope * x1;
        slopeSum += slope;
        interceptSum += intercept;
        count++;
    }
    
    if(count == 0)
        return Vec4i(0,0,0,0);
    
    double avgSlope = slopeSum / count;
    double avgIntercept = interceptSum / count;
    
    // Define endpoints: bottom of ROI (y = height) and top of ROI (y = height/3)
    int yBottom = height;
    int yTop = height / 3;
    int xBottom = (int)((yBottom - avgIntercept) / avgSlope);
    int xTop = (int)((yTop - avgIntercept) / avgSlope);
    
    return Vec4i(xBottom, yBottom, xTop, yTop);
}

void LaneDetectorCV::detect(Mat& frame) {
    int height = frame.rows;
    int width = frame.cols;
    
    // Initialize lane width estimate on first frame.
    if(firstFrame) {
        laneWidthEstimate = width * 0.45;
    }
    
    // 1. Preprocessing: convert to grayscale, blur and detect edges.
    Mat gray;
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    Mat blur;
    GaussianBlur(gray, blur, Size(5,5), 0);
    Mat edges;
    Canny(blur, edges, 50, 150);
    
    // 2. Define ROI covering the lower 2/3 of the frame.
    vector<Point> roiVertices = {
        Point(0, height),
        Point(width, height),
        Point(width, height / 3),
        Point(0, height / 3)
    };
    Mat maskedEdges = regionOfInterest(edges, roiVertices);
    
    // 3. Use the Hough transform to detect line segments.
    vector<Vec4i> lines;
    HoughLinesP(maskedEdges, lines, 1, CV_PI/180, 20, 20, 30);
    
// 4. Separate lines into left and right lanes based on slope and position.
vector<Vec4i> leftLines, rightLines;
int midX = width / 2;  // Middle of the image

    for(auto line : lines) {
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        
        // Avoid division by zero and filter nearly horizontal lines.
        if (abs(x2 - x1) < 10) continue;
        
        double slope = (double)(y2 - y1) / (x2 - x1);
        if (abs(slope) < 0.5) continue;  // Filter out nearly horizontal lines
        
        // Calculate line midpoint
        int lineMidX = (x1 + x2) / 2;
        
        // Use both slope and position for classification
        // Left lane: negative slope OR on left half of image with steep slope
        // Right lane: positive slope OR on right half of image with steep slope
        if ((slope < 0 && lineMidX < midX * 1.2) || (lineMidX < midX * 0.6)) {
            leftLines.push_back(line);
        }
        // Right lane: positive slope OR on right half with steep slope
        else if ((slope > 0 && lineMidX > midX * 0.8) || (lineMidX > midX * 1.4)) {
            rightLines.push_back(line);
        }
    }

    // Post-classification cleanup - remove outliers using RANSAC or distance thresholding
    if (leftLines.size() > 3) {
        // Calculate average position of left lines
        int avgLeftX = 0;
        for (auto line : leftLines) {
            avgLeftX += (line[0] + line[2]) / 2;
        }
        avgLeftX /= leftLines.size();
        
        // Remove outliers that are too far to the right
        vector<Vec4i> filteredLeftLines;
        for (auto line : leftLines) {
            int lineMidX = (line[0] + line[2]) / 2;
            if (lineMidX < avgLeftX * 1.5) {  // Adjust threshold as needed
                filteredLeftLines.push_back(line);
            }
        }
        leftLines = filteredLeftLines;
    }

    // Similar logic for right lines
    if (rightLines.size() > 3) {
        // Calculate average position of right lines
        int avgRightX = 0;
        for (auto line : rightLines) {
            avgRightX += (line[0] + line[2]) / 2;
        }
        avgRightX /= rightLines.size();
        
        // Remove outliers that are too far to the left
        vector<Vec4i> filteredRightLines;
        for (auto line : rightLines) {
            int lineMidX = (line[0] + line[2]) / 2;
            if (lineMidX > avgRightX * 0.5) {  // Adjust threshold as needed
                filteredRightLines.push_back(line);
            }
        }
        rightLines = filteredRightLines;
    }
        
    // 5. Extract polynomial curves for left and right lanes
    vector<Point> leftCurve = extrapolatePolynomialCurve(leftLines);
    vector<Point> rightCurve = extrapolatePolynomialCurve(rightLines);
    
    vector<Point2f> leftPoints, rightPoints;
    for (auto line : leftLines) {
        leftPoints.push_back(Point2f(line[0], line[1]));
        leftPoints.push_back(Point2f(line[2], line[3]));
    }

    for (auto line : rightLines) {
        rightPoints.push_back(Point2f(line[0], line[1]));
        rightPoints.push_back(Point2f(line[2], line[3]));
    }

    // 6. Fallback: if one lane is missing, use previous data or estimate it using lane width
    if (leftCurve.empty()) {
        leftCurve = prevLeftCurve;
        // If we have right curve but no left curve, estimate left curve
        if (!rightCurve.empty() && !prevRightCurve.empty()) {
            leftCurve.clear();
            for (const auto& pt : rightCurve) {
                leftCurve.push_back(Point(pt.x - laneWidthEstimate, pt.y));
            }
        }
    }
    
    if (rightCurve.empty()) {
        rightCurve = prevRightCurve;
        // If we have left curve but no right curve, estimate right curve
        if (!leftCurve.empty() && !prevLeftCurve.empty()) {
            rightCurve.clear();
            for (const auto& pt : leftCurve) {
                rightCurve.push_back(Point(pt.x + laneWidthEstimate, pt.y));
            }
        }
    }
    
    // Update lane width estimate if both curves have points
    if (!leftCurve.empty() && !rightCurve.empty() && 
        leftCurve.size() > 0 && rightCurve.size() > 0) {
        // Find corresponding points at the bottom of the image
        int bottomY = height - 1;
        int leftX = -1, rightX = -1;
        
        for (const auto& pt : leftCurve) {
            if (pt.y == bottomY || (leftX == -1 && pt.y > height * 0.7)) {
                leftX = pt.x;
                break;
            }
        }
        
        for (const auto& pt : rightCurve) {
            if (pt.y == bottomY || (rightX == -1 && pt.y > height * 0.7)) {
                rightX = pt.x;
                break;
            }
        }
        
        if (leftX != -1 && rightX != -1) {
            double currentWidth = rightX - leftX;
            laneWidthEstimate = 0.2 * currentWidth + 0.8 * laneWidthEstimate;
        }
    }
    
    // 7. Smooth curves by averaging with previous frames
    double alpha = 0.2;
    if (!firstFrame) {
        // Only if we have previous curves and current curves
        if (!prevLeftCurve.empty() && !leftCurve.empty() && 
            prevLeftCurve.size() == leftCurve.size()) {
            for (size_t i = 0; i < leftCurve.size(); i++) {
                leftCurve[i].x = (int)(alpha * leftCurve[i].x + (1 - alpha) * prevLeftCurve[i].x);
                leftCurve[i].y = (int)(alpha * leftCurve[i].y + (1 - alpha) * prevLeftCurve[i].y);
            }
        }
        
        if (!prevRightCurve.empty() && !rightCurve.empty() && 
            prevRightCurve.size() == rightCurve.size()) {
            for (size_t i = 0; i < rightCurve.size(); i++) {
                rightCurve[i].x = (int)(alpha * rightCurve[i].x + (1 - alpha) * prevRightCurve[i].x);
                rightCurve[i].y = (int)(alpha * rightCurve[i].y + (1 - alpha) * prevRightCurve[i].y);
            }
        }
    }
    
    // Save current curves for next frame
    prevLeftCurve = leftCurve;
    prevRightCurve = rightCurve;
    
    // 8. Compute mid curve points as average of left and right curves
    vector<Point> midCurve;
    if (!leftCurve.empty() && !rightCurve.empty() && 
        leftCurve.size() == rightCurve.size()) {
        for (size_t i = 0; i < leftCurve.size(); i++) {
            int midX = (leftCurve[i].x + rightCurve[i].x) / 2;
            int midY = (leftCurve[i].y + rightCurve[i].y) / 2;
            midCurve.push_back(Point(midX, midY));
        }
    }
    
    // 9. Compute the midline reference point
    Point midPoint;
    if (!midCurve.empty()) {
        // Use the bottom-most point of the mid curve (or average multiple points)
        size_t bottom_idx = 0;
        for (size_t i = 1; i < midCurve.size(); i++) {
            if (midCurve[i].y > midCurve[bottom_idx].y) {
                bottom_idx = i;
            }
        }
        midPoint = midCurve[bottom_idx];
    } else {
        // Fallback to center of image
        midPoint = Point(width/2, height*2/3);
    }
    
    prevMidCurve = midCurve;
    firstFrame = false;
    
    // Calculate lateral error
    float centerX = width / 2;
    float lateralError = midPoint.x - centerX;
    float divider = width / 2;
    lateralError = lateralError / divider;
    
    // Publish camera error
    if (publisher_) {
        publisher_->publishCameraError(lateralError);
    }
    
    // 10. Draw the detected lane curves and midpoint
    Mat lineImage = Mat::zeros(frame.size(), frame.type());
    
    // // Draw left curve
    // if (!leftCurve.empty()) {
    //     for (size_t i = 1; i < leftCurve.size(); i++) {
    //         line(lineImage, leftCurve[i-1], leftCurve[i], Scalar(255, 0, 0), 5);
    //     }
    // }
    
    // // Draw right curve
    // if (!rightCurve.empty()) {
    //     for (size_t i = 1; i < rightCurve.size(); i++) {
    //         line(lineImage, rightCurve[i-1], rightCurve[i], Scalar(0, 255, 0), 5);
    //     }
    // }
    
    // // Draw mid curve (optional)
    // if (!midCurve.empty()) {
    //     for (size_t i = 1; i < midCurve.size(); i++) {
    //         line(lineImage, midCurve[i-1], midCurve[i], Scalar(0, 0, 255), 3);
    //     }
    // }
    
    // Draw reference point
    circle(lineImage, midPoint, 8, Scalar(0, 0, 255), -1);
    
    // Draw all detected points to see what's being fitted
    for (const auto& pt : leftPoints) {
        circle(lineImage, pt, 4, Scalar(255, 255, 0), -1);  // Yellow for left points
    }

    for (const auto& pt : rightPoints) {
        circle(lineImage, pt, 4, Scalar(0, 255, 255), -1);  // Cyan for right points
    }

    // 11. Overlay the lane curves and reference point on the original frame.
    addWeighted(frame, 0.8, lineImage, 1.0, 0, frame);
}

void LaneDetectorCV::run() {
    Mat frame;
    
    while(true) {
        cap >> frame;
        if(frame.empty())
            break;
        
        if (frame_count % FRAME_SKIP == 0) {
            detect(frame);
            imshow("Lane Detection", frame);
        }
        
        frame_count++;
        if(waitKey(1) == 27)
            break;
    }
}