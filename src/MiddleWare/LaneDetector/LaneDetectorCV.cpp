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

double LaneDetectorCV::getCurrentTime() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec * 1e-6;
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
    
    // 4. Separate lines into left and right lanes based on slope.
    vector<Vec4i> leftLines, rightLines;
    for(auto line : lines) {
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        // Avoid division by zero and filter nearly horizontal lines.
        if (abs(x2 - x1) < 10) continue;
        double slope = (double)(y2 - y1) / (x2 - x1);
        if (abs(slope) < 0.5)
            continue;
        if(slope < 0)
            leftLines.push_back(line);
        else
            rightLines.push_back(line);
    }
    
    // 5. Extrapolate a single left and right lane line by averaging.
    Vec4i leftLine = extrapolateLine(leftLines);
    Vec4i rightLine = extrapolateLine(rightLines);
    
    // 6. Fallback: if one lane is missing, use previous data or estimate it using lane width.
    if(leftLine == Vec4i(0,0,0,0)) {
        // If left line missing, use previous left line if available.
        leftLine = prevLeftLine;
        // Or estimate it from right line if that is detected.
        if(rightLine != Vec4i(0,0,0,0)) {
            leftLine[0] = rightLine[0] - (int)laneWidthEstimate;
            leftLine[2] = rightLine[2] - (int)laneWidthEstimate;
        }
    }
    if(rightLine == Vec4i(0,0,0,0)) {
        // If right line missing, use previous right line if available.
        rightLine = prevRightLine;
        // Or estimate it from left line if that is detected.
        if(leftLine != Vec4i(0,0,0,0)) {
            rightLine[0] = leftLine[0] + (int)laneWidthEstimate;
            rightLine[2] = leftLine[2] + (int)laneWidthEstimate;
        }
    }
    
    // Update lane width estimate if both lines are available.
    if(leftLine != Vec4i(0,0,0,0) && rightLine != Vec4i(0,0,0,0)) {
        int leftXBottom = leftLine[0];
        int rightXBottom = rightLine[0];
        double currentWidth = rightXBottom - leftXBottom;
        // Smooth lane width estimation
        laneWidthEstimate = 0.2 * currentWidth + 0.8 * laneWidthEstimate;
    }
    
    // 7. Smooth lane lines by combining with previous frame values.
    double alpha = 0.2;
    if(!firstFrame){
        leftLine[0] = (int)(alpha * leftLine[0] + (1 - alpha) * prevLeftLine[0]);
        leftLine[1] = (int)(alpha * leftLine[1] + (1 - alpha) * prevLeftLine[1]);
        leftLine[2] = (int)(alpha * leftLine[2] + (1 - alpha) * prevLeftLine[2]);
        leftLine[3] = (int)(alpha * leftLine[3] + (1 - alpha) * prevLeftLine[3]);
        
        rightLine[0] = (int)(alpha * rightLine[0] + (1 - alpha) * prevRightLine[0]);
        rightLine[1] = (int)(alpha * rightLine[1] + (1 - alpha) * prevRightLine[1]);
        rightLine[2] = (int)(alpha * rightLine[2] + (1 - alpha) * prevRightLine[2]);
        rightLine[3] = (int)(alpha * rightLine[3] + (1 - alpha) * prevRightLine[3]);
    }
    prevLeftLine = leftLine;
    prevRightLine = rightLine;
    
    // 8. Compute mid-lane as the average of left and right lane endpoints.
    Vec4i midLine;
    midLine[0] = (leftLine[0] + rightLine[0]) / 2;
    midLine[1] = (leftLine[1] + rightLine[1]) / 2;
    midLine[2] = (leftLine[2] + rightLine[2]) / 2;
    midLine[3] = (leftLine[3] + rightLine[3]) / 2;
    
    // Further smooth the mid-line with previous midline.
    if(!firstFrame){
        midLine[0] = (int)(alpha * midLine[0] + (1 - alpha) * prevMidLine[0]);
        midLine[1] = (int)(alpha * midLine[1] + (1 - alpha) * prevMidLine[1]);
        midLine[2] = (int)(alpha * midLine[2] + (1 - alpha) * prevMidLine[2]);
        midLine[3] = (int)(alpha * midLine[3] + (1 - alpha) * prevMidLine[3]);
    }
    prevMidLine = midLine;
    firstFrame = false;
    
    // 9. Compute the midline reference point (average of the midline endpoints)
    Point midPoint((midLine[0] + midLine[2]) / 2, (midLine[1] + midLine[3]) / 2);

    float centerX = width / 2;
    float lateralError = midPoint.x - centerX;
    float divider = width / 2;
    lateralError = lateralError / divider;
    
    // Publish camera error
    if (publisher_) {
        publisher_->publishCameraError(lateralError);
    }
    
    // 10. Draw the detected lane lines and the midline reference point.
    Mat lineImage = Mat::zeros(frame.size(), frame.type());
    if(leftLine != Vec4i(0,0,0,0))
        line(lineImage, Point(leftLine[0], leftLine[1]), Point(leftLine[2], leftLine[3]), Scalar(255,0,0), 5);
    if(rightLine != Vec4i(0,0,0,0))
        line(lineImage, Point(rightLine[0], rightLine[1]), Point(rightLine[2], rightLine[3]), Scalar(0,255,0), 5);
    
    // Draw the reference point on the midline
    circle(lineImage, midPoint, 8, Scalar(0,0,255), -1);
    
    // 11. Overlay the lane lines and reference point on the original frame.
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