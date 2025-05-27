#include "IPM.hpp"
#include <cmath>

IPM::IPM() {
    initialized = false;
}

IPM::IPM(const cv::Size& _origSize, const cv::Size& _dstSize,
         const std::vector<cv::Point2f>& _origPoints, const std::vector<cv::Point2f>& _dstPoints) {
    
    initialize(_origSize, _dstSize, _origPoints, _dstPoints);
}

void IPM::init(const cv::Size& _origSize, const cv::Size& _dstSize,
                     const std::vector<cv::Point2f>& _origPoints, const std::vector<cv::Point2f>& _dstPoints) {
    origSize = _origSize;
    dstSize = _dstSize;
    origPoints = _origPoints;
    dstPoints = _dstPoints;
    
    // If no source/destination points provided, use default
    if (origPoints.empty() || dstPoints.empty()) {
        // Default points for standard dashcam perspective
        float width = static_cast<float>(origSize.width);
        float height = static_cast<float>(origSize.height);
        
        // Source points - trapezoid on the original image
        origPoints = {
            cv::Point2f(width * 0.35f, height * 0.65f),  // Top-left
            cv::Point2f(width * 0.65f, height * 0.65f),  // Top-right
            cv::Point2f(width * 0.9f, height * 0.95f),   // Bottom-right
            cv::Point2f(width * 0.1f, height * 0.95f)    // Bottom-left
        };
        
        // Destination points - rectangle in bird's eye view
        dstPoints = {
            cv::Point2f(0, 0),                // Top-left
            cv::Point2f(dstSize.width, 0),    // Top-right
            cv::Point2f(dstSize.width, dstSize.height), // Bottom-right
            cv::Point2f(0, dstSize.height)    // Bottom-left
        };
    }
    
    // Calculate perspective transform matrix
    perspectiveMatrix = cv::getPerspectiveTransform(origPoints, dstPoints);
    invPerspectiveMatrix = cv::getPerspectiveTransform(dstPoints, origPoints);
    
    initialized = true;
}

cv::Mat IPM::applyIPM(const cv::Mat& input) {
    if (!initialized) {
        std::cerr << "IPM not initialized. Call initialize() first." << std::endl;
        return input.clone();
    }
    
    // Check if input size matches
    if (input.size() != origSize) {
        std::cerr << "Input size doesn't match initialization size" << std::endl;
        cv::resize(input, resizedInput, origSize);
    } else {
        resizedInput = input;
    }
    // Apply perspective transformation
    cv::Mat result;
    cv::warpPerspective(resizedInput, result, perspectiveMatrix, dstSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    
    return result;
}

cv::Mat IPM::applyInverseIPM(const cv::Mat& input) {
    if (!initialized) {
        std::cerr << "IPM not initialized. Call initialize() first." << std::endl;
        return input.clone();
    }
    
    // Apply inverse perspective transformation
    cv::Mat result;
    cv::warpPerspective(input, result, invPerspectiveMatrix, origSize, cv::INTER_LINEAR);
    
    return result;
}

void IPM::drawPoints(cv::Mat& image, const std::vector<cv::Point2f>& points, const cv::Scalar& color) {
    for (const auto& point : points) {
        cv::circle(image, point, 5, color, -1);
    }
    
    // Connect points with lines to form a polygon
    for (size_t i = 0; i < points.size(); i++) {
        cv::line(image, points[i], points[(i + 1) % points.size()], color, 2);
    }
}

void IPM::calibrateFromCamera(
    float cameraHeight, float cameraPitch,
    float horizontalFOV, float verticalFOV,
    float nearDistance, float farDistance, float laneWidth
) {
    // Calculate source points based on camera parameters
    origPoints = calculateIPMPoints(
        cameraHeight, cameraPitch, horizontalFOV, verticalFOV,
        origSize, nearDistance, farDistance, laneWidth
    );

    // Keep the destination points as a full rectangle
    dstPoints = {
        cv::Point2f(0, 0),                // Top-left
        cv::Point2f(dstSize.width, 0),    // Top-right
        cv::Point2f(dstSize.width, dstSize.height), // Bottom-right
        cv::Point2f(0, dstSize.height)    // Bottom-left
    };
    
    // Re-calculate the transform matrix
    perspectiveMatrix = cv::getPerspectiveTransform(origPoints, dstPoints);
    invPerspectiveMatrix = cv::getPerspectiveTransform(dstPoints, origPoints);
    
    initialized = true;
}

std::vector<cv::Point2f> IPM::calculateIPMPoints(
    float    cameraHeight,    // in meters
    float    cameraPitchDeg,  // + down from horizontal
    float    hFOVDeg,         // horizontal FOV in degrees
    float    vFOVDeg,         // vertical   FOV in degrees
    const cv::Size& imageSize,
    float    nearDistance,    // in meters
    float    farDistance,     // in meters
    float    laneWidth        // in meters
) {
    // build intrinsic from FOV
    float pitch = cameraPitchDeg * CV_PI/180.0f;
    float fx = imageSize.width  / (2.0f * tan(hFOVDeg * CV_PI/180.0f / 2.0f));
    float fy = imageSize.height / (2.0f * tan(vFOVDeg * CV_PI/180.0f / 2.0f));
    float cx = imageSize.width  * 0.5f;
    float cy = imageSize.height * 0.5f;
    cv::Mat K = (cv::Mat_<double>(3,3) << fx, 0, cx,
                                          0, fy, cy,
                                          0,  0,  1);
    cv::Mat distCoeffs = cv::Mat::zeros(5,1,CV_64F);

    // rotate world so that camera is pitched down by +pitch
    // projectPoints expects “rotate world toward camera,” so we use –pitch
    // cv::Mat rvec = (cv::Mat_<double>(3,1) << -pitch, 0.0, 0.0);

    // // camera is H above road, so translate the road by –H in camera Y
    // cv::Mat tvec = (cv::Mat_<double>(3,1) << 0.0, -cameraHeight, 0.0);
    // 1. Make R from your pitch (in radians):
    cv::Mat R_wc;
    cv::Rodrigues(cv::Vec3d(+pitch, 0.0, 0.0), R_wc);
    
    // 2) Camera position in world: (0, cameraHeight, 0)  
    cv::Mat camPos = (cv::Mat_<double>(3,1) << 0.0, cameraHeight, 0.0);
    
    // 3) Convert to world→camera:  
    //    R_cw = R_wc.t()                (since R_wc rotates camera→world)
    //    tvec  = -R_cw * camPos
    cv::Mat R_cw = R_wc.t();
    cv::Mat tvec = -R_cw * camPos;
    
    // 4) Your ground points (X lateral, Y=0, Z forward):
    std::vector<cv::Point3f> roadPts = {
      { -laneWidth/2, 0.0f, farDistance },
      { +laneWidth/2, 0.0f, farDistance },
      { +laneWidth/2, 0.0f, nearDistance },
      { -laneWidth/2, 0.0f, nearDistance }
    };
    
    // 5) Project with the full R_cw & tvec:
    std::vector<cv::Point2f> imgPts;
    cv::projectPoints(roadPts, R_cw, tvec, K, distCoeffs, imgPts);
    for(auto &p : imgPts) {
        p.y = imageSize.height - 1 - p.y;
    }

    return imgPts;
}

cv::Mat IPM::transformPoints(const cv::Mat& mask) {
    // Create an empty result mask of the destination size
    cv::Mat result = cv::Mat::zeros(dstSize, CV_8UC1);
    
    // Find non-zero points in the input mask
    std::vector<cv::Point> points;
    cv::findNonZero(mask, points);
    // std::cout << "Found " << points.size() << " non-zero points in mask" << std::endl;
    
    // If no points found, return empty result
    if (points.empty()) {
        return result;
    }
    
    // Convert points to Point2f for transformation
    std::vector<cv::Point2f> srcPoints;
    srcPoints.reserve(points.size());
    for (const auto& pt : points) {
        srcPoints.push_back(cv::Point2f(static_cast<float>(pt.x), static_cast<float>(pt.y)));
    }
    
    // Transform points using the perspective matrix
    std::vector<cv::Point2f> dstPoints;
    cv::perspectiveTransform(srcPoints, dstPoints, perspectiveMatrix);
    
    // Draw the transformed points on the result mask
    for (const auto& pt : dstPoints) {
        // Check if point is within bounds
        int x = cvRound(pt.x);
        int y = cvRound(pt.y);
        if (x >= 0 && x < dstSize.width && y >= 0 && y < dstSize.height) {
            result.at<uchar>(y, x) = 255;
        }
    }
    
    // Optional: Apply slight dilation to avoid gaps between points
    cv::dilate(result, result, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    
    return result;
}