#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

class IPM {
public:
    // Constructors
    IPM();
    IPM(const cv::Size& _origSize, const cv::Size& _dstSize,
        const std::vector<cv::Point2f>& _origPoints = std::vector<cv::Point2f>(),
        const std::vector<cv::Point2f>& _dstPoints = std::vector<cv::Point2f>());
    
    // Initialize the IPM
    void initialize(const cv::Size& _origSize, const cv::Size& _dstSize,
                   const std::vector<cv::Point2f>& _origPoints = std::vector<cv::Point2f>(),
                   const std::vector<cv::Point2f>& _dstPoints = std::vector<cv::Point2f>());
    
    // Apply IPM to an input image
    cv::Mat applyIPM(const cv::Mat& input);
    
    // Apply inverse IPM to go back to original perspective
    cv::Mat applyInverseIPM(const cv::Mat& input);
    
    // Helper for visualization
    static void drawPoints(cv::Mat& image, const std::vector<cv::Point2f>& points, 
                         const cv::Scalar& color = cv::Scalar(0, 0, 255));
    
    // Getters
    cv::Mat getPerspectiveMatrix() const { return perspectiveMatrix; }
    cv::Mat getInvPerspectiveMatrix() const { return invPerspectiveMatrix; }
    std::vector<cv::Point2f> getOrigPoints() const { return origPoints; }
    std::vector<cv::Point2f> getDstPoints() const { return dstPoints; }

    // Calibrate based on camera parameters
    void calibrateFromCamera(
        float cameraHeight,   // in meters
        float cameraPitch,    // in degrees
        float horizontalFOV,  // in degrees
        float verticalFOV,    // in degrees
        float nearDistance,   // meters from camera
        float farDistance,    // meters from camera
        float laneWidth       // in meters
    );

    std::vector<cv::Point2f> calculateIPMPoints(
        float cameraHeight,       // in meters
        float cameraPitch,        // in degrees (down from horizontal)
        float horizontalFOV,      // in degrees
        float verticalFOV,        // in degrees
        const cv::Size& imageSize,
        float nearDistance,       // meters from camera
        float farDistance,        // meters from camera
        float laneWidth           // in meters
    );
    
    cv::Mat transformPoints(const cv::Mat& mask);

private:
    cv::Mat perspectiveMatrix;
    cv::Mat invPerspectiveMatrix;
    cv::Size origSize;
    cv::Size dstSize;
    std::vector<cv::Point2f> origPoints;
    std::vector<cv::Point2f> dstPoints;
    bool initialized;
    cv::Mat resizedInput; // Used for resizing if input doesn't match original size
};