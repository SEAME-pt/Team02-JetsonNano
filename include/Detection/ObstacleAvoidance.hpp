#pragma once

#pragma once
#include <vector>
#include <queue>
#include <limits>
#include <opencv2/opencv.hpp>


class ObstacleAvoidance
{
public:

    ObstacleAvoidance(int frameWidth, int frameHeight, int cellSizePx);


    void buildOccupancy(const cv::Mat& segmentationMask);
    void buildTrajectoryGrid(const std::vector<cv::Point>& trajectory);
    bool detectAllCollisions();
    std::vector<cv::Point> adjustTrajectory(const std::vector<cv::Point>& originalTrajectory);
    void smoothTrajectory(std::vector<cv::Point>& trajectory); 

    bool pixelToGrid(int px, int py, int& gridR, int& gridC) const;
    void gridToPixel(int gridR, int gridC, int& outPx, int& outPy) const;

    
    std::vector<std::pair<int,int>> computeAstarPath(int startR, int startC,
                                                     int goalR,  int goalC);

    /// Query methods:
    bool needBypass() const { return needBypass_; }
    cv::Point getBypassPixel() const { return cv::Point(bypassX_, collisionRow_); }
    int      getCollisionRow() const { return collisionRow_; }
    int      getCollisionIdx() const { return collisionIdx_; }

    void visualizeGrid(const std::vector<cv::Point>* adjustedTrajectory, 
                       cv::Mat& outputImage);

private:
    int frameWidth_,  frameHeight_;
    int cellSizePx_;             // # of pixels per grid cell (e.g. 8 px)
    int gridWidth_,   gridHeight_; // = frameWidth_/cellSizePx_, frameHeight_/cellSizePx_

    // occupancy_[r*gridWidth_ + c] == true  → cell is blocked / not drivable
    std::vector<bool> occupancy_;
    std::vector<std::vector<bool>> trajectoryGrid_;

    // Temporary: store the last collision index and row
    bool    needBypass_    = false;
    int     collisionIdx_  = -1;
    int     collisionRow_  = -1;
    int     collisionCol_  = -1;  // grid column of the collision
    int     collisionX_    = -1;  // pixel X coordinate of the collision
    int     collisionY_    = -1;  // pixel Y coordinate of the collision
    int     bypassX_       = -1;  // image-col for bypass (same row)

    int bypassGridCol_ = -1;
    int bypassGridRow_ = -1;
    int laneHalfWidthGridCells_ = 2;

    std::vector<std::pair<int, int>> collisionPoints_;
    std::vector<std::pair<int, int>> obstaclePoints_;

    const int proximityRadius_ = 15;

    std::vector<std::pair<int, int>> searchedCollisionPoints_; // Points checked during collision detection
    std::vector<std::pair<int, int>> searchedBypassPoints_;    // Points checked when finding bypass

    int safeDistancePx_ = 0;  // Will be calculated from proximityRadius_


    // internal helper to compute the 1D index in occupancy_[] from (r,c)
    inline int gridIndex(int r, int c) const { return r * gridWidth_ + c; }

    // A* helper: convert (r,c) to a single integer node id:  id = r * gridWidth_ + c
    //             gcost[node], fcost[node], parent[node]
    struct AStarNode {
        int idx;       // r*gridWidth_ + c
        double g, f;   // g = cost so far, f = g + h
    };
    struct CompareAStar {
        bool operator()(AStarNode const &a, AStarNode const &b) const {
            return a.f > b.f; // min-heap on f
        }
    };



};