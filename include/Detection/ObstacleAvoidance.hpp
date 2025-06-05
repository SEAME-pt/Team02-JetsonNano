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
    bool detectFirstCollision();
    bool findBypassOnRow(int collisionRow,
                         const cv::Mat& drivableMask,
                         int laneHalfWidthPx,
                         int& outBypassX) const;

    /// Convert a pixel (image) coordinate into a grid cell (r,c).
    /// Returns false if (x,y) is outside the local grid bounds.
    bool pixelToGrid(int px, int py, int& gridR, int& gridC) const;

    /// Convert a grid cell (r,c) back into the pixel coordinate
    /// at the center of that cell:
    void gridToPixel(int gridR, int gridC, int& outPx, int& outPy) const;

    /// (Optional) Run a simple A* on the occupancy grid from start→goal.
    ///   startR, startC: grid coords of the vehicle (usually bottom-center cell)
    ///   goalR,  goalC:  grid coords of the bypass waypoint
    /// Returns a list of grid cells (r,c) from start→goal (including both).  Empty if no path.
    std::vector<std::pair<int,int>> computeAstarPath(int startR, int startC,
                                                     int goalR,  int goalC);

    /// Query methods:
    bool needBypass() const { return needBypass_; }
    cv::Point getBypassPixel() const { return cv::Point(bypassX_, collisionRow_); }
    int      getCollisionRow() const { return collisionRow_; }
    int      getCollisionIdx() const { return collisionIdx_; }

    void visualizeGrid(
    bool showGridLines,
    const std::vector<cv::Point>& trajectory = std::vector<cv::Point>());

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