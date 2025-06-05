#pragma once

#pragma once
#include <vector>
#include <queue>
#include <limits>
#include <opencv2/opencv.hpp>

/*
  ObstacleAvoidance:
    - Builds a coarse occupancy grid (drivable vs obstacle) from a binary segmentation mask.
    - Provides methods to:
        1) Detect where a given mid-curve (pixel coords) first intersects an obstacle cell.
        2) Find a lateral “bypass” pixel on that same row (searching left/right in drivable area).
        3) (Optionally) Run a simple Dijkstra/A* on the grid from (car grid cell) to (bypass grid cell).

  Usage:
    - Instantiate with image size, grid resolution (in pixels per grid cell), etc.
    - Every frame, call buildOccupancy(segmentation_mask) → this reconstructs a boolean[] occupancy_.
    - Then call detectFirstCollision(midCurve, out collisionIdx) → returns the index in midCurve of the first obstacle hit.
    - Then call findBypassOnRow(collisionRow, out bypassCol) → returns the best bypass pixel column.
    - If you want the entire path, call computeAstarPath(startGridIdx, bypassGridIdx).

  Internally:
    - We treat the origin (0,0) of the image as top-left.  Row increases downward (y), col increases rightward (x).
    - The grid is `gridWidth_ × gridHeight_`, where each cell covers `cellSizePx_ × cellSizePx_` pixels.
    - occupancy_[r*gridWidth_ + c] == true   → that grid cell is blocked (i.e. not drivable).
*/

class ObstacleAvoidance
{
public:

    ObstacleAvoidance();

    void init(int frameWidth, int frameHeight, int cellSizePx);


    /// Build the internal occupancy grid from a binary segmentation mask.
    /// segmentationMask: single-channel CV_8UC1 where 255=drivable, 0=not drivable/obstacle.
    void buildOccupancy(const cv::Mat& segmentationMask);

    /// Scan through midCurve (list of image-px points) to find the first index
    /// whose pixel lands in an occupied grid cell. Returns true if a collision was found.
    ///   midCurve: vector of (x,y) in image coords.  We assume these are sorted by increasing y.
    ///   outCollisionIdx: index in midCurve of the first obstacle pixel.  -1 if none.
    bool detectFirstCollision(const std::vector<cv::Point>& midCurve);

    /// Once you know the “collisionRow” = midCurve[outCollisionIdx].y,
    /// find (on that exact image-row) the nearest free column (x) that is (a) inside drivableMask
    /// and (b) not in occupancy.  We search left/right from the image-center ± laneHalfWidthPx.
    /// Returns true if a bypass was found, and sets outBypassX to that column (in pixel coords).
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

    // Temporary: store the last collision index and row
    bool    needBypass_    = false;
    int     collisionIdx_  = -1;
    int     collisionRow_  = -1;  // image-row (pixel) of first collision
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