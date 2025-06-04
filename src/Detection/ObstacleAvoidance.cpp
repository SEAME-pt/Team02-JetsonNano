#include "ObstacleAvoidance.hpp"
#include <algorithm>
#include <cmath>

//
// Constructor: initialize grid dimensions
//
ObstacleAvoidance::ObstacleAvoidance(int frameW, int frameH, int cellSizePx)
    : frameWidth_(frameW),
      frameHeight_(frameH),
      cellSizePx_(cellSizePx)
{
    // Compute how many grid cells horizontally/vertically
    gridWidth_  = (frameWidth_  + cellSizePx_ - 1) / cellSizePx_;
    gridHeight_ = (frameHeight_ + cellSizePx_ - 1) / cellSizePx_;
    occupancy_.resize(gridWidth_ * gridHeight_, false);
}

//
// buildOccupancy:
//   - segmentationMask: single-channel CV_8UC1 (255=drivable, 0=not drivable).
//   - We bucket each cellSizePx × cellSizePx square of pixels into one grid cell.
//   - If ANY pixel in that block is zero (non-drivable), we mark the grid cell as occupied.
//
void ObstacleAvoidance::buildOccupancy(const cv::Mat& segmentationMask)
{
    CV_Assert(segmentationMask.type() == CV_8UC1);
    // Clear previous occupancy
    std::fill(occupancy_.begin(), occupancy_.end(), false);

    for (int r = 0; r < gridHeight_; ++r)
    {
        int y0 = r * cellSizePx_;
        int y1 = std::min(y0 + cellSizePx_, frameHeight_);
        for (int c = 0; c < gridWidth_; ++c)
        {
            int x0 = c * cellSizePx_;
            int x1 = std::min(x0 + cellSizePx_, frameWidth_);
            bool anyBlocked = false;

            // scan the pixel block (y0..y1-1, x0..x1-1)
            for (int yy = y0; yy < y1 && !anyBlocked; ++yy)
            {
                const uchar* rowPtr = segmentationMask.ptr<uchar>(yy);
                for (int xx = x0; xx < x1; ++xx)
                {
                    if (rowPtr[xx] == 0) // sees non-drivable
                    {
                        anyBlocked = true;
                        break;
                    }
                }
            }
            occupancy_[gridIndex(r,c)] = anyBlocked;
        }
    }
}

//
// detectFirstCollision:
//   Walk down midCurve (assumed sorted by increasing y).  The first midCurve[i] whose pixel
//   falls inside an occupied grid cell (via pixelToGrid → occupancy) is the collision index.
//
bool ObstacleAvoidance::detectFirstCollision(const std::vector<cv::Point>& midCurve,
                                             int& outCollisionIdx) const
{
    outCollisionIdx = -1;
    needBypass_     = false;
    collisionIdx_   = -1;
    collisionRow_   = -1;

    for (int i = 0; i < (int)midCurve.size(); ++i)
    {
        const cv::Point &p = midCurve[i];
        // Make sure the point is in the image
        if (p.x < 0 || p.x >= frameWidth_ || p.y < 0 || p.y >= frameHeight_)
            continue;

        int gr, gc;
        if (!pixelToGrid(p.x, p.y, gr, gc))
            continue;

        if (occupancy_[gridIndex(gr,gc)])
        {
            // found first collision
            outCollisionIdx = i;
            collisionIdx_   = i;
            collisionRow_   = p.y;
            needBypass_     = true;
            return true;
        }
    }

    // no collision
    return false;
}

//
// findBypassOnRow:
//   Given an image row (= collisionRow), search left/right from (centerX ± laneHalfWidthPx)
//   until we find a pixel that is (a) inside the image bounds, (b) drivableMask == 255,
//   and (c) maps to an unoccupied grid cell.  We return that x as bypassX.
//
bool ObstacleAvoidance::findBypassOnRow(int collisionRow,
                                        const cv::Mat& drivableMask,
                                        int laneHalfWidthPx,
                                        int& outBypassX) const
{
    outBypassX = -1;
    if (collisionRow < 0 || collisionRow >= frameHeight_)
        return false;

    int centerX = frameWidth_ / 2;
    // Search offset from 1 .. until image boundary
    for (int offset = 1; offset < frameWidth_/2; ++offset)
    {
        // try left candidate
        int xL = centerX - laneHalfWidthPx - offset;
        if (xL >= 0)
        {
            // check drivableMask at (collisionRow, xL)
            if (drivableMask.at<uchar>(collisionRow, xL) == 255)
            {
                // also check occupancy grid
                int gr, gc;
                if (pixelToGrid(xL, collisionRow, gr, gc)
                    && !occupancy_[gridIndex(gr,gc)])
                {
                    outBypassX = xL;
                    return true;
                }
            }
        }

        // try right candidate
        int xR = centerX + laneHalfWidthPx + offset;
        if (xR < frameWidth_)
        {
            if (drivableMask.at<uchar>(collisionRow, xR) == 255)
            {
                int gr, gc;
                if (pixelToGrid(xR, collisionRow, gr, gc)
                    && !occupancy_[gridIndex(gr,gc)])
                {
                    outBypassX = xR;
                    return true;
                }
            }
        }
    }

    // No bypass found
    return false;
}

//
// pixelToGrid:
//   Convert (px,py) in image coords to grid (r,c).  We define:
//
//     c = px / cellSizePx_
//     r = py / cellSizePx_
//
//   If that pixel lies outside [0..frameW-1] or [0..frameH-1], or the computed (r,c)
//   is outside [0..gridHeight_-1]×[0..gridWidth_-1], we return false.
//
bool ObstacleAvoidance::pixelToGrid(int px, int py, int& gr, int& gc) const
{
    if (px < 0 || px >= frameWidth_ || py < 0 || py >= frameHeight_)
        return false;
    gc = px / cellSizePx_;
    gr = py / cellSizePx_;
    if (gc < 0 || gc >= gridWidth_ || gr < 0 || gr >= gridHeight_)
        return false;
    return true;
}

//
// gridToPixel:
//   Convert a grid cell center (r,c) back to pixel coords (px,py).  We choose the center
//   of that cell: px = c*cellSizePx + cellSizePx/2,  py = r*cellSizePx + cellSizePx/2.
//
void ObstacleAvoidance::gridToPixel(int gridR, int gridC, int& outPx, int& outPy) const
{
    outPx = gridC * cellSizePx_ + cellSizePx_ / 2;
    outPy = gridR * cellSizePx_ + cellSizePx_ / 2;
}

//
// computeAstarPath:
//   A basic A* on the grid.  We treat each free cell as a node.  8‐connected neighbors
//   (dx,dy ∈ {−1,0,1}, not both zero).  The cost of a straight neighbor = 1.0, diagonal=1.414.
//   Heuristic = Euclidean distance to goal.
//
//   Returns a vector of (r,c) from (startR,startC) → (goalR,goalC).  Empty if no path.
//
std::vector<std::pair<int,int>> ObstacleAvoidance::computeAstarPath(int startR, int startC,
                                                                     int goalR,  int goalC)
{
    std::vector<std::pair<int,int>> emptyPath;
    // Bounds check
    if (startR < 0 || startR >= gridHeight_ || startC < 0 || startC >= gridWidth_) return emptyPath;
    if (goalR  < 0 || goalR  >= gridHeight_ || goalC  < 0 || goalC  >= gridWidth_) return emptyPath;

    int N = gridWidth_ * gridHeight_;
    auto idxRC = [&](int r, int c){ return r * gridWidth_ + c; };

    // If start or goal is occupied, no path
    if (occupancy_[idxRC(startR,startC)] || occupancy_[idxRC(goalR,goalC)])
        return emptyPath;

    std::vector<double> gcost(N, std::numeric_limits<double>::infinity());
    std::vector<int> parent(N, -1);
    std::vector<bool> closed(N, false);

    // Min-heap of (idx, g, f)
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareAStar> openSet;
    int startIdx = idxRC(startR, startC);
    int goalIdx  = idxRC(goalR,  goalC);

    // Heuristic function (Euclidean):
    auto heuristic = [&](int r, int c){
        double dr = double(r - goalR);
        double dc = double(c - goalC);
        return std::sqrt(dr*dr + dc*dc);
    };

    gcost[startIdx] = 0.0;
    double h0 = heuristic(startR, startC);
    openSet.push({ startIdx, 0.0, h0 });

    // 8‐connected neighbor offsets
    const int dR[8] = { -1, +1,  0,  0, -1, -1, +1, +1 };
    const int dC[8] = {  0,  0, -1, +1, -1, +1, -1, +1 };

    while (!openSet.empty())
    {
        AStarNode node = openSet.top(); 
        openSet.pop();
        int uIdx = node.idx;
        if (closed[uIdx]) continue;
        closed[uIdx] = true;

        if (uIdx == goalIdx) break;

        int ur = uIdx / gridWidth_;
        int uc = uIdx % gridWidth_;

        for (int k = 0; k < 8; ++k)
        {
            int vr = ur + dR[k];
            int vc = uc + dC[k];
            if (vr < 0 || vr >= gridHeight_ || vc < 0 || vc >= gridWidth_) continue;
            int vIdx = idxRC(vr, vc);
            if (occupancy_[vIdx]) continue; // blocked

            double stepCost = (k < 4) ? 1.0 : 1.41421356; // straight vs diagonal
            double tentative = gcost[uIdx] + stepCost;
            if (tentative < gcost[vIdx])
            {
                gcost[vIdx] = tentative;
                parent[vIdx] = uIdx;
                double h = heuristic(vr, vc);
                openSet.push({ vIdx, tentative, tentative + h });
            }
        }
    }

    // Reconstruct path if goal is reached
    if (!closed[goalIdx]) 
        return emptyPath; // no route found

    std::vector<std::pair<int,int>> path;
    for (int cur = goalIdx; cur != -1; cur = parent[cur])
    {
        int r = cur / gridWidth_;
        int c = cur % gridWidth_;
        path.emplace_back(r, c);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// Add this implementation to ObstacleAvoidance.cpp:
void ObstacleAvoidance::visualizeGrid(cv::Mat& visualImage, bool showGridLines) const
{
    // Create overlay for the occupancy grid
    cv::Mat overlay = visualImage.clone();
    
    // Draw each occupied cell
    for (int r = 0; r < gridHeight_; ++r) {
        for (int c = 0; c < gridWidth_; ++c) {
            if (occupancy_[gridIndex(r, c)]) {
                // Calculate pixel region for this cell
                int x0 = c * cellSizePx_;
                int y0 = r * cellSizePx_;
                int x1 = std::min(x0 + cellSizePx_, frameWidth_);
                int y1 = std::min(y0 + cellSizePx_, frameHeight_);
                
                // Draw filled rectangle for occupied cells
                cv::rectangle(overlay, cv::Point(x0, y0), cv::Point(x1, y1), 
                              cv::Scalar(0, 0, 255), -1); // Red fill
            }
        }
    }
    
    // Blend with original image
    cv::addWeighted(overlay, 0.4, visualImage, 0.6, 0, visualImage);
    
    // Draw grid lines
    if (showGridLines) {
        // Draw horizontal grid lines
        for (int r = 0; r <= gridHeight_; ++r) {
            int y = r * cellSizePx_;
            cv::line(visualImage, cv::Point(0, y), cv::Point(frameWidth_, y), 
                     cv::Scalar(255, 255, 255), 1);
        }
        
        // Draw vertical grid lines
        for (int c = 0; c <= gridWidth_; ++c) {
            int x = c * cellSizePx_;
            cv::line(visualImage, cv::Point(x, 0), cv::Point(x, frameHeight_), 
                     cv::Scalar(255, 255, 255), 1);
        }
    }
    
    // Add text showing grid dimensions
    std::string gridText = "Grid: " + std::to_string(gridWidth_) + "x" + 
                           std::to_string(gridHeight_) + " (Cell: " + 
                           std::to_string(cellSizePx_) + "px)";
    cv::putText(visualImage, gridText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                0.6, cv::Scalar(255, 255, 255), 2);
}

