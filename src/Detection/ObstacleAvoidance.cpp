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
    cv::Mat resizedMask;
    cv::resize(segmentationMask, resizedMask,
                 cv::Size(frameWidth_, frameHeight_), 0, 0, cv::INTER_NEAREST);
    std::fill(occupancy_.begin(), occupancy_.end(), false);

    for (int r = 0; r < gridHeight_; ++r)
    {
        int y0 = r * cellSizePx_;
        int y1 = std::min(y0 + cellSizePx_, frameHeight_);
        for (int c = 0; c < gridWidth_; ++c)
        {
            int x0 = c * cellSizePx_;
            int x1 = std::min(x0 + cellSizePx_, frameWidth_);
            bool anyNonRoad = false;

            // scan the pixel block (y0..y1-1, x0..x1-1)
            for (int yy = y0; yy < y1 && !anyNonRoad; ++yy)
            {
                for (int xx = x0; xx < x1; ++xx)
                {
                    cv::Vec3b pixel = resizedMask.at<cv::Vec3b>(yy, xx);
                    
                    // Check if this is a non-road pixel
                    if (!(pixel == cv::Vec3b(128, 64, 128)))
                    {
                        anyNonRoad = true;
                        break;
                    }
                }
            }
            
            // If any non-road pixel was found, mark cell as occupied
            occupancy_[gridIndex(r,c)] = anyNonRoad;
        }
    }
}

//
// detectFirstCollision:
//   Walk down midCurve (assumed sorted by increasing y).  The first midCurve[i] whose pixel
//   falls inside an occupied grid cell (via pixelToGrid → occupancy) is the collision index.
//
bool ObstacleAvoidance::detectFirstCollision(const std::vector<cv::Point>& midCurve)
{
    needBypass_     = false;
    collisionIdx_   = -1;
    collisionRow_   = -1;
    
    // Calculate the starting point of the bottom "ignore zone" (5/6 of the height)
    int ignoreZoneStart = static_cast<int>(frameHeight_ * 4.5 / 6.0);

    for (int i = 0; i < (int)midCurve.size(); ++i)
    {
        const cv::Point &p = midCurve[i];
        
        if (p.y >= ignoreZoneStart)
            continue;
            
        // Make sure the point is in the image
        if (p.x < 0 || p.x >= frameWidth_ || p.y < 0 || p.y >= frameHeight_)
            continue;

        int gr, gc;
        if (!pixelToGrid(p.x, p.y, gr, gc))
            continue;

        if (occupancy_[gridIndex(gr,gc)])
        {
            // found first collision
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

void ObstacleAvoidance::buildTrajectoryGrid(const std::vector<cv::Point>& trajectory)
{
    std::vector<std::vector<bool>> trajectoryGrid(gridHeight_, std::vector<bool>(gridWidth_, false));
    if (!trajectory.empty()) {
        for (const auto& p : trajectory) {
            int gr, gc;
            if (pixelToGrid(p.x, p.y, gr, gc)) {
                trajectoryGrid[gr][gc] = true;
            }
        }
    }
    trajectoryGrid_ = std::move(trajectoryGrid);
}

void ObstacleAvoidance::visualizeGrid( 
    bool showGridLines,
    const std::vector<cv::Point>& trajectory)
{
    // // Create overlay for the occupancy grid
    // cv::Size overlaySize(frameWidth_, frameHeight_);
    // cv::Mat overlay = cv::Mat::zeros(overlaySize, CV_8UC3);

    cv::Size actualSize(800, 600);    
    // Calculate scaling factors
    double scaleX = static_cast<double>(actualSize.width) / frameWidth_;
    double scaleY = static_cast<double>(actualSize.height) / frameHeight_;
    
    // Create overlay at the target size directly
    cv::Mat overlay = cv::Mat::zeros(actualSize, CV_8UC3);
    // Draw each occupied cell
    for (int r = 0; r < gridHeight_; ++r) {
        for (int c = 0; c < gridWidth_; ++c) {

            int x0 = static_cast<int>(c * cellSizePx_ * scaleX);
            int y0 = static_cast<int>(r * cellSizePx_ * scaleY);
            int x1 = static_cast<int>(std::min((c+1) * cellSizePx_, frameWidth_) * scaleX);
            int y1 = static_cast<int>(std::min((r+1) * cellSizePx_, frameHeight_) * scaleY);
            

            if (trajectoryGrid_[r][c]) {
                cv::rectangle(overlay, cv::Point(x0, y0), cv::Point(x1, y1), 
                          cv::Scalar(255, 255, 0), -1); // Purple fill for trajectory cells
            }
            else if (occupancy_[gridIndex(r, c)]) {
                // Occupied cell (obstacle)
                cv::rectangle(overlay, cv::Point(x0, y0), cv::Point(x1, y1), 
                          cv::Scalar(0, 0, 255), -1); // Red fill
            }
            else {
                // Free cell
                cv::rectangle(overlay, cv::Point(x0, y0), cv::Point(x1, y1), 
                          cv::Scalar(0, 255, 0), -1); // Green fill
            }
        }
    }
    
    // Draw grid lines with proper scaling
    if (showGridLines) {
        for (int r = 0; r <= gridHeight_; ++r) {
            int y = static_cast<int>(r * cellSizePx_ * scaleY);
            cv::line(overlay, cv::Point(0, y), cv::Point(actualSize.width, y), 
                    cv::Scalar(255, 255, 255), 1);
        }
        
        for (int c = 0; c <= gridWidth_; ++c) {
            int x = static_cast<int>(c * cellSizePx_ * scaleX);
            cv::line(overlay, cv::Point(x, 0), cv::Point(x, actualSize.height), 
                    cv::Scalar(255, 255, 255), 1);
        }
    }

    //  // Visualize trajectory if provided
    // // Draw trajectory points and lines (at scaled coordinates)
    // if (!trajectory.empty()) {
    //     std::vector<cv::Point> gridTrajectory;
        
    //     for (const auto& p : trajectory) {
    //         int gr, gc;
    //         int x = static_cast<int>(p.x * scaleX);
    //         int y = static_cast<int>(p.y * scaleY);
    //         if (pixelToGrid(x, y, gr, gc)) {
    //             // Get cell center coordinates (in original space)
    //             int centerX, centerY;
    //             gridToPixel(gr, gc, centerX, centerY);
                
    //             // Mark with blue circle (scaled size)
    //             int circleRadius = static_cast<int>(cellSizePx_ * scaleX / 3);
    //             cv::circle(overlay, cv::Point(centerX, centerY), circleRadius, 
    //                       cv::Scalar(255, 0, 0), -1);
                
    //             gridTrajectory.push_back(cv::Point(centerX, centerY));
    //         }
    //     }
        
    //     // Connect trajectory points with lines
    //     if (gridTrajectory.size() > 1) {
    //         for (size_t i = 0; i < gridTrajectory.size() - 1; i++) {
    //             cv::line(overlay, gridTrajectory[i], gridTrajectory[i+1], 
    //                     cv::Scalar(255, 255, 0), 2);
    //         }
    //     }
    // }
    // cv::addWeighted(overlay, 1, visualImage, 0, 0, visualImage);
    
    if (this->detectFirstCollision(trajectory))
    {
        cv::putText(overlay, "Obstacle Detected", cv::Point(20, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }
    else
    {
        cv::putText(overlay, "No Obstacle Detected", cv::Point(20, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }


    int ignoreZoneStart = static_cast<int>(frameHeight_ * 4.5 / 6.0);
    int scaledIgnoreZoneStart = static_cast<int>(ignoreZoneStart * scaleY);
    cv::rectangle(overlay, 
                cv::Point(0, scaledIgnoreZoneStart), 
                cv::Point(actualSize.width, actualSize.height),
                cv::Scalar(100, 100, 100), // Gray color
                -1); // Filled rectangle

    cv::line(overlay, 
            cv::Point(0, scaledIgnoreZoneStart), 
            cv::Point(actualSize.width, scaledIgnoreZoneStart),
            cv::Scalar(255, 0, 255), 2); // Magenta line

    cv::putText(overlay, "Ignore Zone", 
                cv::Point(20, scaledIgnoreZoneStart + 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                cv::Scalar(255, 255, 255), 2);

    cv::imshow("Occupancy Grid", overlay);
    cv::waitKey(1);
}

