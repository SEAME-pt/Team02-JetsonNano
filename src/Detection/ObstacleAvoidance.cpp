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
bool ObstacleAvoidance::detectFirstCollision()
{
    needBypass_ = false;
    collisionIdx_ = -1;
    collisionRow_ = -1;
    collisionCol_ = -1;  // Add this member variable
    collisionX_ = -1;    // Add this for pixel X coordinate
    collisionY_ = -1;    // Add this for pixel Y coordinate
    
    // Calculate the starting point of the bottom "ignore zone"
    int ignoreZoneRowThreshold = static_cast<int>(frameHeight_ * 4.5 / 6.0) / cellSizePx_;

    for (int r = ignoreZoneRowThreshold; r >= 0; --r) {
        for (int c = 0; c < gridWidth_; ++c) {
            // If this cell is on the trajectory
            if (trajectoryGrid_[r][c]) {
                // Check proximity around the trajectory point
                for (int dr = -proximityRadius_; dr <= proximityRadius_; dr++) {
                    int checkR = r;
                    int checkC = c + dr;
                    
                    // Bounds checking
                    if (checkR < 0 || checkR >= gridHeight_ || 
                        checkC < 0 || checkC >= gridWidth_)
                        continue;

                    searchedCollisionPoints_.emplace_back(checkR, checkC);
                    
                    // If a nearby cell is occupied, it's a collision
                    if (occupancy_[gridIndex(checkR, checkC)]) {
                        // Found collision - store the trajectory point where we need to adjust
                        collisionRow_ = r;
                        collisionCol_ = c;
                        std::cout << "Collision detected at grid cell: (" 
                                  << collisionRow_ << ", " << collisionCol_ << ")\n";
                        // Convert grid coordinates back to pixel coordinates
                        gridToPixel(r, c, collisionX_, collisionY_);
                        
                        needBypass_ = true;
                        collisionIdx_ = 0;
                        
                        return true;
                    }
                }
            }
        }
    }
    // No collision
    return false;
}


bool ObstacleAvoidance::findBypassInGrid()
{
    bypassGridCol_ = -1;
    bypassGridRow_ = collisionRow_; // Using collision row as the bypass row
    
    // Start with collision location we already detected
    int gridRow = collisionRow_;
    int gridCol = collisionCol_;
    
    // Find middle of grid
    int centerGridCol = gridWidth_ / 2;
    
    // Check if we're on a valid row
    if (gridRow < 0 || gridRow >= gridHeight_)
        return false;
    
    // Determine which side to search first based on collision position
    bool searchRightFirst = (gridCol < centerGridCol);
    
    // Create a drivable mask for testing
    cv::Mat drivableMask = cv::Mat::ones(frameHeight_, frameWidth_, CV_8UC1) * 255;
    
    // Search with increasing distance from center
    for (int offset = 1; offset < gridWidth_/2; ++offset)
    {
        int leftCol = centerGridCol - laneHalfWidthGridCells_ - offset;
        int rightCol = centerGridCol + laneHalfWidthGridCells_ + offset;
        
        // Search in optimal direction (away from obstacle)
        if (searchRightFirst) {
            // Try right side first
            if (rightCol < gridWidth_) {
                searchedBypassPoints_.emplace_back(gridRow, rightCol);
                if (!occupancy_[gridIndex(gridRow, rightCol)]) {
                    bypassGridCol_ = rightCol;
                    return true;
                }
            }
            
            // Then left side
            if (leftCol >= 0) {
                searchedBypassPoints_.emplace_back(gridRow, leftCol);
                if (!occupancy_[gridIndex(gridRow, leftCol)]) {
                    bypassGridCol_ = leftCol;
                    return true;
                }
            }
        } else {
            // Try left first then right (same logic, reversed order)
            if (leftCol >= 0) {
                searchedBypassPoints_.emplace_back(gridRow, leftCol);
                if (!occupancy_[gridIndex(gridRow, leftCol)]) {
                    bypassGridCol_ = leftCol;
                    return true;
                }
            }
            
            if (rightCol < gridWidth_) {
                searchedBypassPoints_.emplace_back(gridRow, rightCol);
                if (!occupancy_[gridIndex(gridRow, rightCol)]) {
                    bypassGridCol_ = rightCol;
                    return true;
                }
            }
        }
    }
    // No bypass found
    return false;
}

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

void ObstacleAvoidance::gridToPixel(int gridR, int gridC, int& outPx, int& outPy) const
{
    outPx = gridC * cellSizePx_ + cellSizePx_ / 2;
    outPy = gridR * cellSizePx_ + cellSizePx_ / 2;
}

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

void ObstacleAvoidance::visualizeGrid()
{
    cv::Size actualSize(frameWidth_, frameHeight_);    
    
    // Create overlay at the target size directly
    cv::Mat overlay = cv::Mat::zeros(actualSize, CV_8UC3);
    // Draw each occupied cell
    for (int r = 0; r < gridHeight_; ++r) {
        for (int c = 0; c < gridWidth_; ++c) {

            int x0 = static_cast<int>(c * cellSizePx_);
            int y0 = static_cast<int>(r * cellSizePx_);
            int x1 = static_cast<int>(std::min((c+1) * cellSizePx_, frameWidth_));
            int y1 = static_cast<int>(std::min((r+1) * cellSizePx_, frameHeight_));
            

            if (trajectoryGrid_[r][c]) {
                cv::rectangle(overlay, cv::Point(x0, y0), cv::Point(x1, y1), 
                          cv::Scalar(255, 255, 0), -1); // Purple fill for trajectory cells
            }
            else if (occupancy_[gridIndex(r, c)]) {
                // Occupied cell (obstacle)
                cv::rectangle(overlay, cv::Point(x0, y0), cv::Point(x1, y1), 
                          cv::Scalar(0, 0, 100), -1); // Red fill
            }
            else {
                // Free cell
                cv::rectangle(overlay, cv::Point(x0, y0), cv::Point(x1, y1), 
                          cv::Scalar(0, 100, 0), -1); // Green fill
            }
        }
    }

    for (int r = 0; r < gridHeight_; ++r) {
        for (int c = 0; c < gridWidth_; ++c) {

            int x0 = static_cast<int>(c * cellSizePx_);
            int y0 = static_cast<int>(r * cellSizePx_);
            int x1 = static_cast<int>(std::min((c+1) * cellSizePx_, frameWidth_));
            int y1 = static_cast<int>(std::min((r+1) * cellSizePx_, frameHeight_));
            

            if (trajectoryGrid_[r][c]) {
                cv::rectangle(overlay, cv::Point(x0, y0), cv::Point(x1, y1), 
                        cv::Scalar(255, 255, 0), -1);
            }
        }
    }            

    // Draw grid lines with proper scaling
    for (int r = 0; r <= gridHeight_; ++r) {
        int y = static_cast<int>(r * cellSizePx_);
        cv::line(overlay, cv::Point(0, y), cv::Point(actualSize.width, y), 
                cv::Scalar(100, 100, 100), 1);
    }
    
    for (int c = 0; c <= gridWidth_; ++c) {
        int x = static_cast<int>(c * cellSizePx_);
        cv::line(overlay, cv::Point(x, 0), cv::Point(x, actualSize.height), 
                cv::Scalar(100, 100, 100), 1);
    }

    if (this->detectFirstCollision())
    {

        if (collisionRow_ >= 0 && collisionCol_ >= 0) 
        {
            // Get pixel coordinates for collision cell
            int collX0 = static_cast<int>(collisionCol_ * cellSizePx_);
            int collY0 = static_cast<int>(collisionRow_ * cellSizePx_);
            int collX1 = static_cast<int>(std::min((collisionCol_+1) * cellSizePx_, frameWidth_));
            int collY1 = static_cast<int>(std::min((collisionRow_+1) * cellSizePx_, frameHeight_));
            
            // Draw a distinctive red X over the collision cell
            cv::line(overlay, 
                    cv::Point(collX0, collY0), 
                    cv::Point(collX1, collY1), 
                    cv::Scalar(0, 0, 255), 3); // Red diagonal line
                    
            cv::line(overlay, 
                    cv::Point(collX0, collY1), 
                    cv::Point(collX1, collY0), 
                    cv::Scalar(0, 0, 255), 3); // Red diagonal line
            
            // Draw a border around the collision cell
            cv::rectangle(overlay, cv::Point(collX0, collY0), cv::Point(collX1, collY1), 
                        cv::Scalar(0, 0, 255), 2); // Red border
        }
        cv::putText(overlay, "Obstacle Detected", cv::Point(20, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
                    
        // Try to find a bypass with no parameters
        if (findBypassInGrid()) {
            // Draw the bypass point using member variables
            int bypassX0 = static_cast<int>(bypassGridCol_ * cellSizePx_);
            int bypassY0 = static_cast<int>(bypassGridRow_ * cellSizePx_);
            int bypassX1 = static_cast<int>(std::min((bypassGridCol_+1) * cellSizePx_, frameWidth_));
            int bypassY1 = static_cast<int>(std::min((bypassGridRow_+1) * cellSizePx_, frameHeight_));
            
            cv::rectangle(overlay, cv::Point(bypassX0, bypassY0), cv::Point(bypassX1, bypassY1), 
                         cv::Scalar(255, 255, 0), -1); // Yellow fill
                         
            // Draw a border around the bypass cell
            cv::rectangle(overlay, cv::Point(bypassX0, bypassY0), cv::Point(bypassX1, bypassY1), 
                         cv::Scalar(0, 255, 255), 2); // Cyan border
            
            // Add text indicating the bypass point
            cv::putText(overlay, "BYPASS", 
                       cv::Point(bypassX0, bypassY0 - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
        }
        else {
            cv::putText(overlay, "No bypass found!", cv::Point(20, 80),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }
    }
    else
    {
        cv::putText(overlay, "No Obstacle Detected", cv::Point(20, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }


    int ignoreZoneStart = static_cast<int>(frameHeight_ * 4.5 / 6.0);
    int scaledIgnoreZoneStart = static_cast<int>(ignoreZoneStart);
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

