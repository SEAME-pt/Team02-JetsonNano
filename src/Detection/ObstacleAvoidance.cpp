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
    std::cout << "Resized mask to: " << resizedMask.size() << std::endl;
    std::fill(occupancy_.begin(), occupancy_.end(), false);
    std::cout << "Building occupancy grid with dimensions: "
              << gridWidth_ << "x" << gridHeight_ << std::endl;

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

bool ObstacleAvoidance::detectAllCollisions()
{
    needBypass_ = false;
    collisionIdx_ = -1;
    collisionRow_ = -1;
    collisionCol_ = -1;
    collisionX_ = -1;
    collisionY_ = -1;
    
    // Clear previous collision points
    collisionPoints_.clear();
    searchedCollisionPoints_.clear();
    
    // Calculate the starting point of the bottom "ignore zone"
    int ignoreZoneRowThreshold = static_cast<int>(frameHeight_ * 4.5 / 6.0) / cellSizePx_;

    // Track whether we found any collisions
    bool foundCollision = false;
    
    // First, search from the bottom up to find the lowest collision
    for (int r = ignoreZoneRowThreshold - proximityRadius_ - 1; r >= 0; --r) {
        for (int c = 0; c < gridWidth_; ++c) {
            // If this cell is on the trajectory
            if (trajectoryGrid_[r][c]) {
                // Check proximity around the trajectory point
                for (int dr = -proximityRadius_; dr <= proximityRadius_; dr++) {
                    for (int dc = -proximityRadius_; dc <= proximityRadius_; dc++) {
                        int checkR = r + dr;
                        int checkC = c + dc;
                        
                        // Bounds checking
                        if (checkR < 0 || checkR >= gridHeight_ || 
                            checkC < 0 || checkC >= gridWidth_)
                            continue;

                        searchedCollisionPoints_.emplace_back(checkR, checkC);
                        
                        // If a nearby cell is occupied, it's a collision
                        if (occupancy_[gridIndex(checkR, checkC)]) {
                            // Store this collision point
                            collisionPoints_.emplace_back(r, c);
                            obstaclePoints_.emplace_back(checkR, checkC);
                            
                            // If this is the first collision found, set it as the primary one
                            if (!foundCollision) {
                                collisionRow_ = r;
                                collisionCol_ = c;
                                gridToPixel(r, c, collisionX_, collisionY_);
                                needBypass_ = true;
                                foundCollision = true;
                            }
                            
                            // Continue searching for more collisions
                            continue;
                        }
                    }
                }
            }
        }
    }
    
    // Sort collision points by distance from bottom (closest to car first)
    std::sort(collisionPoints_.begin(), collisionPoints_.end(), 
              [](const auto& a, const auto& b) { return a.first > b.first; });
    
    // if (foundCollision) {
    //     std::cout << "Found " << collisionPoints_.size() << " collision points. Lowest at: ("
    //               << collisionRow_ << ", " << collisionCol_ << ")\n";
    // }
    
    return foundCollision;
}


std::vector<cv::Point> ObstacleAvoidance::adjustTrajectory(const std::vector<cv::Point>& originalTrajectory)
{
    if (collisionPoints_.empty() || originalTrajectory.empty()) {
        return originalTrajectory;
    }
    
    // Create a copy of the trajectory we'll modify
    std::vector<cv::Point> adjustedTrajectory = originalTrajectory;
    
    // Convert proximity radius from grid cells to pixels
    safeDistancePx_ = proximityRadius_ * cellSizePx_;
    int safeDistanceCells = proximityRadius_ + 1; // Add safety margin
    
    // Create a direct mapping from grid cells to trajectory point indices
    std::map<std::pair<int, int>, std::vector<size_t>> gridToTrajectoryPoints;
    for (size_t i = 0; i < originalTrajectory.size(); i++) {
        int r, c;
        if (pixelToGrid(originalTrajectory[i].x, originalTrajectory[i].y, r, c)) {
            gridToTrajectoryPoints[{r, c}].push_back(i);
        }
    }
    
    // Iterate directly through the mapping we just created
    for (const auto& entry : gridToTrajectoryPoints) {
        int r = entry.first.first;   // Grid row
        int c = entry.first.second;  // Grid column
        const auto& pointIndices = entry.second;  // Indices of trajectory points at this grid location
        
            
            // Find obstacles in this row
            std::vector<int> obstacleColumns;
            for (int col = 0; col < gridWidth_; col++) {
                if (occupancy_[gridIndex(r, col)]) {
                    obstacleColumns.push_back(col);
                }
            }
            
            // Skip if no obstacles in this row
            if (obstacleColumns.empty()) {
                continue;
            }
            
            // Find closest obstacle column
            int closestObstacleCol = -1;
            int minDistance = gridWidth_;
            for (int obsCol : obstacleColumns) {
                int distance = std::abs(c - obsCol);
                if (distance < minDistance) {
                    minDistance = distance;
                    closestObstacleCol = obsCol;
                }
            }
            
            // Skip if already at safe distance
            if (minDistance >= safeDistanceCells) {
                continue;
            }
            
            // Determine which side has more drivable area
            int leftFreeSpace = 0;
            int rightFreeSpace = 0;
            
            // Count free cells to the left
            for (int col = closestObstacleCol - 1; col >= 0; col--) {
                if (!occupancy_[gridIndex(r, col)]) {
                    leftFreeSpace++;
                } else {
                    break; // Stop at first obstacle
                }
            }
            
            // Count free cells to the right
            for (int col = closestObstacleCol + 1; col < gridWidth_; col++) {
                if (!occupancy_[gridIndex(r, col)]) {
                    rightFreeSpace++;
                } else {
                    break; // Stop at first obstacle
                }
            }
            
            std::cout << "Row " << r << ", Traj col " << c << ", Obstacle col " << closestObstacleCol 
                     << ", Left space: " << leftFreeSpace << ", Right space: " << rightFreeSpace << std::endl;
            
            // Decide which way to move based on available space
            int newCol;
            if (c < closestObstacleCol) {
                // Trajectory is left of obstacle, stay left if possible
                if (leftFreeSpace >= safeDistanceCells) {
                    newCol = closestObstacleCol - safeDistanceCells;
                } else if (rightFreeSpace > leftFreeSpace + safeDistanceCells) {
                    // Not enough space on left, go right if significantly more space
                    newCol = closestObstacleCol + safeDistanceCells;
                } else {
                    // Stay left but as far as possible
                    newCol = std::max(0, closestObstacleCol - leftFreeSpace);
                }
            } else {
                // Trajectory is right of obstacle, stay right if possible
                if (rightFreeSpace >= safeDistanceCells) {
                    newCol = closestObstacleCol + safeDistanceCells;
                } else if (leftFreeSpace > rightFreeSpace + safeDistanceCells) {
                    // Not enough space on right, go left if significantly more space
                    newCol = closestObstacleCol - safeDistanceCells;
                } else {
                    // Stay right but as far as possible
                    newCol = std::min(gridWidth_ - 1, closestObstacleCol + rightFreeSpace);
                }
            }
            
            // Convert new position back to pixel coordinates
            int newX, newY;
            gridToPixel(r, newCol, newX, newY);
            
            // Apply the adjustment to all trajectory points in this grid cell
            for (size_t idx : it->second) {
                // Keep the original y-coordinate for smooth vertical movement
                adjustedTrajectory[idx].x = newX;
                
                std::cout << "Adjusted trajectory point " << idx << " from col " << c 
                         << " to col " << newCol << std::endl;
            }
        }
    }
    
    // Apply a smoothing filter to prevent jerky movements
    // smoothTrajectory(adjustedTrajectory);
    
    return adjustedTrajectory;
}

// Add this method to apply smoothing to the trajectory
void ObstacleAvoidance::smoothTrajectory(std::vector<cv::Point>& trajectory) 
{
    if (trajectory.size() < 3) return;
    
    std::vector<cv::Point> smoothed = trajectory;
    
    // Simple moving average filter
    const int windowSize = 5;
    
    for (size_t i = windowSize/2; i < trajectory.size() - windowSize/2; i++) {
        int sumX = 0;
        int sumY = 0;
        
        for (int j = -windowSize/2; j <= windowSize/2; j++) {
            sumX += trajectory[i + j].x;
            sumY += trajectory[i + j].y;
        }
        
        smoothed[i].x = sumX / windowSize;
        smoothed[i].y = sumY / windowSize;
    }
    
    trajectory = smoothed;
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

void ObstacleAvoidance::visualizeGrid(const std::vector<cv::Point>* adjustedTrajectory, cv::Mat& outputImage)
{
    cv::Size actualSize(frameWidth_, frameHeight_);    
    
    // Create overlay at the target size directly
    cv::Mat overlay = cv::Mat::zeros(actualSize, CV_8UC3);
    // Draw each occupied cell
    // for (int r = 0; r < gridHeight_; ++r) {
    //     for (int c = 0; c < gridWidth_; ++c) {

    //         int x0 = static_cast<int>(c * cellSizePx_);
    //         int y0 = static_cast<int>(r * cellSizePx_);
    //         int x1 = static_cast<int>(std::min((c+1) * cellSizePx_, frameWidth_));
    //         int y1 = static_cast<int>(std::min((r+1) * cellSizePx_, frameHeight_));
            

    //         if (trajectoryGrid_[r][c]) {
    //             cv::rectangle(overlay, cv::Point(x0, y0), cv::Point(x1, y1), 
    //                       cv::Scalar(255, 255, 0), -1); // Purple fill for trajectory cells
    //         }
    //         else if (occupancy_[gridIndex(r, c)]) {
    //             // Occupied cell (obstacle)
    //             cv::rectangle(overlay, cv::Point(x0, y0), cv::Point(x1, y1), 
    //                       cv::Scalar(0, 0, 150), -1); // Red fill
    //         }
    //         else {
    //             // Free cell
    //             cv::rectangle(overlay, cv::Point(x0, y0), cv::Point(x1, y1), 
    //                       cv::Scalar(0, 150, 0), -1); // Green fill
    //         }
    //     }
    // }

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
    // for (int r = 0; r <= gridHeight_; ++r) {
    //     int y = static_cast<int>(r * cellSizePx_);
    //     cv::line(overlay, cv::Point(0, y), cv::Point(actualSize.width, y), 
    //             cv::Scalar(150, 150, 150), 1);
    // }
    
    // for (int c = 0; c <= gridWidth_; ++c) {
    //     int x = static_cast<int>(c * cellSizePx_);
    //     cv::line(overlay, cv::Point(x, 0), cv::Point(x, actualSize.height), 
    //             cv::Scalar(150, 150, 150), 1);
    // }

    if (this->detectAllCollisions())
    {
        cv::putText(overlay, "Obstacles Detected: " + std::to_string(obstaclePoints_.size()),
                    cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        
        // Visualize all collision points
        for (size_t i = 0; i < obstaclePoints_.size(); i++) {
            int r = obstaclePoints_[i].first;
            int c = obstaclePoints_[i].second;
            
            // Get pixel coordinates for collision cell
            int collX0 = static_cast<int>(c * cellSizePx_);
            int collY0 = static_cast<int>(r * cellSizePx_);
            int collX1 = static_cast<int>(std::min((c+1) * cellSizePx_, frameWidth_));
            int collY1 = static_cast<int>(std::min((r+1) * cellSizePx_, frameHeight_));
            
            // Fill collision cell with bright color and add border
            cv::rectangle(overlay, cv::Point(collX0, collY0), cv::Point(collX1, collY1), 
                        cv::Scalar(255, 0, 255), -1);  // Fill with bright magenta
                        
            // Add white border for contrast
            cv::rectangle(overlay, cv::Point(collX0, collY0), cv::Point(collX1, collY1), 
                        cv::Scalar(255, 255, 255), 2);  // White border
        }
        // cv::putText(overlay, "Obstacle Detected", cv::Point(20, 40),
        //             cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }
    else
    {
        // cv::putText(overlay, "No Obstacle Detected", cv::Point(20, 40),
        //             cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }

    if (adjustedTrajectory && !adjustedTrajectory->empty()) {
        cv::putText(overlay, "Adjusted Trajectory", cv::Point(20, 120),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        
        // Connect points with lines
        for (size_t i = 0; i < adjustedTrajectory->size() - 1; i++) {
            cv::line(overlay, 
                    (*adjustedTrajectory)[i], 
                    (*adjustedTrajectory)[i+1],
                    cv::Scalar(0, 255, 255), 2); // Cyan line
        }
        
        // Draw points
        for (const auto& p : *adjustedTrajectory) {
            cv::circle(overlay, p, 3, cv::Scalar(0, 255, 255), -1);
        }
    }

    // int ignoreZoneStart = static_cast<int>(frameHeight_ * 4.5 / 6.0);
    // int scaledIgnoreZoneStart = static_cast<int>(ignoreZoneStart);
    // cv::rectangle(overlay, 
    //             cv::Point(0, scaledIgnoreZoneStart), 
    //             cv::Point(actualSize.width, actualSize.height),
    //             cv::Scalar(100, 100, 100), // Gray color
    //             -1); // Filled rectangle

    // cv::line(overlay, 
    //         cv::Point(0, scaledIgnoreZoneStart), 
    //         cv::Point(actualSize.width, scaledIgnoreZoneStart),
    //         cv::Scalar(255, 0, 255), 2); // Magenta line

    // cv::putText(overlay, "Ignore Zone", 
    //             cv::Point(20, scaledIgnoreZoneStart + 30),
    //             cv::FONT_HERSHEY_SIMPLEX, 0.7, 
    //             cv::Scalar(255, 255, 255), 2);

    overlay.copyTo(outputImage);
}

