#include "ObstacleAvoidance.hpp"
#include <algorithm>
#include <cmath>

//
// Constructor: initialize grid dimensions
//
ObstacleAvoidance::ObstacleAvoidance(int frameW, int frameH, int cellSizePx)
    : frameWidth_(frameW), frameHeight_(frameH), cellSizePx_(cellSizePx)
{
    // Compute how many grid cells horizontally/vertically
    gridWidth_  = (frameWidth_ + cellSizePx_ - 1) / cellSizePx_;
    gridHeight_ = (frameHeight_ + cellSizePx_ - 1) / cellSizePx_;
    occupancy_.resize(gridWidth_ * gridHeight_, false);
}

//
// buildOccupancy:
//   - segmentationMask: single-channel CV_8UC1 (255=drivable, 0=not drivable).
//   - We bucket each cellSizePx × cellSizePx square of pixels into one grid
//   cell.
//   - If ANY pixel in that block is zero (non-drivable), we mark the grid cell
//   as occupied.
//
void ObstacleAvoidance::buildOccupancy(const cv::Mat& segmentationMask)
{
    cv::Mat resizedMask;
    cv::resize(segmentationMask, resizedMask,
               cv::Size(frameWidth_, frameHeight_), 0, 0, cv::INTER_NEAREST);
    std::cout << "Resized mask to: " << resizedMask.size() << std::endl;
    std::fill(occupancy_.begin(), occupancy_.end(), false);
    std::cout << "Building occupancy grid with dimensions: " << gridWidth_
              << "x" << gridHeight_ << std::endl;

    for (int r = 0; r < gridHeight_; ++r)
    {
        int y0 = r * cellSizePx_;
        int y1 = std::min(y0 + cellSizePx_, frameHeight_);
        for (int c = 0; c < gridWidth_; ++c)
        {
            int x0          = c * cellSizePx_;
            int x1          = std::min(x0 + cellSizePx_, frameWidth_);
            bool anyNonRoad = false;

            // scan the pixel block (y0..y1-1, x0..x1-1)
            for (int yy = y0; yy < y1 && !anyNonRoad; ++yy)
            {
                for (int xx = x0; xx < x1; ++xx)
                {
                    cv::Vec3b pixel = resizedMask.at<cv::Vec3b>(yy, xx);

                    // Check if this is a non-road pixel
                    if (!(pixel == cv::Vec3b(128, 64, 128)) &&
                        !(pixel == cv::Vec3b(0, 0, 0)))
                    {
                        anyNonRoad = true;
                        break;
                    }
                }
            }

            // If any non-road pixel was found, mark cell as occupied
            occupancy_[gridIndex(r, c)] = anyNonRoad;
        }
    }
}

bool ObstacleAvoidance::detectAllCollisions()
{
    needBypass_   = false;
    collisionIdx_ = -1;
    collisionRow_ = -1;
    collisionCol_ = -1;
    collisionX_   = -1;
    collisionY_   = -1;

    // Clear previous collision points
    collisionPoints_.clear();
    searchedCollisionPoints_.clear();
    obstaclePoints_.clear();

    // Calculate the starting point of the bottom "ignore zone"
    int ignoreZoneRowThreshold =
        static_cast<int>(frameHeight_ * 4.5 / 6.0) / cellSizePx_;

    bool foundCollision = false;

    for (int r = 0; r < gridHeight_; r++)
    {
        // Skip if in ignore zone
        if (r >= ignoreZoneRowThreshold)
        {
            continue;
        }

        for (int c = 0; c < gridWidth_; c++)
        {
            // If this cell is occupied, it's a collision point
            if (occupancy_[gridIndex(r, c)])
            {
                obstaclePoints_.emplace_back(r, c);

                // If this is the first collision found, set it as the primary
                // one
                if (!foundCollision)
                {
                    collisionRow_ = r;
                    collisionCol_ = c;
                    gridToPixel(r, c, collisionX_, collisionY_);
                    needBypass_    = true;
                    foundCollision = true;
                }
            }
        }
    }

    // 1. Use all obstacle points as collision points
    collisionPoints_ = obstaclePoints_;

    // 2. Find trajectory cells that are affected 
    // for (const auto& trajCell : trajectoryCells_) {
    //     int r = trajCell.first;
    //     int c = trajCell.second;
    //     if (r < ignoreZoneRowThreshold && occupancy_[gridIndex(r, c)]) {
    //         collisionPoints_.emplace_back(r, c);
    //     }
    // }

    // Sort collision points by distance from bottom (closest to car first)
    std::sort(collisionPoints_.begin(), collisionPoints_.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    return foundCollision;
}

std::vector<cv::Point> ObstacleAvoidance::adjustTrajectory(
    const std::vector<cv::Point>& originalTrajectory)
{
    if (collisionPoints_.empty() || originalTrajectory.empty())
    {
        return originalTrajectory;
    }

    std::vector<cv::Point> adjustedTrajectory = originalTrajectory;

    // Convert proximity radius from grid cells to pixels
    safeDistancePx_       = proximityRadius_ * cellSizePx_;
    int safeDistanceCells = proximityRadius_ + 1; // Add safety margin

    // First, group trajectory points by row for consistent adjustment within
    // rows
    std::map<int, std::vector<std::pair<int, std::vector<size_t>>>>
        rowToColumnPoints;

    for (size_t i = 0; i < originalTrajectory.size(); i++)
    {
        int r, c;
        if (pixelToGrid(originalTrajectory[i].x, originalTrajectory[i].y, r, c))
        {
            // Insert into row-based map
            bool found = false;
            for (auto& colPoints : rowToColumnPoints[r])
            {
                if (colPoints.first == c)
                {
                    colPoints.second.push_back(i);
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                rowToColumnPoints[r].push_back({c, {i}});
            }
        }
    }

    // Process each row separately to ensure consistent avoidance behavior
    for (auto& rowEntry : rowToColumnPoints)
    {
        int r = rowEntry.first; // Current row

        // Find all obstacles in this row
        std::vector<int> obstacleColumns;
        for (int col = 0; col < gridWidth_; col++)
        {
            if (occupancy_[gridIndex(r, col)])
            {
                obstacleColumns.push_back(col);
            }
        }

        // Skip if no obstacles in this row
        if (obstacleColumns.empty())
        {
            continue;
        }

        std::cout << "Row " << r << " has " << obstacleColumns.size()
                  << " obstacles" << std::endl;

        // Process each trajectory column in this row
        for (auto& colEntry : rowEntry.second)
        {
            int c = colEntry.first; // Trajectory column
            const auto& pointIndices =
                colEntry.second; // Indices of points at this position

            // Find obstacles that are close to this trajectory column
            std::vector<int> nearbyObstacleColumns;
            int searchRange =
                safeDistanceCells * 2; // Look ahead by twice the safe distance

            // Find obstacles within relevant range
            for (int obsCol : obstacleColumns)
            {
                if (std::abs(obsCol - c) <= searchRange)
                {
                    nearbyObstacleColumns.push_back(obsCol);
                }
            }

            if (nearbyObstacleColumns.empty())
            {
                continue;
            }

            // Find the left and right edges of the obstacle cluster in front of
            // the trajectory
            int leftEdge  = gridWidth_;
            int rightEdge = -1;

            // First, find the general obstacle area
            for (int obsCol : nearbyObstacleColumns)
            {
                // Obstacles to the right of trajectory that are close
                if (obsCol >= c && obsCol <= c + searchRange)
                {
                    leftEdge  = std::min(leftEdge, obsCol);
                    rightEdge = std::max(rightEdge, obsCol);
                }
                // Obstacles to the left that are close
                else if (obsCol < c && obsCol >= c - searchRange)
                {
                    leftEdge  = std::min(leftEdge, obsCol);
                    rightEdge = std::max(rightEdge, obsCol);
                }
            }

            // Skip if no relevant obstacles found
            if (leftEdge > rightEdge)
            {
                continue;
            }

            std::cout << "Row " << r << ", Traj col " << c
                      << ", Obstacle left edge: " << leftEdge
                      << ", Obstacle right edge: " << rightEdge << std::endl;

            // Calculate free space to the left of obstacle cluster
            int leftFreeSpace = 0;
            for (int col = leftEdge - 1; col >= 0; col--)
            {
                if (!occupancy_[gridIndex(r, col)])
                {
                    leftFreeSpace++;
                }
                else
                {
                    break; // Stop at first obstacle
                }
            }

            // Calculate free space to the right of obstacle cluster
            int rightFreeSpace = 0;
            for (int col = rightEdge + 1; col < gridWidth_; col++)
            {
                if (!occupancy_[gridIndex(r, col)])
                {
                    rightFreeSpace++;
                }
                else
                {
                    break; // Stop at first obstacle
                }
            }

            std::cout << "  Left space: " << leftFreeSpace
                      << ", Right space: " << rightFreeSpace << std::endl;

            // Determine which side to move to
            bool moveLeft = false;

            // If trajectory is inside obstacle region, choose side with more
            // space
            if (c >= leftEdge && c <= rightEdge)
            {
                // We're inside the obstacle - check which side has more space
                if (leftEdge <= 0)
                {
                    // Left edge is against wall, must move right
                    moveLeft = false;
                }
                else if (rightEdge >= gridWidth_ - 1)
                {
                    // Right edge is against wall, must move left
                    moveLeft = true;
                }
                else
                {
                    // Choose side with more free space
                    moveLeft = (leftFreeSpace >= rightFreeSpace);
                }
            }
            else
            {
                // We're outside the obstacle region
                // If trajectory is to the left of obstacle, stay left
                // If trajectory is to the right of obstacle, stay right
                moveLeft = (c < leftEdge);
            }

            // Calculate new column position
            int newCol;
            if (moveLeft)
            {
                // Move to left of obstacle with safe margin
                newCol = std::max(0, leftEdge - safeDistanceCells);
            }
            else
            {
                // Move to right of obstacle with safe margin
                newCol =
                    std::min(gridWidth_ - 1, rightEdge + safeDistanceCells);
            }

            // Find closest obstacle column to original position
            int closestObstacleCol = -1;
            int minDistance        = gridWidth_;
            for (int obsCol : nearbyObstacleColumns)
            {
                int distance = std::abs(c - obsCol);
                if (distance < minDistance)
                {
                    minDistance        = distance;
                    closestObstacleCol = obsCol;
                }
            }
            // Check if new position is occupied by an obstacle - if so, don't
            // update trajectory at all
            bool newPositionOccupied = occupancy_[gridIndex(r, newCol)];

            if (newPositionOccupied)
            {
                std::cout << "  Warning: New position is on an obstacle! No "
                             "trajectory update."
                          << std::endl;
                return originalTrajectory; // Skip this trajectory adjustment
                                           // entirely
            }

            // Additional check: if we have nearby obstacles, ensure new
            // position is actually better
            if (closestObstacleCol != -1)
            {
                // Distance from original position to closest obstacle
                int originalDistance = std::abs(c - closestObstacleCol);

                // Distance from new position to closest obstacle
                int newDistance = std::abs(newCol - closestObstacleCol);

                // If new position is closer to obstacle, don't update
                if (newDistance < originalDistance)
                {
                    std::cout << "  Warning: New position is closer to "
                                 "obstacle! No trajectory update."
                              << std::endl;
                    continue; // Skip this trajectory adjustment entirely
                }
            }

            // Convert new position back to pixel coordinates
            int newX, newY;
            gridToPixel(r, newCol, newX, newY);

            // Apply the adjustment to all trajectory points in this grid cell
            for (size_t idx : pointIndices)
            {
                // Keep the original y-coordinate for smooth vertical movement
                adjustedTrajectory[idx].x = newX;

                std::cout << "  Adjusted trajectory point " << idx
                          << " from col " << c << " to col " << newCol
                          << " (X: " << originalTrajectory[idx].x << " -> "
                          << newX << ")" << std::endl;
            }
        }
    }

    std::cout << "Adjusted trajectory with " << adjustedTrajectory.size()
              << " points" << std::endl;

    // // Apply a smoothing filter to prevent jerky movements
    smoothTrajectory(adjustedTrajectory);

    return adjustedTrajectory;
}

// Add this method to apply smoothing to the trajectory
void ObstacleAvoidance::smoothTrajectory(std::vector<cv::Point>& trajectory)
{
    if (trajectory.size() < 6)
        return; // Need sufficient points

    std::vector<cv::Point> smoothed = trajectory;

    // Find first and last adjusted points
    int firstAdjustedIdx = -1;
    int lastAdjustedIdx  = -1;

    // Sort trajectory by Y (bottom to top)
    std::vector<size_t> indices(trajectory.size());
    for (size_t i = 0; i < indices.size(); i++)
    {
        indices[i] = i;
    }

    std::sort(indices.begin(), indices.end(), [&trajectory](size_t a, size_t b)
              { return trajectory[a].y > trajectory[b].y; });

    // Find first significant lateral change (possible obstacle avoidance)
    for (size_t i = 1; i < indices.size(); i++)
    {
        int idx     = indices[i];
        int prevIdx = indices[i - 1];

        if (std::abs(trajectory[idx].x - trajectory[prevIdx].x) > cellSizePx_)
        {
            firstAdjustedIdx = std::min(prevIdx, idx);
            break;
        }
    }

    if (firstAdjustedIdx < 0)
        return; // No adjustment found

    // Look ahead 3-5 cells from first adjustment to find the avoidance zone
    int lookAhead = std::min(
        20, static_cast<int>(trajectory.size() - firstAdjustedIdx - 1));
    lastAdjustedIdx = firstAdjustedIdx + lookAhead;

    // Create pre-transition point (start curving earlier)
    int preTransitionIdx = std::max(0, firstAdjustedIdx - 5);

    // Create post-transition point
    int postTransitionIdx =
        std::min(static_cast<int>(trajectory.size()) - 1, lastAdjustedIdx + 5);

    // Apply cubic interpolation between these points
    for (int i = preTransitionIdx; i <= postTransitionIdx; i++)
    {
        double t = static_cast<double>(i - preTransitionIdx) /
                   (postTransitionIdx - preTransitionIdx);

        // Cubic interpolation factor (slow start, fast middle, slow end)
        double factor = t * t * (3 - 2 * t); // Cubic Hermite spline

        int startX = trajectory[preTransitionIdx].x;
        int endX   = trajectory[postTransitionIdx].x;

        // Apply smooth transition
        smoothed[i].x = startX + static_cast<int>(factor * (endX - startX));
    }

    // Apply additional moving average smoothing for extra smoothness
    const int windowSize = 5;

    for (size_t i = windowSize / 2; i < trajectory.size() - windowSize / 2; i++)
    {
        int sumX = 0;

        for (int j = -windowSize / 2; j <= windowSize / 2; j++)
        {
            sumX += smoothed[i + j].x;
        }

        trajectory[i].x = sumX / windowSize;
    }
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

void ObstacleAvoidance::gridToPixel(int gridR, int gridC, int& outPx,
                                    int& outPy) const
{
    outPx = gridC * cellSizePx_ + cellSizePx_ / 2;
    outPy = gridR * cellSizePx_ + cellSizePx_ / 2;
}

void ObstacleAvoidance::buildTrajectoryGrid(
    const std::vector<cv::Point>& trajectory)
{
    // Clear previous trajectory data
    trajectoryCells_.clear();
    trajectoryCellMap_.clear();

    if (!trajectory.empty())
    {
        // Reserve space to avoid reallocation
        trajectoryCells_.reserve(trajectory.size());

        for (const auto& p : trajectory)
        {
            int gr, gc;
            if (pixelToGrid(p.x, p.y, gr, gc))
            {
                // Add to list if not already there
                int cellIdx = gridIndex(gr, gc);
                if (trajectoryCellMap_.find(cellIdx) ==
                    trajectoryCellMap_.end())
                {
                    trajectoryCells_.emplace_back(gr, gc);
                    trajectoryCellMap_[cellIdx] = true;
                }
            }
        }

        // Sort cells by row for consistency
        std::sort(trajectoryCells_.begin(), trajectoryCells_.end());
    }

    std::cout << "Built trajectory with " << trajectoryCells_.size() << " cells"
              << std::endl;
}

void ObstacleAvoidance::visualizeGrid(
    const std::vector<cv::Point>* adjustedTrajectory, cv::Mat& outputImage)
{
    cv::Size actualSize(frameWidth_, frameHeight_);

    // Create overlay at the target size directly
    cv::Mat overlay = cv::Mat::zeros(actualSize, CV_8UC3);
    // Draw each occupied cell
    // for (int r = 0; r < gridHeight_; ++r) {
    //     for (int c = 0; c < gridWidth_; ++c) {

    //         int x0 = static_cast<int>(c * cellSizePx_);
    //         int y0 = static_cast<int>(r * cellSizePx_);
    //         int x1 = static_cast<int>(std::min((c+1) * cellSizePx_,
    //         frameWidth_)); int y1 = static_cast<int>(std::min((r+1) *
    //         cellSizePx_, frameHeight_));

    //         if (trajectoryGrid_[r][c]) {
    //             cv::rectangle(overlay, cv::Point(x0, y0), cv::Point(x1, y1),
    //                       cv::Scalar(255, 255, 0), -1); // Purple fill for
    //                       trajectory cells
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

    // Draw trajectory cells
    for (const auto& trajCell : trajectoryCells_)
    {
        int r = trajCell.first;
        int c = trajCell.second;

        int x0 = static_cast<int>(c * cellSizePx_);
        int y0 = static_cast<int>(r * cellSizePx_);
        int x1 = static_cast<int>(std::min((c + 1) * cellSizePx_, frameWidth_));
        int y1 =
            static_cast<int>(std::min((r + 1) * cellSizePx_, frameHeight_));

        cv::rectangle(overlay, cv::Point(x0, y0), cv::Point(x1, y1),
                      cv::Scalar(255, 255, 0),
                      -1); // Yellow fill for trajectory cells
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
        cv::putText(overlay,
                    "Obstacles Detected: " +
                        std::to_string(obstaclePoints_.size()),
                    cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 0, 255), 2);

        // Visualize all collision points
        for (size_t i = 0; i < obstaclePoints_.size(); i++)
        {
            int r = obstaclePoints_[i].first;
            int c = obstaclePoints_[i].second;

            // Get pixel coordinates for collision cell
            int collX0 = static_cast<int>(c * cellSizePx_);
            int collY0 = static_cast<int>(r * cellSizePx_);
            int collX1 =
                static_cast<int>(std::min((c + 1) * cellSizePx_, frameWidth_));
            int collY1 =
                static_cast<int>(std::min((r + 1) * cellSizePx_, frameHeight_));

            // Fill collision cell with bright color and add border
            cv::rectangle(overlay, cv::Point(collX0, collY0),
                          cv::Point(collX1, collY1), cv::Scalar(255, 0, 255),
                          -1); // Fill with bright magenta

            // Add white border for contrast
            cv::rectangle(overlay, cv::Point(collX0, collY0),
                          cv::Point(collX1, collY1), cv::Scalar(255, 255, 255),
                          2); // White border
        }
        // cv::putText(overlay, "Obstacle Detected", cv::Point(20, 40),
        //             cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }
    else
    {
        // cv::putText(overlay, "No Obstacle Detected", cv::Point(20, 40),
        //             cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }

    if (adjustedTrajectory && !adjustedTrajectory->empty())
    {
        cv::putText(overlay, "Adjusted Trajectory", cv::Point(20, 120),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255),
                    2);

        // Connect points with lines
        for (size_t i = 0; i < adjustedTrajectory->size() - 1; i++)
        {
            cv::line(overlay, (*adjustedTrajectory)[i],
                     (*adjustedTrajectory)[i + 1], cv::Scalar(0, 255, 255),
                     2); // Cyan line
        }

        // Draw points
        for (const auto& p : *adjustedTrajectory)
        {
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

    // Draw trajectory cells
    // for (const auto& cell : trajectoryCells_) {
    //     int r = cell.first;
    //     int c = cell.second;

    //     int x0 = static_cast<int>(c * cellSizePx_);
    //     int y0 = static_cast<int>(r * cellSizePx_);
    //     int x1 = static_cast<int>(std::min((c+1) * cellSizePx_,
    //     frameWidth_)); int y1 = static_cast<int>(std::min((r+1) *
    //     cellSizePx_, frameHeight_));

    //     cv::rectangle(overlay, cv::Point(x0, y0), cv::Point(x1, y1),
    //                 cv::Scalar(255, 255, 0), -1); // Yellow fill for
    //                 trajectory cells
    // }

    overlay.copyTo(outputImage);
}
