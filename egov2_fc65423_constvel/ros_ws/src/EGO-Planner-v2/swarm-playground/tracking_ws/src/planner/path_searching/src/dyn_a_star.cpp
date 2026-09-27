#include "path_searching/dyn_a_star.h"

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    grid_map_ = occ_map;
}

void AStar::setVisibilityCostContext(
    bool enabled, const Eigen::Vector3d &target_position,
    const Eigen::Vector3d &target_velocity, double reference_speed,
    double weight_m)
{
    visibility_cost_enabled_ = enabled && grid_map_ &&
                               target_position.allFinite() &&
                               target_velocity.allFinite();
    visibility_target_position_ = target_position;
    visibility_target_velocity_ = target_velocity;
    visibility_reference_speed_ = std::max(0.1, reference_speed);
    visibility_cost_weight_m_ = std::max(0.0, weight_m);
}

void AStar::clearVisibilityCostContext()
{
    visibility_cost_enabled_ = false;
}

double AStar::visibilityPenalty(const Eigen::Vector3d &node_position,
                                double path_distance_m, bool &valid) const
{
    valid = false;
    if (!visibility_cost_enabled_ || !grid_map_ || !node_position.allFinite() ||
        !std::isfinite(path_distance_m))
        return 0.0;
    const double node_time = std::max(0.0, path_distance_m) /
                             visibility_reference_speed_;
    const Eigen::Vector3d target = visibility_target_position_ +
                                   visibility_target_velocity_ * node_time;
    double occupied_fraction = 0.0;
    valid = grid_map_->getInflatedLineOccupancyFraction(
        node_position, target, std::max(0.05, grid_map_->getResolution()),
        occupied_fraction);
    return valid ? std::min(1.0, std::max(0.0, occupied_fraction)) : 0.0;
}

void AStar::updateVisibilityStats(const std::vector<GridNodePtr> &path,
                                  double normalization_length_m)
{
    visibility_stats_ = VisibilityCostStats();
    visibility_stats_.enabled = visibility_cost_enabled_;
    if (!visibility_cost_enabled_ || path.empty())
        return;

    double sum_fraction = 0.0;
    int blocked = 0;
    for (GridNodePtr node : path)
    {
        bool valid = false;
        const double fraction = visibilityPenalty(
            Index2Coord(node->index), node->distanceScore * step_size_, valid);
        ++visibility_stats_.query_count;
        if (!valid)
        {
            ++visibility_stats_.invalid_query_count;
            continue;
        }
        sum_fraction += fraction;
        visibility_stats_.max_occupied_fraction = std::max(
            visibility_stats_.max_occupied_fraction, fraction);
        blocked += static_cast<int>(fraction > 1.0e-9);
        if (node->cameFrom != NULL)
        {
            const double edge_length_m =
                (Index2Coord(node->index) -
                 Index2Coord(node->cameFrom->index)).norm();
            visibility_stats_.equivalent_penalty_m +=
                visibility_cost_weight_m_ * fraction * edge_length_m /
                std::max(step_size_, normalization_length_m);
        }
    }
    const int valid_queries = visibility_stats_.query_count -
                              visibility_stats_.invalid_query_count;
    if (valid_queries > 0)
    {
        visibility_stats_.mean_occupied_fraction =
            sum_fraction / static_cast<double>(valid_queries);
        visibility_stats_.blocked_node_ratio =
            static_cast<double>(blocked) / valid_queries;
    }
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    int occ;
    if (checkOccupancy(Index2Coord(start_idx)))
    {
        // ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            // cout << "start_pt=" << start_pt.transpose() << endl;
            if (!Coord2Index(start_pt, start_idx))
            {
                return false;
            }

            // Check the adjusted start cell itself.
            occ = checkOccupancy(Index2Coord(start_idx));
            if (occ == -1)
            {
                ROS_WARN("[Astar] Start point outside the map region.");
                return false;
            }
        } while (occ);
    }

    if (checkOccupancy(Index2Coord(end_idx)))
    {
        // ROS_WARN("End point is insdide an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            // cout << "end_pt=" << end_pt.transpose() << endl;
            if (!Coord2Index(end_pt, end_idx))
            {
                return false;
            }

            // Check the adjusted end cell itself.
            occ = checkOccupancy(Index2Coord(end_idx));
            if (occ == -1)
            {
                ROS_WARN("[Astar] End point outside the map region.");
                return false;
            }
        } while (checkOccupancy(Index2Coord(end_idx)));
    }

    return true;
}

ASTAR_RET AStar::AstarSearch(
    const double step_size, Vector3d start_pt, Vector3d end_pt,
    const Vector3d *secondary_plane_point,
    const Vector3d *secondary_plane_normal,
    bool enforce_secondary_halfspace)
{
    ros::Time time_1 = ros::Time::now();
    ++rounds_;

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;
    visibility_stats_ = VisibilityCostStats();
    visibility_stats_.enabled = visibility_cost_enabled_;
    const double visibility_normalization_length_m =
        std::max(step_size_, (end_pt - start_pt).norm());
    const bool secondary_preference_enabled =
        secondary_plane_point != nullptr && secondary_plane_normal != nullptr &&
        secondary_plane_point->allFinite() && secondary_plane_normal->allFinite() &&
        secondary_plane_normal->head<2>().norm() > 1.0e-6;
    Vector3d secondary_normal = Vector3d::Zero();
    if (secondary_preference_enabled)
        secondary_normal = secondary_plane_normal->normalized();
    const auto secondary_score = [&](const Vector3d &position) {
        // This value is used only after equal A* f-scores.  It cannot make a
        // longer path win or turn the task preference into a feasibility
        // condition.  Positive displacement along the supplied normal is the
        // preferred half-plane, hence its queue score is lower.
        return secondary_preference_enabled
                   ? -secondary_normal.dot(position - *secondary_plane_point)
                   : 0.0;
    };

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        ROS_ERROR("Unable to handle the initial or end point, force return!");
        return ASTAR_RET::INIT_ERR;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    endPtr->index = end_idx;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->distanceScore = 0;
    startPtr->secondaryScore = secondary_score(Index2Coord(start_idx));
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); //put start in open set

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath_ = retrievePath(current);
            updateVisibilityStats(gridPath_, visibility_normalization_length_m);
            return ASTAR_RET::SUCCESS;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    Vector3i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;
                    neighborIdx(1) = (current->index)(1) + dy;
                    neighborIdx(2) = (current->index)(2) + dz;

                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                    {
                        continue;
                    }

                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                    neighborPtr->index = neighborIdx;

                    const Vector3d neighbor_position =
                        Index2Coord(neighborPtr->index);
                    const bool neighbor_is_end = neighborIdx == end_idx;
                    if (secondary_preference_enabled &&
                        enforce_secondary_halfspace && !neighbor_is_end &&
                        secondary_normal.dot(neighbor_position -
                                             *secondary_plane_point) <
                            -0.5 * step_size_)
                    {
                        continue;
                    }

                    bool flag_explored = neighborPtr->rounds == rounds_;

                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    {
                        continue; //in closed set.
                    }

                    neighborPtr->rounds = rounds_;

                    if (checkOccupancy(neighbor_position))
                    {
                        continue;
                    }

                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    const double tentative_distance_score =
                        current->distanceScore + static_cost;
                    const double current_outward_steps =
                        secondary_preference_enabled
                            ? std::max(0.0, -secondary_normal.dot(
                                  Index2Coord(current->index) -
                                  *secondary_plane_point)) /
                                  step_size_
                            : 0.0;
                    const double neighbor_outward_steps =
                        secondary_preference_enabled
                            ? std::max(0.0, -secondary_normal.dot(
                                  Index2Coord(neighborPtr->index) -
                                  *secondary_plane_point)) /
                                  step_size_
                            : 0.0;
                    // A target-facing repair uses ordinary geometric path
                    // length plus only the newly accumulated displacement on
                    // the opposite side of the direct chord.  The coefficient
                    // is one because both terms are measured in grid steps;
                    // there is no tunable visibility weight.  The cost is
                    // one-sided and non-negative, so an outer path remains
                    // available when it is genuinely necessary, while a
                    // marginally shorter outer detour no longer wins merely
                    // because of neighbor expansion order.
                    const double outward_detour_cost =
                        std::max(0.0,
                                 neighbor_outward_steps - current_outward_steps);
                    bool visibility_query_valid = false;
                    const double visibility_fraction = visibilityPenalty(
                        neighbor_position,
                        tentative_distance_score * step_size_,
                        visibility_query_valid);
                    // gScore is measured in grid steps.  Normalizing each
                    // edge by the direct repair length makes a fully blocked
                    // path add approximately visibility_cost_weight_m_ of
                    // physical path-equivalent cost, independent of node
                    // count.  Invalid LOS data is a zero soft cost, never a
                    // hard rejection.
                    const double visibility_cost =
                        visibility_query_valid && visibility_cost_enabled_
                            ? (visibility_cost_weight_m_ / step_size_) *
                                  visibility_fraction *
                                  (static_cost * step_size_) /
                                  visibility_normalization_length_m
                            : 0.0;
                    tentative_gScore = current->gScore + static_cost +
                                       outward_detour_cost + visibility_cost;

                    if (!flag_explored)
                    {
                        //discover a new node
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->distanceScore = tentative_distance_score;
                        neighborPtr->secondaryScore = secondary_score(
                            Index2Coord(neighborPtr->index));
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr); //put neighbor in open set and record it.
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    { //in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->distanceScore = tentative_distance_score;
                        neighborPtr->secondaryScore = secondary_score(
                            Index2Coord(neighborPtr->index));
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    }
                }
        ros::Time time_2 = ros::Time::now();
        if ((time_2 - time_1).toSec() > 0.2)
        {
            ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return ASTAR_RET::SEARCH_ERR;
        }
    }

    ros::Time time_2 = ros::Time::now();

    if ((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

    return ASTAR_RET::SEARCH_ERR;
}

vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}
