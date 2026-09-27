#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <cmath>
#include <plan_env/grid_map.h>
#include <queue>

constexpr double inf = 1 >> 20;
struct GridNode;
typedef GridNode *GridNodePtr;

enum ASTAR_RET
{
	SUCCESS,
	INIT_ERR,
	SEARCH_ERR
};

struct GridNode
{
	enum enum_state
	{
		OPENSET = 1,
		CLOSEDSET = 2,
		UNDEFINED = 3
	};

	int rounds{0}; // Distinguish every call
	enum enum_state state
	{
		UNDEFINED
	};
	Eigen::Vector3i index;

	double gScore{inf}, fScore{inf};
	double distanceScore{inf};
	double secondaryScore{0.0};
	GridNodePtr cameFrom{NULL};
};

class NodeComparator
{
public:
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		if (std::abs(node1->fScore - node2->fScore) <= 1.0e-9)
			return node1->secondaryScore > node2->secondaryScore;
		return node1->fScore > node2->fScore;
	}
};

class AStar
{
public:
	struct VisibilityCostStats
	{
		bool enabled{false};
		int query_count{0};
		int invalid_query_count{0};
		double mean_occupied_fraction{0.0};
		double max_occupied_fraction{0.0};
		double blocked_node_ratio{0.0};
		double equivalent_penalty_m{0.0};
	};

private:
	GridMap::Ptr grid_map_;

	inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
	double getManhHeu(GridNodePtr node1, GridNodePtr node2);
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);

	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

	inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
	inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	inline int checkOccupancy(const Eigen::Vector3d &pos) { return grid_map_->getInflateOccupancy(pos); }

	std::vector<GridNodePtr> retrievePath(GridNodePtr current);
	double visibilityPenalty(const Eigen::Vector3d &node_position,
	                         double path_distance_m, bool &valid) const;
	void updateVisibilityStats(const std::vector<GridNodePtr> &path,
	                           double normalization_length_m);

	double step_size_, inv_step_size_;
	Eigen::Vector3d center_;
	Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	std::vector<GridNodePtr> gridPath_;

	GridNodePtr ***GridNodeMap_;
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

	int rounds_{0};
	bool visibility_cost_enabled_{false};
	double visibility_cost_weight_m_{0.75};
	double visibility_reference_speed_{1.0};
	Eigen::Vector3d visibility_target_position_{Eigen::Vector3d::Zero()};
	Eigen::Vector3d visibility_target_velocity_{Eigen::Vector3d::Zero()};
	VisibilityCostStats visibility_stats_;

public:
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);
	void setVisibilityCostContext(bool enabled,
	                              const Eigen::Vector3d &target_position,
	                              const Eigen::Vector3d &target_velocity,
	                              double reference_speed,
	                              double weight_m);
	void clearVisibilityCostContext();
	VisibilityCostStats getLastVisibilityCostStats() const
	{
		return visibility_stats_;
	}

	ASTAR_RET AstarSearch(
	    const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt,
	    const Eigen::Vector3d *secondary_plane_point = nullptr,
	    const Eigen::Vector3d *secondary_plane_normal = nullptr,
	    bool enforce_secondary_halfspace = false);

	std::vector<Eigen::Vector3d> getPath();
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	return tie_breaker_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const
{
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

	if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
	{
		ROS_ERROR("Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
		return false;
	}

	return true;
};

#endif
