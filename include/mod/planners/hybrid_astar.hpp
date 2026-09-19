/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
 *   This file is part of mod.
 *
 *   mod is free software: you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public License as
 *   published by the Free Software Foundation, either version 3 of the License,
 *   or (at your option) any later version.
 *
 *   mod is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with mod.  If not, see
 *   <https://www.gnu.org/licenses/>.
 *
 * The search design (cell x heading-bin duplicate detection over exact continuous poses, three minimum-radius
 * primitives of length cell * sqrt(2), Nav2's analytic-expansion schedule, h = max(grid cost-to-go, kinematic
 * distance)) follows the description of Nav2's SmacPlannerHybrid (Apache-2.0, Steve Macenski et al.). No Nav2
 * code is included.
 */

#pragma once

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mod/grid_dijkstra.hpp>
#include <mod/parameters.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace MoD {

/**
 * Hybrid A* over the same OMPL SpaceInformation (state space, bounds, validity checker and collision resolution)
 * and OptimizationObjective as the sampling planners, so its solution cost is directly comparable in the run
 * logs. Own implementation; not an ompl::base::Planner.
 *
 * - Nodes keep their exact continuous pose; the (cell, heading bin[, direction]) key is only for duplicate
 *   detection.
 * - Primitives: straight, left and right arcs at the minimum turning radius, of length `cell * sqrt(2)`
 *   (parameter). With `allow_reverse` the three reverse counterparts are added. Validity by `si->checkMotion`,
 *   g-cost by `objective->motionCost` (the Dubins / Reeds-Shepp interpolation between the two endpoints of a
 *   minimum-radius arc is that arc, so the objective integrates along the primitive).
 * - Heuristic: `h = max(h_grid, h_kin)`; `h_grid` is a goal-rooted, lazily expanded GridDijkstra with the
 *   objective's `motionCost` as edge weight; `h_kin` is `w_d * Dubins distance` (forward-only) or, with reverse
 *   motion, `w_d * min(Dubins, Reeds-Shepp length + change_penalty * cusps)`, memoized per key.
 * - Analytic expansion (Nav2 schedule): a Dubins (or Reeds-Shepp) shot to the goal every
 *   `max(1, floor(h_kin / (analytic_ratio * primitive_length)))` expansions if its length is at most
 *   `analytic_max_length_m`; the first valid shot ends the search.
 * - Goal test: a valid shot, or a node whose cell and bin equal the goal's (then connected to the exact goal if
 *   `checkMotion` allows it).
 * - No reverse penalty ever; one `change_penalty` per direction flip. The first primitive out of the start is
 *   free to choose its direction.
 *
 * One instance per run; shares only const maps through the objective; safe for one thread per run.
 */
class HybridAStar {
 public:
  enum class Direction : uint8_t { forward = 0, reverse = 1 };

  /// One expanded (closed) node, for overlays.
  struct ExpandedNode {
    double x, y, yaw;
    unsigned int bin;
    Direction dir;
  };

  struct Result {
    bool solved{false};
    double cost{std::numeric_limits<double>::infinity()};  ///< g of the solution: objective cost + cusp penalties
    double objective_cost{std::numeric_limits<double>::infinity()};  ///< objective cost only
    size_t cusps{0};
    size_t expansions{0};        ///< nodes popped and expanded
    size_t generated{0};         ///< nodes pushed
    size_t analytic_attempts{0};
    bool solved_by_shot{false};
    double planning_time_s{0.0};
    double heuristic_setup_s{0.0};
    std::string termination;  ///< "solution" | "time" | "cancelled" | "max_expansions" | "exhausted" | "invalid"
  };

  /**
   * @param si            state space (Dubins, Reeds-Shepp or plain SE2), bounds, validity checker, resolution
   * @param objective     the cost of every primitive and shot (`motionCost`); the heuristic grid's edge weight
   * @param params        search parameters; `allow_reverse` is forced to false unless the space is Reeds-Shepp
   * @param turning_radius minimum turning radius of the primitives and the kinematic heuristic [m]
   */
  HybridAStar(ompl::base::SpaceInformationPtr si, ompl::base::OptimizationObjectivePtr objective,
              HybridAStarParameters params, double turning_radius);
  ~HybridAStar();

  HybridAStar(const HybridAStar &) = delete;
  HybridAStar &operator=(const HybridAStar &) = delete;

  /// Runs the search. Returns the path (start, primitive endpoints, [goal]); empty when no solution was found.
  /// `cancel` (optional) is polled together with the wall clock every 256 expansions.
  ompl::geometric::PathGeometric solve(const ompl::base::State *start, const ompl::base::State *goal,
                                       double time_budget_s, const std::function<bool()> &cancel = nullptr);

  const Result &result() const { return result_; }
  const HybridAStarParameters &parameters() const { return params_; }
  double primitiveLength() const { return primitive_length_; }
  bool reverseEnabled() const { return params_.allow_reverse; }

  /// Closed nodes of the last solve, in expansion order.
  std::vector<ExpandedNode> expandedNodes() const;
  /// Motion direction of every segment of the last solution path (size = states - 1).
  const std::vector<Direction> &pathDirections() const { return path_directions_; }

  /// Heuristic terms of a state for the last solve's goal (the grid expands lazily; +inf when unreachable).
  double heuristicGrid(const ompl::base::State *s);
  double heuristicKinematic(const ompl::base::State *s, Direction dir = Direction::forward);
  double heuristic(const ompl::base::State *s, Direction dir = Direction::forward);

  /// Heuristic grid statistics of the last solve.
  const GridDijkstra *heuristicGridDijkstra() const { return grid_.get(); }

 private:
  struct Node {
    double x, y, yaw;
    double g, h;
    size_t parent;
    Direction dir;
    bool closed;
  };
  struct Pose {
    double x, y, yaw;
  };

  static constexpr size_t kNone = std::numeric_limits<size_t>::max();

  // Setup per solve.
  void setupHeuristic(const ompl::base::State *goal);
  double gridEdgeCost(size_t from, size_t to) const;

  // Keys.
  unsigned int binOf(double yaw) const;
  uint64_t keyOf(double x, double y, double yaw, Direction dir) const;
  bool inBounds(double x, double y) const;

  // Primitives.
  Pose primitiveEnd(const Pose &p, double signed_length, double curvature) const;
  void toState(const Pose &p, ompl::base::State *s) const;
  static Pose fromState(const ompl::base::State *s);

  // Heuristic terms.
  double hGrid(double x, double y);
  double hKin(const Pose &p, Direction dir);
  double hKinRaw(const Pose &p, Direction dir) const;
  double hOf(const Pose &p, Direction dir);

  // Analytic expansion. Returns true and fills `cost` (objective) and `cusps` when the shot is valid.
  bool tryShot(size_t node, double &objective_cost, size_t &cusps);
  bool shotGeometry(const Pose &from, Direction dir, double &length, size_t &cusps) const;

  ompl::geometric::PathGeometric extractPath(size_t node, bool append_goal);

  ompl::base::SpaceInformationPtr si_;
  ompl::base::OptimizationObjectivePtr objective_;
  HybridAStarParameters params_;
  double turning_radius_;
  double w_d_{1.0};
  double primitive_length_{0.0};
  double x_min_{0.0}, x_max_{0.0}, y_min_{0.0}, y_max_{0.0};

  std::shared_ptr<ompl::base::DubinsStateSpace> dubins_;
  const ompl::base::ReedsSheppStateSpace *reeds_shepp_{nullptr};  ///< the run's space when it is Reeds-Shepp

  // Scratch states allocated from si_ (the Dubins space shares the SE2 state layout).
  ompl::base::State *sa_{nullptr}, *sb_{nullptr}, *goal_state_{nullptr};

  std::unique_ptr<GridDijkstra> grid_;
  std::unordered_map<uint64_t, double> h_kin_memo_;

  std::vector<Node> nodes_;
  Pose goal_{0.0, 0.0, 0.0};
  uint64_t goal_key_{0};
  Result result_;
  std::vector<Direction> path_directions_;
};

typedef std::shared_ptr<HybridAStar> HybridAStarPtr;

}  // namespace MoD
