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
 * Search design after the description of Nav2's SmacPlannerHybrid (Apache-2.0, Steve Macenski et al.); no Nav2
 * code is included. See the header.
 */

#include <mod/planners/hybrid_astar.hpp>

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mod/log.hpp>
#include <ompl/mod/objectives/MoDOptimizationObjective.h>
#include <queue>
#include <stdexcept>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace MoD {

namespace {
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kSegmentEps = 1e-9;

double wrapAngle(double a) {
  a = std::fmod(a, kTwoPi);
  if (a < 0.0) a += kTwoPi;
  if (a >= kTwoPi) a -= kTwoPi;
  return a;
}
}  // namespace

HybridAStar::HybridAStar(ob::SpaceInformationPtr si, ob::OptimizationObjectivePtr objective,
                         HybridAStarParameters params, double turning_radius)
    : si_(std::move(si)), objective_(std::move(objective)), params_(params), turning_radius_(turning_radius) {
  if (params_.cell_size_m <= 0.0) throw std::invalid_argument("HybridAStar: cell_size_m must be > 0");
  if (params_.angle_bins == 0) throw std::invalid_argument("HybridAStar: angle_bins must be > 0");
  if (turning_radius_ <= 0.0) throw std::invalid_argument("HybridAStar: turning radius must be > 0");
  primitive_length_ = params_.primitive_length_m > 0.0 ? params_.primitive_length_m : params_.cell_size_m * M_SQRT2;

  const auto space = si_->getStateSpace();
  reeds_shepp_ = dynamic_cast<const ob::ReedsSheppStateSpace *>(space.get());
  if (!reeds_shepp_) params_.allow_reverse = false;  // reverse motion needs Reeds-Shepp; Dubins is forward-only
  const ob::RealVectorBounds bounds = space->as<ob::SE2StateSpace>()->getBounds();
  x_min_ = bounds.low[0];
  x_max_ = bounds.high[0];
  y_min_ = bounds.low[1];
  y_max_ = bounds.high[1];

  if (const auto *mod = dynamic_cast<const ompl::MoD::MoDOptimizationObjective *>(objective_.get()))
    w_d_ = mod->getParameters().w_d;

  dubins_ = std::make_shared<ob::DubinsStateSpace>(turning_radius_);
  sa_ = si_->allocState();
  sb_ = si_->allocState();
  goal_state_ = si_->allocState();
}

HybridAStar::~HybridAStar() {
  si_->freeState(sa_);
  si_->freeState(sb_);
  si_->freeState(goal_state_);
}

// ---------------------------------------------------------------------------------------------------------------
// Keys and poses

bool HybridAStar::inBounds(double x, double y) const {
  return x >= x_min_ && x <= x_max_ && y >= y_min_ && y <= y_max_;
}

unsigned int HybridAStar::binOf(double yaw) const {
  const double step = kTwoPi / static_cast<double>(params_.angle_bins);
  const auto bin = static_cast<unsigned int>(std::lround(wrapAngle(yaw) / step));
  return bin % params_.angle_bins;
}

uint64_t HybridAStar::keyOf(double x, double y, double yaw, Direction dir) const {
  const auto col = static_cast<uint64_t>(std::max(0.0, std::floor((x - x_min_) / params_.cell_size_m)));
  const auto row = static_cast<uint64_t>(std::max(0.0, std::floor((y - y_min_) / params_.cell_size_m)));
  const uint64_t bin = binOf(yaw);
  const uint64_t d = params_.allow_reverse && dir == Direction::reverse ? 1u : 0u;
  return (col & 0x3FFFFFu) | ((row & 0x3FFFFFu) << 22) | ((bin & 0xFFFFu) << 44) | (d << 60);
}

HybridAStar::Pose HybridAStar::fromState(const ob::State *s) {
  const auto *se2 = s->as<ob::SE2StateSpace::StateType>();
  return {se2->getX(), se2->getY(), se2->getYaw()};
}

void HybridAStar::toState(const Pose &p, ob::State *s) const {
  auto *se2 = s->as<ob::SE2StateSpace::StateType>();
  se2->setX(p.x);
  se2->setY(p.y);
  double yaw = std::atan2(std::sin(p.yaw), std::cos(p.yaw));
  if (yaw >= M_PI) yaw = -M_PI;  // OMPL's SO(2) bounds are [-pi, pi)
  se2->setYaw(yaw);
}

/// Endpoint of a constant-curvature motion of signed length `s` (negative = reverse) and curvature `k`.
HybridAStar::Pose HybridAStar::primitiveEnd(const Pose &p, double s, double k) const {
  if (k == 0.0) return {p.x + s * std::cos(p.yaw), p.y + s * std::sin(p.yaw), p.yaw};
  const double yaw = p.yaw + s * k;
  return {p.x + (std::sin(yaw) - std::sin(p.yaw)) / k, p.y - (std::cos(yaw) - std::cos(p.yaw)) / k, yaw};
}

// ---------------------------------------------------------------------------------------------------------------
// Heuristic

double HybridAStar::gridEdgeCost(size_t from, size_t to) const {
  const double xi = grid_->x(from), yi = grid_->y(from);
  const double xf = grid_->x(to), yf = grid_->y(to);
  const double heading = std::atan2(yf - yi, xf - xi);
  toState({xi, yi, heading}, sa_);
  toState({xf, yf, heading}, sb_);
  return objective_->motionCost(sa_, sb_).value();
}

void HybridAStar::setupHeuristic(const ob::State *goal) {
  grid_ = std::make_unique<GridDijkstra>(x_min_, x_max_, y_min_, y_max_, params_.cell_size_m);
  const auto checker = si_->getStateValidityChecker();
  ob::State *probe = si_->allocState();
  grid_->computeValidity([&](double x, double y) {
    toState({x, y, 0.0}, probe);
    return checker->isValid(probe);
  });
  si_->freeState(probe);
  grid_->setWeight([this](size_t from, size_t to) { return gridEdgeCost(from, to); });
  const Pose g = fromState(goal);
  const size_t goal_node = grid_->nodeAt(g.x, g.y);
  grid_->setValid(goal_node, true);  // the goal pose is valid even if its cell centre is not
  grid_->setRoot(goal_node, GridDijkstra::Mode::reverse);
  h_kin_memo_.clear();
}

double HybridAStar::hGrid(double x, double y) {
  if (!grid_ || !inBounds(x, y)) return kInf;
  return grid_->costTo(grid_->nodeAt(x, y));
}

/// Cusps of a Reeds-Shepp path when entered with motion direction `dir`; false if it needs reverse motion while
/// reverse is disabled. `length` in metres.
bool HybridAStar::shotGeometry(const Pose &from, Direction dir, double &length, size_t &cusps) const {
  cusps = 0;
  toState(from, sa_);
  if (!reeds_shepp_) {
    length = dubins_->distance(sa_, goal_state_);
    if (dir == Direction::reverse) cusps = 1;  // the Dubins path is driven forward
    return true;
  }
  const auto path = reeds_shepp_->getPath(sa_, goal_state_);
  length = turning_radius_ * path.length();
  int current = dir == Direction::reverse ? -1 : 1;
  for (double l : path.length_) {
    if (std::abs(l) < kSegmentEps) continue;
    const int sign = l < 0.0 ? -1 : 1;
    if (sign < 0 && !params_.allow_reverse) return false;
    if (sign != current) ++cusps;
    current = sign;
  }
  return true;
}

double HybridAStar::hKinRaw(const Pose &p, Direction dir) const {
  toState(p, sa_);
  const double dubins = dubins_->distance(sa_, goal_state_);
  if (!params_.allow_reverse) return w_d_ * dubins;
  // Reverse allowed: the cheapest of driving the Dubins path forward (a cusp if arriving in reverse) and the
  // Reeds-Shepp path with its cusps (relative to the arriving direction) charged at change_penalty each.
  double rs_length = 0.0;
  size_t cusps = 0;
  shotGeometry(p, dir, rs_length, cusps);
  const double forward = w_d_ * dubins + (dir == Direction::reverse ? params_.change_penalty : 0.0);
  const double rs = w_d_ * rs_length + params_.change_penalty * static_cast<double>(cusps);
  return std::min(forward, rs);
}

double HybridAStar::hKin(const Pose &p, Direction dir) {
  const uint64_t key = keyOf(p.x, p.y, p.yaw, dir);
  const auto it = h_kin_memo_.find(key);
  if (it != h_kin_memo_.end()) return it->second;
  const double h = hKinRaw(p, dir);
  h_kin_memo_.emplace(key, h);
  return h;
}

double HybridAStar::hOf(const Pose &p, Direction dir) {
  const double hg = hGrid(p.x, p.y);
  const double hk = hKin(p, dir);
  // A pose whose cell centre is invalid or unreachable on the grid still gets the kinematic term.
  return std::max(std::isfinite(hg) ? hg : 0.0, hk);
}

double HybridAStar::heuristicGrid(const ob::State *s) {
  const Pose p = fromState(s);
  return hGrid(p.x, p.y);
}
double HybridAStar::heuristicKinematic(const ob::State *s, Direction dir) { return hKin(fromState(s), dir); }
double HybridAStar::heuristic(const ob::State *s, Direction dir) { return hOf(fromState(s), dir); }

// ---------------------------------------------------------------------------------------------------------------
// Analytic expansion

bool HybridAStar::tryShot(size_t node, double &objective_cost, size_t &cusps) {
  const Node &n = nodes_[node];
  const Pose p{n.x, n.y, n.yaw};
  double length = 0.0;
  if (!shotGeometry(p, n.dir, length, cusps)) return false;
  if (length > params_.analytic_max_length_m) return false;
  toState(p, sa_);
  if (!si_->checkMotion(sa_, goal_state_)) return false;
  objective_cost = objective_->motionCost(sa_, goal_state_).value();
  return true;
}

// ---------------------------------------------------------------------------------------------------------------
// Path

og::PathGeometric HybridAStar::extractPath(size_t node, bool append_goal) {
  std::vector<size_t> chain;
  for (size_t n = node; n != kNone; n = nodes_[n].parent) chain.push_back(n);
  std::reverse(chain.begin(), chain.end());

  og::PathGeometric path(si_);
  path_directions_.clear();
  ob::ScopedState<> s(si_->getStateSpace());
  for (size_t i = 0; i < chain.size(); ++i) {
    const Node &n = nodes_[chain[i]];
    toState({n.x, n.y, n.yaw}, s.get());
    path.append(s.get());
    if (i > 0) path_directions_.push_back(n.dir);
  }
  if (append_goal) {
    path.append(goal_state_);
    // Direction of the shot's first segment.
    Direction dir = Direction::forward;
    if (reeds_shepp_) {
      const Node &last = nodes_[chain.back()];
      toState({last.x, last.y, last.yaw}, sa_);
      const auto rs = reeds_shepp_->getPath(sa_, goal_state_);
      for (double l : rs.length_) {
        if (std::abs(l) < kSegmentEps) continue;
        dir = l < 0.0 ? Direction::reverse : Direction::forward;
        break;
      }
    }
    path_directions_.push_back(dir);
  }
  return path;
}

std::vector<HybridAStar::ExpandedNode> HybridAStar::expandedNodes() const {
  std::vector<ExpandedNode> out;
  out.reserve(result_.expansions);
  for (const Node &n : nodes_)
    if (n.closed) out.push_back({n.x, n.y, n.yaw, binOf(n.yaw), n.dir});
  return out;
}

// ---------------------------------------------------------------------------------------------------------------
// Search

og::PathGeometric HybridAStar::solve(const ob::State *start, const ob::State *goal, double time_budget_s,
                                     const std::function<bool()> &cancel) {
  typedef std::pair<double, size_t> OpenItem;
  const auto t0 = std::chrono::steady_clock::now();
  const auto elapsed = [&]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); };

  result_ = Result{};
  nodes_.clear();
  path_directions_.clear();
  og::PathGeometric path(si_);

  if (!si_->isValid(start) || !si_->isValid(goal)) {
    result_.termination = "invalid";
    result_.planning_time_s = elapsed();
    MOD_LOG("HybridAStar: %s is not valid", si_->isValid(start) ? "goal" : "start");
    return path;
  }

  si_->copyState(goal_state_, goal);
  goal_ = fromState(goal);
  goal_key_ = keyOf(goal_.x, goal_.y, goal_.yaw, Direction::forward);
  setupHeuristic(goal);
  result_.heuristic_setup_s = elapsed();

  // Primitive set: (signed length, curvature).
  const double kappa = 1.0 / turning_radius_;
  std::vector<std::pair<double, double>> primitives = {
      {primitive_length_, 0.0}, {primitive_length_, kappa}, {primitive_length_, -kappa}};
  if (params_.allow_reverse) {
    primitives.push_back({-primitive_length_, 0.0});
    primitives.push_back({-primitive_length_, kappa});
    primitives.push_back({-primitive_length_, -kappa});
  }

  std::unordered_map<uint64_t, size_t> key_to_node;
  std::priority_queue<OpenItem, std::vector<OpenItem>, std::greater<OpenItem>> open;

  const Pose s0 = fromState(start);
  nodes_.push_back({s0.x, s0.y, s0.yaw, 0.0, hOf(s0, Direction::forward), kNone, Direction::forward, false});
  key_to_node.emplace(keyOf(s0.x, s0.y, s0.yaw, Direction::forward), 0);
  open.emplace(nodes_[0].h, 0);
  result_.generated = 1;

  size_t since_shot = 0;
  size_t solution_node = kNone;
  bool solution_appends_goal = false;
  double solution_cost = kInf, solution_objective_cost = kInf;
  size_t solution_cusps = 0;

  while (!open.empty()) {
    const OpenItem top = open.top();
    open.pop();
    const size_t idx = top.second;
    if (nodes_[idx].closed || top.first != nodes_[idx].g + nodes_[idx].h) continue;  // lazy deletion
    nodes_[idx].closed = true;
    ++result_.expansions;

    if ((result_.expansions & 255u) == 1u) {
      if (elapsed() >= time_budget_s) {
        result_.termination = "time";
        break;
      }
      if (cancel && cancel()) {
        result_.termination = "cancelled";
        break;
      }
    }
    if (result_.expansions > params_.max_expansions) {
      result_.termination = "max_expansions";
      break;
    }

    const Node node = nodes_[idx];  // copy: nodes_ may reallocate below
    const Pose p{node.x, node.y, node.yaw};

    // Cusps along the parent chain (for the result record).
    auto chainCusps = [&](size_t n) {
      size_t c = 0;
      for (; n != kNone && nodes_[n].parent != kNone; n = nodes_[n].parent) {
        const Node &child = nodes_[n];
        const Node &parent = nodes_[child.parent];
        if (parent.parent != kNone && parent.dir != child.dir) ++c;
      }
      return c;
    };

    // Goal test: same cell and heading bin.
    if (keyOf(p.x, p.y, p.yaw, Direction::forward) == goal_key_) {
      double connect = 0.0, shot_length = 0.0;
      size_t cusps = 0;
      bool exact = false;
      toState(p, sa_);
      if (shotGeometry(p, node.dir, shot_length, cusps) && si_->checkMotion(sa_, goal_state_)) {
        connect = objective_->motionCost(sa_, goal_state_).value();
        exact = true;
      } else {
        cusps = 0;
      }
      solution_node = idx;
      solution_appends_goal = exact;
      solution_cusps = chainCusps(idx) + cusps;
      solution_cost = node.g + connect + params_.change_penalty * static_cast<double>(cusps);
      solution_objective_cost = solution_cost - params_.change_penalty * static_cast<double>(solution_cusps);
      result_.termination = "solution";
      break;
    }

    // Analytic expansion on Nav2's schedule.
    ++since_shot;
    const double hk = hKin(p, node.dir);
    const size_t threshold =
        std::max<size_t>(1, static_cast<size_t>(std::floor(hk / (params_.analytic_ratio * primitive_length_))));
    if (since_shot >= threshold) {
      since_shot = 0;
      ++result_.analytic_attempts;
      double shot_cost = 0.0;
      size_t cusps = 0;
      if (tryShot(idx, shot_cost, cusps)) {
        solution_node = idx;
        solution_appends_goal = true;
        solution_cusps = chainCusps(idx) + cusps;
        solution_cost = node.g + shot_cost + params_.change_penalty * static_cast<double>(cusps);
        solution_objective_cost = solution_cost - params_.change_penalty * static_cast<double>(solution_cusps);
        result_.solved_by_shot = true;
        result_.termination = "solution";
        break;
      }
    }

    // Expand.
    for (const auto &[length, curvature] : primitives) {
      const Direction dir = length < 0.0 ? Direction::reverse : Direction::forward;
      const Pose end = primitiveEnd(p, length, curvature);
      if (!inBounds(end.x, end.y)) continue;
      const uint64_t key = keyOf(end.x, end.y, end.yaw, dir);
      const auto existing = key_to_node.find(key);
      if (existing != key_to_node.end() && nodes_[existing->second].closed) continue;

      toState(p, sa_);
      toState(end, sb_);
      if (!si_->checkMotion(sa_, sb_)) continue;
      double g = node.g + objective_->motionCost(sa_, sb_).value();
      if (node.parent != kNone && dir != node.dir) g += params_.change_penalty;

      if (existing != key_to_node.end()) {
        Node &other = nodes_[existing->second];
        if (g >= other.g) continue;
        other.x = end.x;
        other.y = end.y;
        other.yaw = end.yaw;
        other.g = g;
        other.h = hOf(end, dir);
        other.parent = idx;
        other.dir = dir;
        open.emplace(other.g + other.h, existing->second);
      } else {
        const double h = hOf(end, dir);
        nodes_.push_back({end.x, end.y, end.yaw, g, h, idx, dir, false});
        key_to_node.emplace(key, nodes_.size() - 1);
        open.emplace(g + h, nodes_.size() - 1);
      }
      ++result_.generated;
    }
  }
  if (result_.termination.empty()) result_.termination = "exhausted";

  if (solution_node != kNone) {
    path = extractPath(solution_node, solution_appends_goal);
    result_.solved = true;
    result_.cost = solution_cost;
    result_.objective_cost = solution_objective_cost;
    result_.cusps = solution_cusps;
  }
  result_.planning_time_s = elapsed();
  char tail[128];
  if (result_.solved)
    std::snprintf(tail, sizeof(tail), "cost %.3f, %zu cusps%s", result_.cost, result_.cusps,
                  result_.solved_by_shot ? " (analytic shot)" : "");
  else
    std::snprintf(tail, sizeof(tail), "no solution");
  MOD_LOG("HybridAStar: %s after %zu expansions (%zu generated, %zu shots) in %.3f s; heuristic grid %zu x %zu, "
          "%zu / %zu settled, %zu edges, setup %.3f s; %s",
          result_.termination.c_str(), result_.expansions, result_.generated, result_.analytic_attempts,
          result_.planning_time_s, grid_->rows(), grid_->cols(), grid_->settledCount(), grid_->size(),
          grid_->evaluatedEdges(), result_.heuristic_setup_s, tail);
  return path;
}

}  // namespace MoD
