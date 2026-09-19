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
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace MoD {

/**
 * Dijkstra over an implicit 8-neighbour grid laid over [x_min, x_max] x [y_min, y_max] at a given cell size.
 * Node (row, col) sits at (x_min + col * cell, y_min + row * cell). Validity is cached per node from a checker
 * callback; an edge exists iff both end nodes are valid. Edge weights come from a callback `w(n, m)` evaluated
 * when node n is popped (lazy deletion binary heap).
 *
 * Forward mode: root = start, `costTo(node)` is the cost from the root, `pathTo(node)` runs root -> node.
 * Reverse mode: root = goal, `costTo(node)` is the cost-to-go, relaxing `w(n, m)` on predecessors n of a
 * settled m; `pathTo(node)` runs node -> root.
 *
 * `costTo` is lazy: it expands the queue until that node settles (or the queue runs dry), so callers that need
 * only a region pay only for that region; `expandAll()` settles everything reachable.
 */
class GridDijkstra {
 public:
  enum class Mode { forward, reverse };
  typedef std::function<bool(double x, double y)> ValidityFn;
  typedef std::function<double(size_t from, size_t to)> WeightFn;

  static constexpr size_t kNoNode = std::numeric_limits<size_t>::max();
  static constexpr double kInf = std::numeric_limits<double>::infinity();

  GridDijkstra(double x_min, double x_max, double y_min, double y_max, double cell_size)
      : x_min_(x_min), y_min_(y_min), cell_size_(cell_size) {
    cols_ = static_cast<size_t>((x_max - x_min) / cell_size) + 1u;
    rows_ = static_cast<size_t>((y_max - y_min) / cell_size) + 1u;
    x_max_ = colToX(cols_ - 1);
    y_max_ = rowToY(rows_ - 1);
    valid_.assign(rows_ * cols_, 1);
    reset();
  }

  size_t rows() const { return rows_; }
  size_t cols() const { return cols_; }
  size_t size() const { return rows_ * cols_; }
  double cellSize() const { return cell_size_; }
  double xMin() const { return x_min_; }
  double yMin() const { return y_min_; }
  /// Coordinates of the last column / row (may be below the bounds given to the constructor).
  double xMax() const { return x_max_; }
  double yMax() const { return y_max_; }

  double colToX(size_t col) const { return static_cast<double>(col) * cell_size_ + x_min_; }
  double rowToY(size_t row) const { return static_cast<double>(row) * cell_size_ + y_min_; }
  size_t index(size_t row, size_t col) const { return row * cols_ + col; }
  size_t row(size_t node) const { return node / cols_; }
  size_t col(size_t node) const { return node % cols_; }
  double x(size_t node) const { return colToX(col(node)); }
  double y(size_t node) const { return rowToY(row(node)); }

  /// The node whose cell (node position +/- cell/2) contains (x, y), clamped to the grid.
  size_t nodeAt(double x, double y) const {
    const long c = std::lround((x - x_min_) / cell_size_);
    const long r = std::lround((y - y_min_) / cell_size_);
    const size_t cc = static_cast<size_t>(std::clamp<long>(c, 0, static_cast<long>(cols_) - 1));
    const size_t rr = static_cast<size_t>(std::clamp<long>(r, 0, static_cast<long>(rows_) - 1));
    return index(rr, cc);
  }

  /// Fills the validity cache with one checker call per node.
  void computeValidity(const ValidityFn &checker) {
    for (size_t n = 0; n < size(); ++n) valid_[n] = checker(x(n), y(n)) ? 1 : 0;
  }
  void setValid(size_t node, bool v) { valid_[node] = v ? 1 : 0; }
  bool valid(size_t node) const { return valid_[node] != 0; }
  size_t validCount() const {
    size_t n = 0;
    for (auto v : valid_) n += v;
    return n;
  }

  void setWeight(WeightFn w) { weight_ = std::move(w); }

  /// Starts a new search from `root`; discards previous distances.
  void setRoot(size_t root, Mode mode) {
    reset();
    root_ = root;
    mode_ = mode;
    if (root_ < size() && valid(root_)) {
      dist_[root_] = 0.0;
      queue_.emplace(0.0, root_);
    }
  }

  size_t root() const { return root_; }
  Mode mode() const { return mode_; }

  /// Cost from the root (forward) or to the root (reverse); expands until `node` settles. +inf if unreachable.
  double costTo(size_t node) {
    if (node >= size()) return kInf;
    if (!settled_[node]) run(node);
    return dist_[node];
  }

  /// Settles every reachable node.
  void expandAll() { run(kNoNode); }

  bool settled(size_t node) const { return settled_[node] != 0; }
  size_t settledCount() const { return settled_count_; }
  size_t evaluatedEdges() const { return evaluated_edges_; }

  /// Node sequence root -> node (forward) or node -> root (reverse); empty if `node` is unreachable.
  std::vector<size_t> pathTo(size_t node) {
    std::vector<size_t> path;
    if (!std::isfinite(costTo(node))) return path;
    for (size_t n = node; n != kNoNode; n = pred_[n]) path.push_back(n);
    if (mode_ == Mode::forward) std::reverse(path.begin(), path.end());
    return path;
  }

 private:
  typedef std::pair<double, size_t> QueueItem;

  void reset() {
    dist_.assign(size(), kInf);
    pred_.assign(size(), kNoNode);
    settled_.assign(size(), 0);
    queue_ = std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>>();
    settled_count_ = 0;
    evaluated_edges_ = 0;
  }

  /// Runs the queue until `target` settles (or everything is settled when target == kNoNode).
  void run(size_t target) {
    while (!queue_.empty()) {
      const QueueItem top = queue_.top();
      queue_.pop();
      const size_t n = top.second;
      if (settled_[n]) continue;  // lazy deletion
      settled_[n] = 1;
      ++settled_count_;
      if (n == target) return;
      const double d = dist_[n];
      const size_t r = row(n), c = col(n);
      for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
          if (dr == 0 && dc == 0) continue;
          const long rr = static_cast<long>(r) + dr, cc = static_cast<long>(c) + dc;
          if (rr < 0 || cc < 0 || rr >= static_cast<long>(rows_) || cc >= static_cast<long>(cols_)) continue;
          const size_t m = index(static_cast<size_t>(rr), static_cast<size_t>(cc));
          if (!valid_[m] || settled_[m]) continue;
          const double w = mode_ == Mode::forward ? weight_(n, m) : weight_(m, n);
          ++evaluated_edges_;
          const double nd = d + w;
          if (nd < dist_[m]) {
            dist_[m] = nd;
            pred_[m] = n;
            queue_.emplace(nd, m);
          }
        }
      }
    }
  }

  double x_min_, y_min_, cell_size_;
  double x_max_{0.0}, y_max_{0.0};
  size_t rows_{0}, cols_{0};
  std::vector<uint8_t> valid_;
  WeightFn weight_;

  size_t root_{kNoNode};
  Mode mode_{Mode::forward};
  std::vector<double> dist_;
  std::vector<size_t> pred_;
  std::vector<uint8_t> settled_;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue_;
  size_t settled_count_{0};
  size_t evaluated_edges_{0};
};

}  // namespace MoD
