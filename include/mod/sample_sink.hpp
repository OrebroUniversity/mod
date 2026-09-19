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

namespace MoD {

/// Which branch produced a sample.
enum class SampleSource { uniform, ellipse, intensity, dijkstra };

inline const char *to_string(SampleSource s) {
  switch (s) {
    case SampleSource::uniform:
      return "uniform";
    case SampleSource::ellipse:
      return "ellipse";
    case SampleSource::intensity:
      return "intensity";
    case SampleSource::dijkstra:
      return "dijkstra";
  }
  return "unknown";
}

/// Optional receiver of every sample a sampler draws. One sink per planner run; not shared between threads.
class SampleSink {
 public:
  virtual ~SampleSink() = default;
  virtual void record(double x, double y, double theta, SampleSource source) = 0;
};

}  // namespace MoD
