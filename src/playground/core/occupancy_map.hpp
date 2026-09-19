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

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace MoD::playground {

struct Bounds {
  double x_min{0.0}, x_max{0.0}, y_min{0.0}, y_max{0.0};
};

/**
 * Occupancy grid from a ROS map_server style yaml (image, resolution, origin, negate, occupied_thresh,
 * free_thresh) and a binary P5 pgm. A pixel is occupied iff its occupancy probability
 * `p = (255 - v) / 255` (inverted when `negate`) is >= `occupied_thresh`; unknown pixels count as free.
 * World (x, y) maps to the pixel column `floor((x - origin_x) / res)` and the row `floor((y - origin_y) / res)`
 * counted from the bottom of the image. Everything outside the image is occupied.
 */
class OccupancyMap {
 public:
  explicit OccupancyMap(const std::string &yaml_path);

  /// In-memory map: `occupied` is row-major with row 0 at the bottom (y = origin_y).
  OccupancyMap(size_t width, size_t height, double resolution, double origin_x, double origin_y,
               std::vector<uint8_t> occupied);

  const std::string &yamlPath() const { return yaml_path_; }
  const std::string &imagePath() const { return image_path_; }
  size_t width() const { return width_; }
  size_t height() const { return height_; }
  double pixel_size() const { return resolution_; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }
  Bounds bounds() const { return {origin_x_, origin_x_ + resolution_ * static_cast<double>(width_), origin_y_,
                                  origin_y_ + resolution_ * static_cast<double>(height_)}; }

  bool occupied(double x, double y) const;
  /// Pixel test; `row` counts from the bottom. Outside the image is occupied.
  bool occupiedPixel(long col, long row) const {
    if (col < 0 || row < 0 || col >= static_cast<long>(width_) || row >= static_cast<long>(height_)) return true;
    return occupied_[static_cast<size_t>(row) * width_ + static_cast<size_t>(col)] != 0;
  }
  /// World to pixel (row from the bottom); false if outside the image.
  bool worldToPixel(double x, double y, long &col, long &row) const;
  double pixelToX(long col) const { return origin_x_ + (static_cast<double>(col) + 0.5) * resolution_; }
  double pixelToY(long row) const { return origin_y_ + (static_cast<double>(row) + 0.5) * resolution_; }

  /// Original 8-bit pixel values in image order (row 0 at the top); empty for in-memory maps.
  const std::vector<uint8_t> &gray() const { return gray_; }
  /// Occupancy flags, row-major with row 0 at the bottom.
  const std::vector<uint8_t> &occupancy() const { return occupied_; }

  size_t occupiedCount() const;

 private:
  std::string yaml_path_, image_path_;
  size_t width_{0}, height_{0};
  double resolution_{0.0}, origin_x_{0.0}, origin_y_{0.0};
  bool negate_{false};
  double occupied_thresh_{0.65}, free_thresh_{0.196};
  std::vector<uint8_t> gray_;
  std::vector<uint8_t> occupied_;
};

typedef std::shared_ptr<const OccupancyMap> OccupancyMapConstPtr;

}  // namespace MoD::playground
