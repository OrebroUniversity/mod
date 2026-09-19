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

#include "core/occupancy_map.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <mod/log.hpp>
#include <sstream>
#include <stdexcept>

namespace MoD::playground {

namespace {

std::string trim(const std::string &s) {
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

/// Parses "[a, b, c]" into numbers.
std::vector<double> parseList(const std::string &text) {
  std::vector<double> out;
  std::string body = text;
  body.erase(std::remove(body.begin(), body.end(), '['), body.end());
  body.erase(std::remove(body.begin(), body.end(), ']'), body.end());
  std::replace(body.begin(), body.end(), ',', ' ');
  std::istringstream in(body);
  double v;
  while (in >> v) out.push_back(v);
  return out;
}

/// Reads the next whitespace-separated token of a PNM header, skipping '#' comments.
std::string pnmToken(std::istream &in) {
  std::string tok;
  int c;
  while ((c = in.get()) != EOF) {
    if (c == '#') {
      while (c != EOF && c != '\n') c = in.get();
      continue;
    }
    if (std::isspace(c)) {
      if (!tok.empty()) break;
      continue;
    }
    tok.push_back(static_cast<char>(c));
  }
  return tok;
}

}  // namespace

OccupancyMap::OccupancyMap(const std::string &yaml_path) : yaml_path_(yaml_path) {
  std::ifstream yaml(yaml_path);
  if (!yaml) throw std::runtime_error("OccupancyMap: cannot open " + yaml_path);
  std::string image;
  std::string line;
  while (std::getline(yaml, line)) {
    const auto colon = line.find(':');
    if (colon == std::string::npos) continue;
    const std::string key = trim(line.substr(0, colon));
    const std::string value = trim(line.substr(colon + 1));
    if (key == "image") {
      image = value;
    } else if (key == "resolution") {
      resolution_ = std::stod(value);
    } else if (key == "origin") {
      const auto o = parseList(value);
      if (o.size() < 2) throw std::runtime_error("OccupancyMap: bad origin in " + yaml_path);
      origin_x_ = o[0];
      origin_y_ = o[1];
    } else if (key == "negate") {
      negate_ = std::stoi(value) != 0;
    } else if (key == "occupied_thresh") {
      occupied_thresh_ = std::stod(value);
    } else if (key == "free_thresh") {
      free_thresh_ = std::stod(value);
    }
  }
  if (image.empty() || resolution_ <= 0.0) throw std::runtime_error("OccupancyMap: missing image/resolution in " +
                                                                     yaml_path);
  std::filesystem::path img(image);
  if (img.is_relative()) img = std::filesystem::path(yaml_path).parent_path() / img;
  image_path_ = img.string();

  std::ifstream pgm(image_path_, std::ios::binary);
  if (!pgm) throw std::runtime_error("OccupancyMap: cannot open " + image_path_);
  const std::string magic = pnmToken(pgm);
  if (magic != "P5") throw std::runtime_error("OccupancyMap: " + image_path_ + " is not a binary P5 pgm");
  width_ = std::stoul(pnmToken(pgm));
  height_ = std::stoul(pnmToken(pgm));
  const unsigned long maxval = std::stoul(pnmToken(pgm));
  if (maxval != 255) throw std::runtime_error("OccupancyMap: only 8-bit pgm is supported");
  // pnmToken consumed exactly one whitespace character after the maxval token.
  gray_.resize(width_ * height_);
  pgm.read(reinterpret_cast<char *>(gray_.data()), static_cast<std::streamsize>(gray_.size()));
  if (static_cast<size_t>(pgm.gcount()) != gray_.size())
    throw std::runtime_error("OccupancyMap: truncated pgm " + image_path_);

  occupied_.assign(width_ * height_, 0);
  for (size_t img_row = 0; img_row < height_; ++img_row) {
    const size_t row = height_ - 1 - img_row;  // row 0 at the bottom
    for (size_t col = 0; col < width_; ++col) {
      const double v = gray_[img_row * width_ + col];
      double p = (255.0 - v) / 255.0;
      if (negate_) p = 1.0 - p;
      occupied_[row * width_ + col] = p >= occupied_thresh_ ? 1 : 0;
    }
  }
  MOD_LOG("OccupancyMap: %s %zu x %zu px at %.3f m, origin (%.2f, %.2f), %zu occupied", image_path_.c_str(), width_,
          height_, resolution_, origin_x_, origin_y_, occupiedCount());
}

OccupancyMap::OccupancyMap(size_t width, size_t height, double resolution, double origin_x, double origin_y,
                           std::vector<uint8_t> occupied)
    : width_(width),
      height_(height),
      resolution_(resolution),
      origin_x_(origin_x),
      origin_y_(origin_y),
      occupied_(std::move(occupied)) {
  if (occupied_.size() != width_ * height_) throw std::invalid_argument("OccupancyMap: occupancy size mismatch");
}

bool OccupancyMap::worldToPixel(double x, double y, long &col, long &row) const {
  col = static_cast<long>(std::floor((x - origin_x_) / resolution_));
  row = static_cast<long>(std::floor((y - origin_y_) / resolution_));
  return col >= 0 && row >= 0 && col < static_cast<long>(width_) && row < static_cast<long>(height_);
}

bool OccupancyMap::occupied(double x, double y) const {
  long col, row;
  if (!worldToPixel(x, y, col, row)) return true;
  return occupied_[static_cast<size_t>(row) * width_ + static_cast<size_t>(col)] != 0;
}

size_t OccupancyMap::occupiedCount() const {
  size_t n = 0;
  for (auto v : occupied_) n += v;
  return n;
}

}  // namespace MoD::playground
