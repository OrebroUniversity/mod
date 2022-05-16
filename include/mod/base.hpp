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

#include <string>

namespace MoD {
class Base {
 protected:
  /// Frame ID in the ROS message. Might be used for transformation. Right now,
  /// it is only used to fill in the ROS message's header.frame_id.
  std::string frame_id_;
 public:
   /**
   * \brief Get the Frame ID used in ROS messages.
   * \return ROS message header.frame_id.
   */
  inline std::string getFrameID() const { return frame_id_; }

  /**
   * \brief Set the frame ID in ROS message.
   */
  inline void setFrameID(const std::string &frame_id) {
    this->frame_id_ = frame_id;
  }
};
}