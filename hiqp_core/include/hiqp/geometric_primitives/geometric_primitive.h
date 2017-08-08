// The HiQP Control Framework, an optimal control framework targeted at robotics
// Copyright (C) 2016 Marcus A Johansson
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef HIQP_GEOMETRIC_PRIMITIVE_H
#define HIQP_GEOMETRIC_PRIMITIVE_H

#include <atomic>
#include <string>
#include <vector>

namespace hiqp {
namespace geometric_primitives {

/*!
 * \class GeometricPrimitive
 * \brief Abstract base class for all geometric primitives.
 */
class GeometricPrimitive {
 public:
  GeometricPrimitive(const std::string& name, const std::string& frame_id,
                     bool visible, const std::vector<double>& color)
      : name_(name), frame_id_(frame_id), visible_(visible), visual_id_(-1) {
    r_ = color.at(0);
    g_ = color.at(1);
    b_ = color.at(2);
    a_ = color.at(3);
  }

  ~GeometricPrimitive() noexcept = default;

  /*! \brief Must be specified by the inheriting class. */
  virtual int init(const std::vector<double>& parameters) = 0;

  inline int setVisualId(int visual_id) { visual_id_ = visual_id; }
  inline int getVisualId() { return visual_id_; }

  inline std::string getName() { return name_; }
  inline std::string getFrameId() { return frame_id_; }
  inline bool isVisible() { return visible_; }

  inline double getRedComponent() { return r_; }
  inline double getGreenComponent() { return g_; }
  inline double getBlueComponent() { return b_; }
  inline double getAlphaComponent() { return a_; }
  inline std::vector<double> getColor() { return {r_, g_, b_, a_}; }
  inline std::vector<double> getParameters() { return parameters_; }

  inline void resetFrameId(std::string frame_id) { frame_id_ = frame_id; }
 protected:
  std::string name_;
  std::string frame_id_;
  bool visible_;
  int visual_id_;
  double r_, g_, b_, a_;
  std::vector<double> parameters_;

 private:
  GeometricPrimitive(const GeometricPrimitive& other) = delete;
  GeometricPrimitive(GeometricPrimitive&& other) = delete;
  GeometricPrimitive& operator=(const GeometricPrimitive& other) = delete;
  GeometricPrimitive& operator=(GeometricPrimitive&& other) noexcept = delete;
};

}  // namespace geometric_primitives

}  // namespace hiqp

#endif  // include guard
