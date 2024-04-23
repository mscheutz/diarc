/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * File:   Grasp.hpp
 * Author: Evan Krause
 *
 * Created on July 22, 2015, 8:10 PM
 */

#ifndef GRASP_HPP
#define	GRASP_HPP

#include <eigen3/Eigen/Geometry>

namespace ade {
  namespace stm {

    class Grasp {
    public:

      enum Type {
        PINCH_TOGETHER, PINCH_APART, PUSH, TWO_ARM
      };

      //! type of grasp
      Type type_;

      //! 3D point(s) representing location(s) on object
      std::vector<Eigen::Vector3f> points_;

      //! quaternion(s) specifying orientation(s) of end effector(s)
      std::vector<Eigen::Quaternionf> orientations_;
    };

  } //namespace stm
} //namespace ade

#endif	/* GRASP_HPP */

