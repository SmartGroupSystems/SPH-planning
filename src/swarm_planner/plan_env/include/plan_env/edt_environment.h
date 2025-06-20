/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <utility>

#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_map.h>

using std::cout;
using std::endl;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace FLAG_Race {
class EDTEnvironment {
private:
  /* data */
  ObjPrediction obj_prediction_;
  ObjScale obj_scale_;
  double resolution_inv_;
  double distToBox(int idx, const Eigen::Vector3d& pos, const double& time);
  double minDistToAllBox(const Eigen::Vector3d& pos, const double& time);

public:
  EDTEnvironment(/* args */) {
  }
  ~EDTEnvironment() {
  }

  SDFMap::Ptr sdf_map_;

  void init();
  void setMap(SDFMap::Ptr map);
  void setObjPrediction(ObjPrediction prediction);
  void setObjScale(ObjScale scale);
  void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
  pair<double, Eigen::Vector3d> interpolateTrilinear(double values[2][2][2], const Eigen::Vector3d& diff,
                                                     double& value, Eigen::Vector3d& grad);
  pair<double, Eigen::Vector3d> evaluateEDTWithGrad(const Eigen::Vector3d& pos, double time,
                                                    double& dist, Eigen::Vector3d& grad);
  double evaluateCoarseEDT(Eigen::Vector3d& pos, double time);
  void getMapRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
    sdf_map_->getRegion(ori, size);
  }

  inline void printArrays(const Eigen::Vector3d sur_pts[2][2][2], const double dists[2][2][2]) {
    std::cout << "Printing sur_pts:" << std::endl;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 2; ++k) {
                std::cout << "sur_pts[" << i << "][" << j << "][" << k << "] = "
                          << sur_pts[i][j][k].transpose() << std::endl;
            }
        }
    }

    std::cout << "\nPrinting dists:" << std::endl;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 2; ++k) {
                std::cout << "dists[" << i << "][" << j << "][" << k << "] = "
                          << dists[i][j][k] << std::endl;
            }
        }
    }
};

  typedef shared_ptr<EDTEnvironment> Ptr;
};

}  // namespace FLAG_Race

#endif