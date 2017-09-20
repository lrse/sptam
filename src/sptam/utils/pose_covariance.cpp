/**
 * This file is part of S-PTAM.
 *
 * Copyright (C) 2013-2017 Taihú Pire
 * Copyright (C) 2014-2017 Thomas Fischer
 * Copyright (C) 2016-2017 Gastón Castro
 * Copyright (C) 2017 Matias Nitsche
 * For more information see <https://github.com/lrse/sptam>
 *
 * S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:  Taihú Pire
 *           Thomas Fischer
 *           Gastón Castro
 *           Matías Nitsche
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */
#include "pose_covariance.hpp"

#include <Eigen/Geometry>

Eigen::Matrix<double, 3, 6> dXCdmu(const Eigen::Vector3d& xw, double mu_x, double mu_y, double mu_z, double mu_roll, double mu_pitch, double mu_yaw)
{
  Eigen::Matrix<double, 3, 6> dXC_dmu;

  dXC_dmu(0, 0) = -cos(mu_pitch)*cos(mu_yaw);
  dXC_dmu(0, 1) = -sin(mu_yaw)*cos(mu_pitch);
  dXC_dmu(0, 2) = sin(mu_pitch);
  dXC_dmu(0, 3) = 0;
  dXC_dmu(0, 4) = -xw[0]*sin(mu_pitch)*cos(mu_yaw) - xw[1]*sin(mu_pitch)*sin(mu_yaw) - xw[2]*cos(mu_pitch) + mu_x*sin(mu_pitch)*cos(mu_yaw) + mu_y*sin(mu_pitch)*sin(mu_yaw) + mu_z*cos(mu_pitch);
  dXC_dmu(0, 5) = -xw[0]*sin(mu_yaw)*cos(mu_pitch) + xw[1]*cos(mu_pitch)*cos(mu_yaw) + mu_x*sin(mu_yaw)*cos(mu_pitch) - mu_y*cos(mu_pitch)*cos(mu_yaw);

  dXC_dmu(1, 0) = -sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) + sin(mu_yaw)*cos(mu_roll);
  dXC_dmu(1, 1) = -sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) - cos(mu_roll)*cos(mu_yaw);
  dXC_dmu(1, 2) = -sin(mu_roll)*cos(mu_pitch);
  dXC_dmu(1, 3) = xw[0]*(sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) + sin(mu_roll)*sin(mu_yaw)) + xw[1]*(sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) - sin(mu_roll)*cos(mu_yaw)) + xw[2]*cos(mu_pitch)*cos(mu_roll) + mu_x*(-sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) - sin(mu_roll)*sin(mu_yaw)) + mu_y*(-sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) + sin(mu_roll)*cos(mu_yaw)) - mu_z*cos(mu_pitch)*cos(mu_roll);
  dXC_dmu(1, 4) = xw[0]*sin(mu_roll)*cos(mu_pitch)*cos(mu_yaw) + xw[1]*sin(mu_roll)*sin(mu_yaw)*cos(mu_pitch) - xw[2]*sin(mu_pitch)*sin(mu_roll) - mu_x*sin(mu_roll)*cos(mu_pitch)*cos(mu_yaw) - mu_y*sin(mu_roll)*sin(mu_yaw)*cos(mu_pitch) + mu_z*sin(mu_pitch)*sin(mu_roll);
  dXC_dmu(1, 5) = xw[0]*(-sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) - cos(mu_roll)*cos(mu_yaw)) + xw[1]*(sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) - sin(mu_yaw)*cos(mu_roll)) + mu_x*(sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) + cos(mu_roll)*cos(mu_yaw)) + mu_y*(-sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) + sin(mu_yaw)*cos(mu_roll));

  dXC_dmu(2, 0) = -sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) - sin(mu_roll)*sin(mu_yaw);
  dXC_dmu(2, 1) = -sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) + sin(mu_roll)*cos(mu_yaw);
  dXC_dmu(2, 2) = -cos(mu_pitch)*cos(mu_roll);
  dXC_dmu(2, 3) = xw[0]*(-sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) + sin(mu_yaw)*cos(mu_roll)) + xw[1]*(-sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) - cos(mu_roll)*cos(mu_yaw)) - xw[2]*sin(mu_roll)*cos(mu_pitch) + mu_x*(sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) - sin(mu_yaw)*cos(mu_roll)) + mu_y*(sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) + cos(mu_roll)*cos(mu_yaw)) + mu_z*sin(mu_roll)*cos(mu_pitch);
  dXC_dmu(2, 4) = xw[0]*cos(mu_pitch)*cos(mu_roll)*cos(mu_yaw) + xw[1]*sin(mu_yaw)*cos(mu_pitch)*cos(mu_roll) - xw[2]*sin(mu_pitch)*cos(mu_roll) - mu_x*cos(mu_pitch)*cos(mu_roll)*cos(mu_yaw) - mu_y*sin(mu_yaw)*cos(mu_pitch)*cos(mu_roll) + mu_z*sin(mu_pitch)*cos(mu_roll);
  dXC_dmu(2, 5) = xw[0]*(-sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) + sin(mu_roll)*cos(mu_yaw)) + xw[1]*(sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) + sin(mu_roll)*sin(mu_yaw)) + mu_x*(sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) - sin(mu_roll)*cos(mu_yaw)) + mu_y*(-sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) - sin(mu_roll)*sin(mu_yaw));

  return dXC_dmu;
}

Eigen::Matrix6d dXC2dmu2_0(const Eigen::Vector3d& xw, double mu_x, double mu_y, double mu_z, double mu_roll, double mu_pitch, double mu_yaw)
{
  Eigen::Matrix6d dXC2_dmu2_0;

  dXC2_dmu2_0(0, 0) = 0;
  dXC2_dmu2_0(0, 1) = 0;
  dXC2_dmu2_0(0, 2) = 0;
  dXC2_dmu2_0(0, 3) = 0;
  dXC2_dmu2_0(0, 4) = sin(mu_pitch)*cos(mu_yaw);
  dXC2_dmu2_0(0, 5) = sin(mu_yaw)*cos(mu_pitch);

  dXC2_dmu2_0(1, 0) = 0;
  dXC2_dmu2_0(1, 1) = 0;
  dXC2_dmu2_0(1, 2) = 0;
  dXC2_dmu2_0(1, 3) = 0;
  dXC2_dmu2_0(1, 4) = sin(mu_pitch)*sin(mu_yaw);
  dXC2_dmu2_0(1, 5) = -cos(mu_pitch)*cos(mu_yaw);

  dXC2_dmu2_0(2, 0) = 0;
  dXC2_dmu2_0(2, 1) = 0;
  dXC2_dmu2_0(2, 2) = 0;
  dXC2_dmu2_0(2, 3) = 0;
  dXC2_dmu2_0(2, 4) = cos(mu_pitch);
  dXC2_dmu2_0(2, 5) = 0;

  dXC2_dmu2_0(3, 0) = 0;
  dXC2_dmu2_0(3, 1) = 0;
  dXC2_dmu2_0(3, 2) = 0;
  dXC2_dmu2_0(3, 3) = 0;
  dXC2_dmu2_0(3, 4) = 0;
  dXC2_dmu2_0(3, 5) = 0;

  dXC2_dmu2_0(4, 0) = sin(mu_pitch)*cos(mu_yaw);
  dXC2_dmu2_0(4, 1) = sin(mu_pitch)*sin(mu_yaw);
  dXC2_dmu2_0(4, 2) = cos(mu_pitch);
  dXC2_dmu2_0(4, 3) = 0;
  dXC2_dmu2_0(4, 4) = -xw[0]*cos(mu_pitch)*cos(mu_yaw) - xw[1]*sin(mu_yaw)*cos(mu_pitch) + xw[2]*sin(mu_pitch) + mu_x*cos(mu_pitch)*cos(mu_yaw) + mu_y*sin(mu_yaw)*cos(mu_pitch) - mu_z*sin(mu_pitch);
  dXC2_dmu2_0(4, 5) = xw[0]*sin(mu_pitch)*sin(mu_yaw) - xw[1]*sin(mu_pitch)*cos(mu_yaw) - mu_x*sin(mu_pitch)*sin(mu_yaw) + mu_y*sin(mu_pitch)*cos(mu_yaw);

  dXC2_dmu2_0(5, 0) = sin(mu_yaw)*cos(mu_pitch);
  dXC2_dmu2_0(5, 1) = -cos(mu_pitch)*cos(mu_yaw);
  dXC2_dmu2_0(5, 2) = 0;
  dXC2_dmu2_0(5, 3) = 0;
  dXC2_dmu2_0(5, 4) = xw[0]*sin(mu_pitch)*sin(mu_yaw) - xw[1]*sin(mu_pitch)*cos(mu_yaw) - mu_x*sin(mu_pitch)*sin(mu_yaw) + mu_y*sin(mu_pitch)*cos(mu_yaw);
  dXC2_dmu2_0(5, 5) = -xw[0]*cos(mu_pitch)*cos(mu_yaw) - xw[1]*sin(mu_yaw)*cos(mu_pitch) + mu_x*cos(mu_pitch)*cos(mu_yaw) + mu_y*sin(mu_yaw)*cos(mu_pitch);

  return dXC2_dmu2_0;
}

Eigen::Matrix6d dXC2dmu2_1(const Eigen::Vector3d& xw, double mu_x, double mu_y, double mu_z, double mu_roll, double mu_pitch, double mu_yaw)
{
  Eigen::Matrix6d dXC2_dmu2_1;

  dXC2_dmu2_1(0, 0) = 0;
  dXC2_dmu2_1(0, 1) = 0;
  dXC2_dmu2_1(0, 2) = 0;
  dXC2_dmu2_1(0, 3) = -sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) - sin(mu_roll)*sin(mu_yaw);
  dXC2_dmu2_1(0, 4) = -sin(mu_roll)*cos(mu_pitch)*cos(mu_yaw);
  dXC2_dmu2_1(0, 5) = sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) + cos(mu_roll)*cos(mu_yaw);

  dXC2_dmu2_1(1, 0) = 0;
  dXC2_dmu2_1(1, 1) = 0;
  dXC2_dmu2_1(1, 2) = 0;
  dXC2_dmu2_1(1, 3) = -sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) + sin(mu_roll)*cos(mu_yaw);
  dXC2_dmu2_1(1, 4) = -sin(mu_roll)*sin(mu_yaw)*cos(mu_pitch);
  dXC2_dmu2_1(1, 5) = -sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) + sin(mu_yaw)*cos(mu_roll);

  dXC2_dmu2_1(2, 0) = 0;
  dXC2_dmu2_1(2, 1) = 0;
  dXC2_dmu2_1(2, 2) = 0;
  dXC2_dmu2_1(2, 3) = -cos(mu_pitch)*cos(mu_roll);
  dXC2_dmu2_1(2, 4) = sin(mu_pitch)*sin(mu_roll);
  dXC2_dmu2_1(2, 5) = 0;

  dXC2_dmu2_1(3, 0) = -sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) - sin(mu_roll)*sin(mu_yaw);
  dXC2_dmu2_1(3, 1) = -sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) + sin(mu_roll)*cos(mu_yaw);
  dXC2_dmu2_1(3, 2) = -cos(mu_pitch)*cos(mu_roll);
  dXC2_dmu2_1(3, 3) = xw[0]*(-sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) + sin(mu_yaw)*cos(mu_roll)) + xw[1]*(-sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) - cos(mu_roll)*cos(mu_yaw)) - xw[2]*sin(mu_roll)*cos(mu_pitch) + mu_x*(sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) - sin(mu_yaw)*cos(mu_roll)) + mu_y*(sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) + cos(mu_roll)*cos(mu_yaw)) + mu_z*sin(mu_roll)*cos(mu_pitch);
  dXC2_dmu2_1(3, 4) = xw[0]*cos(mu_pitch)*cos(mu_roll)*cos(mu_yaw) + xw[1]*sin(mu_yaw)*cos(mu_pitch)*cos(mu_roll) - xw[2]*sin(mu_pitch)*cos(mu_roll) - mu_x*cos(mu_pitch)*cos(mu_roll)*cos(mu_yaw) - mu_y*sin(mu_yaw)*cos(mu_pitch)*cos(mu_roll) + mu_z*sin(mu_pitch)*cos(mu_roll);
  dXC2_dmu2_1(3, 5) = xw[0]*(-sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) + sin(mu_roll)*cos(mu_yaw)) + xw[1]*(sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) + sin(mu_roll)*sin(mu_yaw)) + mu_x*(sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) - sin(mu_roll)*cos(mu_yaw)) + mu_y*(-sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) - sin(mu_roll)*sin(mu_yaw));

  dXC2_dmu2_1(4, 0) = -sin(mu_roll)*cos(mu_pitch)*cos(mu_yaw);
  dXC2_dmu2_1(4, 1) = -sin(mu_roll)*sin(mu_yaw)*cos(mu_pitch);
  dXC2_dmu2_1(4, 2) = sin(mu_pitch)*sin(mu_roll);
  dXC2_dmu2_1(4, 3) = xw[0]*cos(mu_pitch)*cos(mu_roll)*cos(mu_yaw) + xw[1]*sin(mu_yaw)*cos(mu_pitch)*cos(mu_roll) - xw[2]*sin(mu_pitch)*cos(mu_roll) - mu_x*cos(mu_pitch)*cos(mu_roll)*cos(mu_yaw) - mu_y*sin(mu_yaw)*cos(mu_pitch)*cos(mu_roll) + mu_z*sin(mu_pitch)*cos(mu_roll);
  dXC2_dmu2_1(4, 4) = -xw[0]*sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) - xw[1]*sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) - xw[2]*sin(mu_roll)*cos(mu_pitch) + mu_x*sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) + mu_y*sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) + mu_z*sin(mu_roll)*cos(mu_pitch);
  dXC2_dmu2_1(4, 5) = -xw[0]*sin(mu_roll)*sin(mu_yaw)*cos(mu_pitch) + xw[1]*sin(mu_roll)*cos(mu_pitch)*cos(mu_yaw) + mu_x*sin(mu_roll)*sin(mu_yaw)*cos(mu_pitch) - mu_y*sin(mu_roll)*cos(mu_pitch)*cos(mu_yaw);

  dXC2_dmu2_1(5, 0) = sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) + cos(mu_roll)*cos(mu_yaw);
  dXC2_dmu2_1(5, 1) = -sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) + sin(mu_yaw)*cos(mu_roll);
  dXC2_dmu2_1(5, 2) = 0;
  dXC2_dmu2_1(5, 3) = xw[0]*(-sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) + sin(mu_roll)*cos(mu_yaw)) + xw[1]*(sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) + sin(mu_roll)*sin(mu_yaw)) + mu_x*(sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) - sin(mu_roll)*cos(mu_yaw)) + mu_y*(-sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) - sin(mu_roll)*sin(mu_yaw));
  dXC2_dmu2_1(5, 4) = -xw[0]*sin(mu_roll)*sin(mu_yaw)*cos(mu_pitch) + xw[1]*sin(mu_roll)*cos(mu_pitch)*cos(mu_yaw) + mu_x*sin(mu_roll)*sin(mu_yaw)*cos(mu_pitch) - mu_y*sin(mu_roll)*cos(mu_pitch)*cos(mu_yaw);
  dXC2_dmu2_1(5, 5) = xw[0]*(-sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) + sin(mu_yaw)*cos(mu_roll)) + xw[1]*(-sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) - cos(mu_roll)*cos(mu_yaw)) + mu_x*(sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) - sin(mu_yaw)*cos(mu_roll)) + mu_y*(sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) + cos(mu_roll)*cos(mu_yaw));

  return dXC2_dmu2_1;
}

Eigen::Matrix6d dXC2dmu2_2(const Eigen::Vector3d& xw, double mu_x, double mu_y, double mu_z, double mu_roll, double mu_pitch, double mu_yaw)
{
  Eigen::Matrix6d dXC2_dmu2_2;

  dXC2_dmu2_2(0, 0) = 0;
  dXC2_dmu2_2(0, 1) = 0;
  dXC2_dmu2_2(0, 2) = 0;
  dXC2_dmu2_2(0, 3) = sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) - sin(mu_yaw)*cos(mu_roll);
  dXC2_dmu2_2(0, 4) = -cos(mu_pitch)*cos(mu_roll)*cos(mu_yaw);
  dXC2_dmu2_2(0, 5) = sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) - sin(mu_roll)*cos(mu_yaw);

  dXC2_dmu2_2(1, 0) = 0;
  dXC2_dmu2_2(1, 1) = 0;
  dXC2_dmu2_2(1, 2) = 0;
  dXC2_dmu2_2(1, 3) = sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) + cos(mu_roll)*cos(mu_yaw);
  dXC2_dmu2_2(1, 4) = -sin(mu_yaw)*cos(mu_pitch)*cos(mu_roll);
  dXC2_dmu2_2(1, 5) = -sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) - sin(mu_roll)*sin(mu_yaw);

  dXC2_dmu2_2(2, 0) = 0;
  dXC2_dmu2_2(2, 1) = 0;
  dXC2_dmu2_2(2, 2) = 0;
  dXC2_dmu2_2(2, 3) = sin(mu_roll)*cos(mu_pitch);
  dXC2_dmu2_2(2, 4) = sin(mu_pitch)*cos(mu_roll);
  dXC2_dmu2_2(2, 5) = 0;

  dXC2_dmu2_2(3, 0) = sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) - sin(mu_yaw)*cos(mu_roll);
  dXC2_dmu2_2(3, 1) = sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) + cos(mu_roll)*cos(mu_yaw);
  dXC2_dmu2_2(3, 2) = sin(mu_roll)*cos(mu_pitch);
  dXC2_dmu2_2(3, 3) = xw[0]*(-sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) - sin(mu_roll)*sin(mu_yaw)) + xw[1]*(-sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) + sin(mu_roll)*cos(mu_yaw)) - xw[2]*cos(mu_pitch)*cos(mu_roll) + mu_x*(sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) + sin(mu_roll)*sin(mu_yaw)) + mu_y*(sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) - sin(mu_roll)*cos(mu_yaw)) + mu_z*cos(mu_pitch)*cos(mu_roll);
  dXC2_dmu2_2(3, 4) = -xw[0]*sin(mu_roll)*cos(mu_pitch)*cos(mu_yaw) - xw[1]*sin(mu_roll)*sin(mu_yaw)*cos(mu_pitch) + xw[2]*sin(mu_pitch)*sin(mu_roll) + mu_x*sin(mu_roll)*cos(mu_pitch)*cos(mu_yaw) + mu_y*sin(mu_roll)*sin(mu_yaw)*cos(mu_pitch) - mu_z*sin(mu_pitch)*sin(mu_roll);
  dXC2_dmu2_2(3, 5) = xw[0]*(sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) + cos(mu_roll)*cos(mu_yaw)) + xw[1]*(-sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) + sin(mu_yaw)*cos(mu_roll)) + mu_x*(-sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) - cos(mu_roll)*cos(mu_yaw)) + mu_y*(sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) - sin(mu_yaw)*cos(mu_roll));

  dXC2_dmu2_2(4, 0) = -cos(mu_pitch)*cos(mu_roll)*cos(mu_yaw);
  dXC2_dmu2_2(4, 1) = -sin(mu_yaw)*cos(mu_pitch)*cos(mu_roll);
  dXC2_dmu2_2(4, 2) = sin(mu_pitch)*cos(mu_roll);
  dXC2_dmu2_2(4, 3) = -xw[0]*sin(mu_roll)*cos(mu_pitch)*cos(mu_yaw) - xw[1]*sin(mu_roll)*sin(mu_yaw)*cos(mu_pitch) + xw[2]*sin(mu_pitch)*sin(mu_roll) + mu_x*sin(mu_roll)*cos(mu_pitch)*cos(mu_yaw) + mu_y*sin(mu_roll)*sin(mu_yaw)*cos(mu_pitch) - mu_z*sin(mu_pitch)*sin(mu_roll);
  dXC2_dmu2_2(4, 4) = -xw[0]*sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) - xw[1]*sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) - xw[2]*cos(mu_pitch)*cos(mu_roll) + mu_x*sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) + mu_y*sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) + mu_z*cos(mu_pitch)*cos(mu_roll);
  dXC2_dmu2_2(4, 5) = -xw[0]*sin(mu_yaw)*cos(mu_pitch)*cos(mu_roll) + xw[1]*cos(mu_pitch)*cos(mu_roll)*cos(mu_yaw) + mu_x*sin(mu_yaw)*cos(mu_pitch)*cos(mu_roll) - mu_y*cos(mu_pitch)*cos(mu_roll)*cos(mu_yaw);

  dXC2_dmu2_2(5, 0) = sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) - sin(mu_roll)*cos(mu_yaw);
  dXC2_dmu2_2(5, 1) = -sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) - sin(mu_roll)*sin(mu_yaw);
  dXC2_dmu2_2(5, 2) = 0;
  dXC2_dmu2_2(5, 3) = xw[0]*(sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) + cos(mu_roll)*cos(mu_yaw)) + xw[1]*(-sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) + sin(mu_yaw)*cos(mu_roll)) + mu_x*(-sin(mu_pitch)*sin(mu_roll)*sin(mu_yaw) - cos(mu_roll)*cos(mu_yaw)) + mu_y*(sin(mu_pitch)*sin(mu_roll)*cos(mu_yaw) - sin(mu_yaw)*cos(mu_roll));
  dXC2_dmu2_2(5, 4) = -xw[0]*sin(mu_yaw)*cos(mu_pitch)*cos(mu_roll) + xw[1]*cos(mu_pitch)*cos(mu_roll)*cos(mu_yaw) + mu_x*sin(mu_yaw)*cos(mu_pitch)*cos(mu_roll) - mu_y*cos(mu_pitch)*cos(mu_roll)*cos(mu_yaw);
  dXC2_dmu2_2(5, 5) = xw[0]*(-sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) - sin(mu_roll)*sin(mu_yaw)) + xw[1]*(-sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) + sin(mu_roll)*cos(mu_yaw)) + mu_x*(sin(mu_pitch)*cos(mu_roll)*cos(mu_yaw) + sin(mu_roll)*sin(mu_yaw)) + mu_y*(sin(mu_pitch)*sin(mu_yaw)*cos(mu_roll) - sin(mu_roll)*cos(mu_yaw));

  return dXC2_dmu2_2;
}

Eigen::Matrix6d sarasa1(const Eigen::Vector3d& xw, double mu_x, double mu_y, double mu_z, double mu_roll, double mu_pitch, double mu_yaw, const Eigen::Matrix<double, 1, 3>& xz_XC)
{
  return
      xz_XC(0, 0) * dXC2dmu2_0(xw, mu_x, mu_y, mu_z, mu_roll, mu_pitch, mu_yaw)
    + xz_XC(0, 1) * dXC2dmu2_1(xw, mu_x, mu_y, mu_z, mu_roll, mu_pitch, mu_yaw)
    + xz_XC(0, 2) * dXC2dmu2_2(xw, mu_x, mu_y, mu_z, mu_roll, mu_pitch, mu_yaw);
}

Eigen::Matrix6d sarasa2(const Eigen::Vector3d& xw, double mu_x, double mu_y, double mu_z, double mu_roll, double mu_pitch, double mu_yaw, const Eigen::Matrix<double, 1, 3>& yz_XC)
{
  return
      yz_XC(0, 0) * dXC2dmu2_0(xw, mu_x, mu_y, mu_z, mu_roll, mu_pitch, mu_yaw)
    + yz_XC(0, 1) * dXC2dmu2_1(xw, mu_x, mu_y, mu_z, mu_roll, mu_pitch, mu_yaw)
    + yz_XC(0, 2) * dXC2dmu2_2(xw, mu_x, mu_y, mu_z, mu_roll, mu_pitch, mu_yaw);
}

Eigen::Matrix6d computeMeasurementCovariance(double mu_x, double mu_y, double mu_z, double mu_roll, double mu_pitch, double mu_yaw, const Eigen::Vector3d& xw, const Eigen::Vector3d& xc, const Eigen::Vector2d& z, const Eigen::Matrix3d& K, const Eigen::Matrix2d& cov_z)
{
  double fu = K(0, 0);
  double fv = K(1, 1);
  double u0 = K(0, 2);
  double v0 = K(1, 2);

  // compute derivative of XC[0]/XC[2] (the part of g1 that depends on mu) over XC
  Eigen::Matrix<double, 1, 3> xz_XC;
  xz_XC(0, 0) = 1 / xc[2];
  xz_XC(0, 1) = 0;
  xz_XC(0, 2) = xc[0] / (xc[2]*xc[2]);

  Eigen::Matrix3d xz2_XC2 = Eigen::Matrix3d::Zero();
  xz2_XC2(0, 2) = -1 / (xc[2]*xc[2]);
  xz2_XC2(2, 0) = -1 / (xc[2]*xc[2]);
  xz2_XC2(2, 2) = 2*xc[0] / (xc[2]*xc[2]*xc[2]);

  // compute derivative of XC[1]/XC[2] (the part of g2 that depends on mu) over XC
  Eigen::Matrix<double, 1, 3> yz_XC;
  yz_XC(0, 0) = 0;
  yz_XC(0, 1) = 1 / xc[2];
  yz_XC(0, 2) = xc[1] / (xc[2]*xc[2]);

  Eigen::Matrix3d yz2_XC2 = Eigen::Matrix3d::Zero();
  yz2_XC2(1, 2) = -1 / (xc[2]*xc[2]);
  yz2_XC2(2, 1) = -1 / (xc[2]*xc[2]);
  yz2_XC2(2, 2) = 2*xc[1] / (xc[2]*xc[2]*xc[2]);

  Eigen::Matrix<double, 3, 6> dXC_dmu = dXCdmu( xw, mu_x, mu_y, mu_z, mu_roll, mu_pitch, mu_yaw );

  Eigen::Matrix<double, 1, 6> dg1_dmu = -2 * fu * xz_XC * dXC_dmu;
  Eigen::Matrix<double, 1, 6> dg2_dmu = -2 * fv * yz_XC * dXC_dmu;

  // Compute d^2 e / dz du
  Eigen::Matrix<double, 6, 2> d2e_dzdu;
  d2e_dzdu.block( 0, 0, 6, 1 ) = dg1_dmu.transpose();
  d2e_dzdu.block( 0, 1, 6, 1 ) = dg2_dmu.transpose();

  double g1 = z[0] - (fu*xc[0]/xc[2] + u0);
  double g2 = z[1] - (fv*xc[1]/xc[2] + v0);

  // Compute d^2 e / du^2
  // sarasa1: (xz_XC * dXC2_dmu2)
  // sarasa2: (yz_XC * dXC2_dmu2)
  Eigen::Matrix6d d2e_du2 =
      dg1_dmu.transpose() * dg1_dmu + g1*2*fu*( dXC_dmu.transpose() * xz2_XC2 * dXC_dmu + sarasa1(xw, mu_x, mu_y, mu_z, mu_roll, mu_pitch, mu_yaw, xz_XC) )
    + dg2_dmu.transpose() * dg2_dmu + g2*2*fv*( dXC_dmu.transpose() * yz2_XC2 * dXC_dmu + sarasa2(xw, mu_x, mu_y, mu_z, mu_roll, mu_pitch, mu_yaw, yz_XC) );

  Eigen::Matrix<double, 6, 2> dA_dz = d2e_du2.inverse() * d2e_dzdu;

  return dA_dz * cov_z * dA_dz.transpose();
}
