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
#pragma once

// src: http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/

#include <Eigen/Eigenvalues> 

enum prob_t {PROB_95=0, PROB_99};

// for 2D ellipses
const double chi2_threshold_2D[2] = {5.991, 9.210};

// for 3D ellipses
const double chi2_threshold_3D[2] = {7.815, 11.345};

struct Ellipse2D
{
  double len1, len2;
  Eigen::Vector2d ax1, ax2;
};

struct Ellipsoid3D
{
  double len1, len2, len3;
  Eigen::Vector3d ax1, ax2, ax3;
};

Ellipse2D computeCovarianceEllipse(const Eigen::Matrix2d& covariance, prob_t probability)
{
  // A matrix that has real only entries is self adjoint (Hermitian)
  // iif it is symmetric. Since this holds for covariance matrices,
  // we can use a simpler solver.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver;
  eigensolver.compute( covariance );

  Ellipse2D ret;

  ret.ax1 = eigensolver.eigenvectors().col( 0 );
  ret.ax2 = eigensolver.eigenvectors().col( 1 );

  ret.len1 = sqrt( chi2_threshold_2D[ probability ] * eigensolver.eigenvalues()( 0 ) );
  ret.len2 = sqrt( chi2_threshold_2D[ probability ] * eigensolver.eigenvalues()( 1 ) );

  return ret;
}

Ellipsoid3D computeCovarianceEllipsoid(const Eigen::Matrix3d& covariance, prob_t probability)
{
  // A matrix that has real only entries is self adjoint (Hermitian)
  // iif it is symmetric. Since this holds for covariance matrices,
  // we can use a simpler solver.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver;
  eigensolver.compute( covariance );

  Ellipsoid3D ret;

  ret.ax1 = eigensolver.eigenvectors().col( 0 );
  ret.ax2 = eigensolver.eigenvectors().col( 1 );
  ret.ax3 = eigensolver.eigenvectors().col( 2 );

  ret.len1 = sqrt( chi2_threshold_3D[ probability ] * eigensolver.eigenvalues()( 0 ) );
  ret.len2 = sqrt( chi2_threshold_3D[ probability ] * eigensolver.eigenvalues()( 1 ) );
  ret.len3 = sqrt( chi2_threshold_3D[ probability ] * eigensolver.eigenvalues()( 2 ) );

  return ret;
}
