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

#include "projection_derivatives.hpp"

inline Eigen::Matrix3d skewSymmetricMatrix(const Eigen::Vector3d& w)
{
  Eigen::Matrix3d skew_sym;

  skew_sym(0, 0) = 0;     skew_sym(0, 1) = -w(2); skew_sym(0, 2) = w(1);
  skew_sym(1, 0) = w(2);  skew_sym(1, 1) = 0;     skew_sym(1, 2) = -w(0);
  skew_sym(2, 0) = -w(1); skew_sym(2, 1) = w(0);  skew_sym(2, 2) = 0;

  return skew_sym;
}

// d proj(Xij) / d Xij
Eigen::Matrix<double,2,3> dPXc(const double fx, const double fy, const Eigen::Vector3d& point_camera)
{
  // Jacobians wrt camera parameters
  // set d(quat-x) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
  double pcx = point_camera(0);
  double pcy = point_camera(1);
  double pcz = point_camera(2);

  // d proj / d Xcam (2x3)

  Eigen::Matrix<double,2,3> dPdXc;
  dPdXc(0, 0) = fx / pcz;  dPdXc(0, 1) = 0;         dPdXc(0, 2) = -fx * pcx / (pcz*pcz);
  dPdXc(1, 0) = 0;         dPdXc(1, 1) = fy / pcz;  dPdXc(1, 2) = -fy * pcy / (pcz*pcz);
  
  return dPdXc;
}

Eigen::Matrix<double,2,6> jacobianXj(const Eigen::Matrix<double,3,4>& camera_transform, const Eigen::Matrix3d& Kcam, const Eigen::Vector3d& point_world)
{
  const Eigen::Matrix3d R = camera_transform.block<3,3>(0,0);
  const Eigen::Vector3d t = camera_transform.col(3);

  const Eigen::Vector3d point_camera = R * point_world + t;

  const Eigen::Vector3d camera_position = -R * t;

  Eigen::Matrix<double,2,3> dPdXc = dPXc(Kcam(0,0), Kcam(1,1), point_camera);

  // d Xcam / d cam (3x6)
/*
  const Eigen::Matrix3d R = projection.block<3,3>(0,0);
  const Eigen::Vector3d point_camera = R * point_world + projection.col(3);

  Eigen::Matrix<double,3,6> dXcC = Eigen::Matrix<double,3,6>::Zero();
  dXcC.block<3,3>(0, 0) = -R;
  dXcC.block<3,3>(0, 3) = -skewSymmetricMatrix( point_camera );

  //~ double roll, pitch, yaw;
  //~ Eigen::Matrix3d dRdx; dRdx <<
   //~ 0.0, 0.0, 0.0,
   //~ cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw),    cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw), cos(roll)*cos(pitch),
   //~ -sin(roll)*sin(pitch)*cos(yaw) + cos(roll)*sin(yaw),   -sin(roll)*sin(pitch)*sin(yaw) - cos(roll)*cos(yaw), -sin(roll)*cos(pitch);

  //~ dXcC.block<3,3>(0, 3) = 
*/
  //////////////////////////////////////////////////////////////////////////////
  // Sigo la idea de g2o
/*
  const Eigen::Vector3d pwt = point_world - camera_position;

  Eigen::Matrix3d dRidx, dRidy, dRidz;
  dRidx <<
    0.0,0.0,0.0,
    0.0,0.0,2.0,
    0.0,-2.0,0.0;
  dRidy <<
    0.0,0.0,-2.0,
    0.0,0.0,0.0,
    2.0,0.0,0.0;
  dRidz <<
    0.0,2.0,0.0,
    -2.0,0.0,0.0,
    0.0,0.0,0.0;

  // for dS'*R', with dS the incremental change
  dXcC.block<3,1>(0, 3) = dRidx * R * pwt;
  dXcC.block<3,1>(0, 4) = dRidy * R * pwt;
  dXcC.block<3,1>(0, 5) = dRidz * R * pwt;

  //~ std::cout << dXcC << std::endl;
*/
  //////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////
  // Usando quaternions y chain rule
/*
  // dR(q) / dqw
  Eigen::Matrix3d Fw = 2 * skewSymmetricMatrix(qx, qy, qz) + 2 * Eigen::Matrix3d::Identity() * qw;

  // dR(q) / dqx
  Eigen::Matrix3d Fx; Fx << 
    q1,   q2,   q3,
    q2,   -q1,  -q0,
    q3,   q0,   -q1;

  // dR(q) / dqy
  Eigen::Matrix3d Fy; Fy << 
    -q2,   q1,   q0,
    q1,   q2,  q3,
    -q0,   q3,   -q2;

  // dR(q) / dqy
  Eigen::Matrix3d Fz; Fz << 
    -q3,   -q0,   q1,
    q0,   -q3,  q2,
    q1,   q2,   q3;

  // dq(u) / du1
*/
  //////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////
  // Derivando la ecuación de la matriz de rotación
  //////////////////////////////////////////////////////////////////////////////

  Eigen::Vector3d rpy = R.eulerAngles(0,1,2); 

  Eigen::Vector3d aux = point_world - camera_position;

  // d(R*v) / d(r,p,y)
  Eigen::Matrix3d dRdu;
  {
    // d(R*v) / d( rpy[0] )
    dRdu.block<3,1>(0,0) = Eigen::Vector3d(
    aux[1]*(sin( rpy[1] )*cos( rpy[0] )*cos( rpy[2] ) + sin( rpy[0] )*sin( rpy[2] )) + aux[2]*(-sin( rpy[1] )*sin( rpy[0] )*cos( rpy[2] ) + sin( rpy[2] )*cos( rpy[0] )),
    aux[1]*(sin( rpy[1] )*sin( rpy[2] )*cos( rpy[0] ) - sin( rpy[0] )*cos( rpy[2] )) + aux[2]*(-sin( rpy[1] )*sin( rpy[0] )*sin( rpy[2] ) - cos( rpy[0] )*cos( rpy[2] )),
                             aux[1]*cos( rpy[1] )*cos( rpy[0] ) - aux[2]*sin( rpy[0] )*cos( rpy[1] )
    );

    // d(R*v) / d( rpy[1] )
    dRdu.block<3,1>(0,1) = Eigen::Vector3d(
      -aux[0]*sin( rpy[1] )*cos( rpy[2] ) + aux[1]*sin( rpy[0] )*cos( rpy[1] )*cos( rpy[2] ) + aux[2]*cos( rpy[1] )*cos( rpy[0] )*cos( rpy[2] ),
      -aux[0]*sin( rpy[1] )*sin( rpy[2] ) + aux[1]*sin( rpy[0] )*sin( rpy[2] )*cos( rpy[1] ) + aux[2]*sin( rpy[2] )*cos( rpy[1] )*cos( rpy[0] ),
                -aux[0]*cos( rpy[1] ) - aux[1]*sin( rpy[1] )*sin( rpy[0] ) - aux[2]*sin( rpy[1] )*cos( rpy[0] )
    );

    // d(R*v) / d( rpy[2] )
    dRdu.block<3,1>(0,2) = Eigen::Vector3d(
      -aux[0]*sin( rpy[2] )*cos( rpy[1] ) + aux[1]*(-sin( rpy[1] )*sin( rpy[0] )*sin( rpy[2] ) - cos( rpy[0] )*cos( rpy[2] )) + aux[2]*(-sin( rpy[1] )*sin( rpy[2] )*cos( rpy[0] ) + sin( rpy[0] )*cos( rpy[2] )),
       aux[0]*cos( rpy[1] )*cos( rpy[2] ) + aux[1]*(sin( rpy[1] )*sin( rpy[0] )*cos( rpy[2] ) - sin( rpy[2] )*cos( rpy[0] )) + aux[2]*(sin( rpy[1] )*cos( rpy[0] )*cos( rpy[2] ) + sin( rpy[0] )*sin( rpy[2] )),
                                                          0
    );
  }

  Eigen::Matrix<double,3,6> dXcC = Eigen::Matrix<double,3,6>::Zero();
  dXcC.block<3,3>(0, 0) = -R; // -R
  dXcC.block<3,3>(0, 3) = dRdu;

  return dPdXc * dXcC;
}

Eigen::Matrix<double,2,3> jacobianXi(const Eigen::Matrix<double,3,4>& camera_transform, const Eigen::Matrix3d& Kcam, const Eigen::Vector3d& point_world)
{
  const Eigen::Matrix3d R = camera_transform.block<3,3>(0,0);

  const Eigen::Vector3d point_camera = R * point_world + camera_transform.col(3);

  Eigen::Matrix<double,2,3> dPdXc = dPXc(Kcam(0,0), Kcam(1,1), point_camera);

  // d Xcam / d Xworld
  
  Eigen::Matrix3d dXcXw = R;
  
  return dPdXc * dXcXw;
}
