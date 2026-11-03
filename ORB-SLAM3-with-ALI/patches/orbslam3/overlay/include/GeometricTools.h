/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef GEOMETRIC_TOOLS_H
#define GEOMETRIC_TOOLS_H

// ali-orbslam3 modification notice (2026-11-03).
// Derived from ORB-SLAM3 (GPLv3); the additions here are the project's ALI triangulation.
// See patches/orbslam3/README.md.

#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Core>

namespace ORB_SLAM3
{

class KeyFrame;

class GeometricTools
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Compute the Fundamental matrix between KF1 and KF2
    static Eigen::Matrix3f ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    //Triangulate point with KF1 and KF2
    static bool Triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2,Eigen::Matrix<float,3,4> &Tc1w ,Eigen::Matrix<float,3,4> &Tc2w , Eigen::Vector3f &x3D);

    // ALI (Anisotropic Landmark Initialization).
    //
    // The same two-view triangulation, but each observation is weighted by a 2x2 covariance in the
    // NORMALISED image plane instead of being treated as equally precise in both directions.  The
    // midpoint/SVD estimate above is the initial value; three to five Gauss-Newton steps then minimise the
    // whitened reprojection error
    //
    //     sum_i (pi(T_i X) - x_i)^T Cov_i^-1 (pi(T_i X) - x_i),      pi(q) = (q0/q2, q1/q2)
    //
    // so a keypoint whose local structure is a line is trusted mostly across the line -- the direction in
    // which its position is actually well defined -- and much less along it.
    //
    // Cov1/Cov2 are in the normalised plane; see Frame::mvKpCov for how the pixel covariance is produced
    // and LocalMapping::CreateNewMapPoints for the pixel-to-normalised scaling.  A covariance that is not
    // positive definite, or a point that leaves the front of either camera, leaves the SVD estimate in
    // place rather than failing the triangulation: the caller's existing front-of-camera and reprojection
    // gates still apply.
    static bool TriangulateALI(const Eigen::Vector3f &x_c1, const Eigen::Matrix2f &Cov1,
                               const Eigen::Vector3f &x_c2, const Eigen::Matrix2f &Cov2,
                               const Eigen::Matrix<float,3,4> &Tc1w, const Eigen::Matrix<float,3,4> &Tc2w,
                               Eigen::Vector3f &x3D);

    template<int rows, int cols>
    static bool CheckMatrices(const cv::Mat &cvMat, const Eigen::Matrix<float,rows,cols> &eigMat) {
        const float epsilon = 1e-3;
        // std::cout << cvMat.cols - cols << cvMat.rows - rows << std::endl;
        if(rows != cvMat.rows || cols != cvMat.cols) {
            std::cout << "wrong cvmat size\n";
            return false;
        }
        for(int i = 0; i < rows; i++)
            for(int j = 0; j < cols; j++)
                if ((cvMat.at<float>(i,j) > (eigMat(i,j) + epsilon)) ||
                    (cvMat.at<float>(i,j) < (eigMat(i,j) - epsilon))){
                    std::cout << "cv mat:\n" << cvMat << std::endl;
                    std::cout << "eig mat:\n" << eigMat << std::endl;
                    return false;
                }
        return true;
    }

    template<typename T, int rows, int cols>
    static bool CheckMatrices( const Eigen::Matrix<T,rows,cols> &eigMat1, const Eigen::Matrix<T,rows,cols> &eigMat2) {
        const float epsilon = 1e-3;
        for(int i = 0; i < rows; i++)
            for(int j = 0; j < cols; j++)
                if ((eigMat1(i,j) > (eigMat2(i,j) + epsilon)) ||
                    (eigMat1(i,j) < (eigMat2(i,j) - epsilon))){
                    std::cout << "eig mat 1:\n" << eigMat1 << std::endl;
                    std::cout << "eig mat 2:\n" << eigMat2 << std::endl;
                    return false;
                }
        return true;
    }

};

}// namespace ORB_SLAM

#endif // GEOMETRIC_TOOLS_H
