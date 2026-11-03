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

// ali-orbslam3 modification notice (2026-11-03).
// Derived from ORB-SLAM3 (GPLv3); TriangulateALI is this project's addition.
#include "GeometricTools.h"

#include "KeyFrame.h"

#include <Eigen/Cholesky>

#include <cmath>

namespace ORB_SLAM3
{

Eigen::Matrix3f GeometricTools::ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2)
{
    Sophus::SE3<float> Tc1w = pKF1->GetPose();
    Sophus::Matrix3<float> Rc1w = Tc1w.rotationMatrix();
    Sophus::SE3<float>::TranslationMember tc1w = Tc1w.translation();

    Sophus::SE3<float> Tc2w = pKF2->GetPose();
    Sophus::Matrix3<float> Rc2w = Tc2w.rotationMatrix();
    Sophus::SE3<float>::TranslationMember tc2w = Tc2w.translation();

    Sophus::Matrix3<float> Rc1c2 = Rc1w * Rc2w.transpose();
    Eigen::Vector3f tc1c2 = -Rc1c2 * tc2w + tc1w;

    Eigen::Matrix3f tc1c2x = Sophus::SO3f::hat(tc1c2);

    const Eigen::Matrix3f K1 = pKF1->mpCamera->toK_();
    const Eigen::Matrix3f K2 = pKF2->mpCamera->toK_();

    return K1.transpose().inverse() * tc1c2x * Rc1c2 * K2.inverse();
}

bool GeometricTools::Triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2,Eigen::Matrix<float,3,4> &Tc1w ,Eigen::Matrix<float,3,4> &Tc2w , Eigen::Vector3f &x3D)
{
    Eigen::Matrix4f A;
    A.block<1,4>(0,0) = x_c1(0) * Tc1w.block<1,4>(2,0) - Tc1w.block<1,4>(0,0);
    A.block<1,4>(1,0) = x_c1(1) * Tc1w.block<1,4>(2,0) - Tc1w.block<1,4>(1,0);
    A.block<1,4>(2,0) = x_c2(0) * Tc2w.block<1,4>(2,0) - Tc2w.block<1,4>(0,0);
    A.block<1,4>(3,0) = x_c2(1) * Tc2w.block<1,4>(2,0) - Tc2w.block<1,4>(1,0);

    Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);

    Eigen::Vector4f x3Dh = svd.matrixV().col(3);

    if(x3Dh(3)==0)
        return false;

    // Euclidean coordinates
    x3D = x3Dh.head(3)/x3Dh(3);

    return true;
}

bool GeometricTools::TriangulateALI(const Eigen::Vector3f &x_c1, const Eigen::Matrix2f &Cov1,
                                    const Eigen::Vector3f &x_c2, const Eigen::Matrix2f &Cov2,
                                    const Eigen::Matrix<float,3,4> &Tc1w, const Eigen::Matrix<float,3,4> &Tc2w,
                                    Eigen::Vector3f &x3D)
{
    // The isotropic estimate is the initial value and the fallback.  Triangulate() takes non-const
    // references only because upstream wrote it that way; the copies make that call possible here.
    Eigen::Vector3f x1 = x_c1;
    Eigen::Vector3f x2 = x_c2;
    Eigen::Matrix<float,3,4> T1 = Tc1w;
    Eigen::Matrix<float,3,4> T2 = Tc2w;
    if(!Triangulate(x1, x2, T1, T2, x3D))
        return false;

    // Whiten each camera once: Cov = L L^T, W = L^-1, so W Cov W^T = I.
    Eigen::LLT<Eigen::Matrix2f> llt1(Cov1);
    Eigen::LLT<Eigen::Matrix2f> llt2(Cov2);
    if(llt1.info() != Eigen::Success || llt2.info() != Eigen::Success)
        return true;
    const Eigen::Matrix2f W1 = llt1.matrixL().solve(Eigen::Matrix2f::Identity());
    const Eigen::Matrix2f W2 = llt2.matrixL().solve(Eigen::Matrix2f::Identity());

    const Eigen::Matrix<float,3,4> T[2] = {Tc1w, Tc2w};
    const Eigen::Vector3f x[2] = {x_c1, x_c2};
    const Eigen::Matrix2f W[2] = {W1, W2};

    // Whitened cost of a candidate point, and whether every observation is in front of its camera.
    auto costOf = [&](const Eigen::Vector3f &X, float *cost) -> bool {
        *cost = 0.0f;
        for(int i=0; i<2; ++i)
        {
            const Eigen::Vector3f q = T[i].block<3,3>(0,0) * X + T[i].block<3,1>(0,3);
            if(q(2) <= 1e-6f)
                return false;
            const Eigen::Vector2f p(q(0)/q(2), q(1)/q(2));
            const Eigen::Vector2f rw = W[i] * (p - x[i].head<2>());
            *cost += rw.squaredNorm();
        }
        return true;
    };

    const Eigen::Vector3f xStart = x3D;
    float startCost = 0.0f;
    if(!costOf(xStart, &startCost))
        return true;

    for(int iter=0; iter<5; ++iter)
    {
        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        Eigen::Vector3f g = Eigen::Vector3f::Zero();
        bool ok = true;
        for(int i=0; i<2 && ok; ++i)
        {
            const Eigen::Vector3f q = T[i].block<3,3>(0,0) * x3D + T[i].block<3,1>(0,3);
            if(q(2) <= 1e-6f)
            {
                ok = false;
                break;
            }
            const Eigen::Vector2f p(q(0)/q(2), q(1)/q(2));
            const Eigen::Vector2f r = p - x[i].head<2>();

            Eigen::Matrix<float,2,3> J;
            J.row(0) = (T[i].block<1,3>(0,0) - p(0) * T[i].block<1,3>(2,0)) / q(2);
            J.row(1) = (T[i].block<1,3>(1,0) - p(1) * T[i].block<1,3>(2,0)) / q(2);

            const Eigen::Matrix<float,2,3> Jw = W[i] * J;
            const Eigen::Vector2f rw = W[i] * r;
            H.noalias() += Jw.transpose() * Jw;
            g.noalias() += Jw.transpose() * rw;
        }
        if(!ok)
            break;

        Eigen::Vector3f delta = H.ldlt().solve(-g);
        if(!delta.allFinite())
            break;
        // Step limit: near-zero parallax leaves the depth direction almost unobservable, and an
        // unconstrained Gauss-Newton step there can move the point by more than its own depth.  Half the
        // current depth (camera 1) per iteration is generous for a well-conditioned pair and stops the
        // ill-conditioned ones from wandering.
        const Eigen::Vector3f q0 = T[0].block<3,3>(0,0) * x3D + T[0].block<3,1>(0,3);
        const float stepCap = 0.5f * std::max(0.1f, std::fabs(q0(2)));
        if(delta.norm() > stepCap)
            delta *= stepCap / delta.norm();
        x3D += delta;
        if(delta.norm() < 1e-6f)
            break;
    }

    // The Gauss-Newton point is kept only if it actually lowers the cost this weighting defines; otherwise
    // the isotropic SVD estimate -- which is exactly what stock ORB-SLAM3 would have used -- is returned.
    float endCost = 0.0f;
    const Eigen::Vector3f qEnd = T[0].block<3,3>(0,0) * x3D + T[0].block<3,1>(0,3);
    const float depthEnd = std::max(0.1f, std::fabs(qEnd(2)));
    if(!costOf(x3D, &endCost) || endCost > startCost || (x3D - xStart).norm() > 0.1f * depthEnd)
        x3D = xStart;
    return true;
}

} //namespace ORB_SLAM
