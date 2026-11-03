// GIO_GVIL modification notice (2026-10-21).
// Derived from ORB-SLAM3 (GPLv3) and modified by GIO_GVIL; see patches/orbslam3/README.md.
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

#ifndef G2OTYPES_H
#define G2OTYPES_H

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_sba.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"

#include<opencv2/core/core.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <Frame.h>
#include <KeyFrame.h>
#include <MarginalizationPrior.h>   // route E, goal (F): the dense sliding-window prior carried by the edge below

#include"Converter.h"
#include <math.h>

namespace ORB_SLAM3
{

class KeyFrame;
class Frame;
class GeometricCamera;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 15, 1> Vector15d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 15, 15> Matrix15d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;

Eigen::Matrix3d ExpSO3(const double x, const double y, const double z);
Eigen::Matrix3d ExpSO3(const Eigen::Vector3d &w);

Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R);

Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d &v);
Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d &v);
Eigen::Matrix3d RightJacobianSO3(const double x, const double y, const double z);

Eigen::Matrix3d Skew(const Eigen::Vector3d &w);
Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y, const double z);

template<typename T = double>
Eigen::Matrix<T,3,3> NormalizeRotation(const Eigen::Matrix<T,3,3> &R) {
    Eigen::JacobiSVD<Eigen::Matrix<T,3,3>> svd(R,Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
}


class ImuCamPose
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImuCamPose(){}
    ImuCamPose(KeyFrame* pKF);
    ImuCamPose(Frame* pF);
    ImuCamPose(Eigen::Matrix3d &_Rwc, Eigen::Vector3d &_twc, KeyFrame* pKF);

    void SetParam(const std::vector<Eigen::Matrix3d> &_Rcw, const std::vector<Eigen::Vector3d> &_tcw, const std::vector<Eigen::Matrix3d> &_Rbc,
                  const std::vector<Eigen::Vector3d> &_tbc, const double &_bf);

    void Update(const double *pu); // update in the imu reference
    void UpdateW(const double *pu); // update in the world reference
    Eigen::Vector2d Project(const Eigen::Vector3d &Xw, int cam_idx=0) const; // Mono
    Eigen::Vector3d ProjectStereo(const Eigen::Vector3d &Xw, int cam_idx=0) const; // Stereo
    bool isDepthPositive(const Eigen::Vector3d &Xw, int cam_idx=0) const;

public:
    // For IMU
    Eigen::Matrix3d Rwb;
    Eigen::Vector3d twb;

    // For set of cameras
    std::vector<Eigen::Matrix3d> Rcw;
    std::vector<Eigen::Vector3d> tcw;
    std::vector<Eigen::Matrix3d> Rcb, Rbc;
    std::vector<Eigen::Vector3d> tcb, tbc;
    double bf;
    std::vector<GeometricCamera*> pCamera;

    // For posegraph 4DoF
    Eigen::Matrix3d Rwb0;
    Eigen::Matrix3d DR;

    int its;
};

class InvDepthPoint
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    InvDepthPoint(){}
    InvDepthPoint(double _rho, double _u, double _v, KeyFrame* pHostKF);

    void Update(const double *pu);

    double rho;
    double u, v; // they are not variables, observation in the host frame

    double fx, fy, cx, cy, bf; // from host frame

    int its;
};

// Optimizable parameters are IMU pose
class VertexPose : public g2o::BaseVertex<6,ImuCamPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPose(){}
    VertexPose(KeyFrame* pKF){
        setEstimate(ImuCamPose(pKF));
    }
    VertexPose(Frame* pF){
        setEstimate(ImuCamPose(pF));
    }


    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
        }

    virtual void oplusImpl(const double* update_){
        _estimate.Update(update_);
        updateCache();
    }
};

class VertexPose4DoF : public g2o::BaseVertex<4,ImuCamPose>
{
    // Translation and yaw are the only optimizable variables
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPose4DoF(){}
    VertexPose4DoF(KeyFrame* pKF){
        setEstimate(ImuCamPose(pKF));
    }
    VertexPose4DoF(Frame* pF){
        setEstimate(ImuCamPose(pF));
    }
    VertexPose4DoF(Eigen::Matrix3d &_Rwc, Eigen::Vector3d &_twc, KeyFrame* pKF){

        setEstimate(ImuCamPose(_Rwc, _twc, pKF));
    }

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    virtual void setToOriginImpl() {
        }

    virtual void oplusImpl(const double* update_){
        double update6DoF[6];
        update6DoF[0] = 0;
        update6DoF[1] = 0;
        update6DoF[2] = update_[0];
        update6DoF[3] = update_[1];
        update6DoF[4] = update_[2];
        update6DoF[5] = update_[3];
        _estimate.UpdateW(update6DoF);
        updateCache();
    }
};

class VertexVelocity : public g2o::BaseVertex<3,Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexVelocity(){}
    VertexVelocity(KeyFrame* pKF);
    VertexVelocity(Frame* pF);

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    virtual void setToOriginImpl() {
        }

    virtual void oplusImpl(const double* update_){
        Eigen::Vector3d uv;
        uv << update_[0], update_[1], update_[2];
        setEstimate(estimate()+uv);
    }
};

class VertexGyroBias : public g2o::BaseVertex<3,Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexGyroBias(){}
    VertexGyroBias(KeyFrame* pKF);
    VertexGyroBias(Frame* pF);

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    virtual void setToOriginImpl() {
        }

    virtual void oplusImpl(const double* update_){
        Eigen::Vector3d ubg;
        ubg << update_[0], update_[1], update_[2];
        setEstimate(estimate()+ubg);
    }
};


class VertexAccBias : public g2o::BaseVertex<3,Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexAccBias(){}
    VertexAccBias(KeyFrame* pKF);
    VertexAccBias(Frame* pF);

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    virtual void setToOriginImpl() {
        }

    virtual void oplusImpl(const double* update_){
        Eigen::Vector3d uba;
        uba << update_[0], update_[1], update_[2];
        setEstimate(estimate()+uba);
    }
};


// Gravity direction vertex
class GDirection
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GDirection(){}
    GDirection(Eigen::Matrix3d pRwg): Rwg(pRwg){}

    void Update(const double *pu)
    {
        Rwg=Rwg*ExpSO3(pu[0],pu[1],0.0);
    }

    Eigen::Matrix3d Rwg, Rgw;

    int its;
};

class VertexGDir : public g2o::BaseVertex<2,GDirection>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexGDir(){}
    VertexGDir(Eigen::Matrix3d pRwg){
        setEstimate(GDirection(pRwg));
    }

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    virtual void setToOriginImpl() {
        }

    virtual void oplusImpl(const double* update_){
        _estimate.Update(update_);
        updateCache();
    }
};

// scale vertex
class VertexScale : public g2o::BaseVertex<1,double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexScale(){
        setEstimate(1.0);
    }
    VertexScale(double ps){
        setEstimate(ps);
    }

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    virtual void setToOriginImpl(){
        setEstimate(1.0);
    }

    virtual void oplusImpl(const double *update_){
        setEstimate(estimate()*exp(*update_));
    }
};


// Inverse depth point (just one parameter, inverse depth at the host frame)
class VertexInvDepth : public g2o::BaseVertex<1,InvDepthPoint>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexInvDepth(){}
    VertexInvDepth(double invDepth, double u, double v, KeyFrame* pHostKF){
        setEstimate(InvDepthPoint(invDepth, u, v, pHostKF));
    }

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    virtual void setToOriginImpl() {
        }

    virtual void oplusImpl(const double* update_){
        _estimate.Update(update_);
        updateCache();
    }
};

class EdgeMono : public g2o::BaseBinaryEdge<2,Eigen::Vector2d,g2o::VertexSBAPointXYZ,VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeMono(int cam_idx_=0): cam_idx(cam_idx_){
    }

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    void computeError(){
        const g2o::VertexSBAPointXYZ* VPoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
        const Eigen::Vector2d obs(_measurement);
        _error = obs - VPose->estimate().Project(VPoint->estimate(),cam_idx);
    }


    virtual void linearizeOplus();

    bool isDepthPositive()
    {
        const g2o::VertexSBAPointXYZ* VPoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
        return VPose->estimate().isDepthPositive(VPoint->estimate(),cam_idx);
    }

    Eigen::Matrix<double,2,9> GetJacobian(){
        linearizeOplus();
        Eigen::Matrix<double,2,9> J;
        J.block<2,3>(0,0) = _jacobianOplusXi;
        J.block<2,6>(0,3) = _jacobianOplusXj;
        return J;
    }

    Eigen::Matrix<double,9,9> GetHessian(){
        linearizeOplus();
        Eigen::Matrix<double,2,9> J;
        J.block<2,3>(0,0) = _jacobianOplusXi;
        J.block<2,6>(0,3) = _jacobianOplusXj;
        return J.transpose()*information()*J;
    }

public:
    const int cam_idx;
};

class EdgeMonoOnlyPose : public g2o::BaseUnaryEdge<2,Eigen::Vector2d,VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeMonoOnlyPose(const Eigen::Vector3f &Xw_, int cam_idx_=0):Xw(Xw_.cast<double>()),
        cam_idx(cam_idx_){}

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    void computeError(){
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);
        const Eigen::Vector2d obs(_measurement);
        _error = obs - VPose->estimate().Project(Xw,cam_idx);
    }

    virtual void linearizeOplus();

    bool isDepthPositive()
    {
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);
        return VPose->estimate().isDepthPositive(Xw,cam_idx);
    }

    Eigen::Matrix<double,6,6> GetHessian(){
        linearizeOplus();
        return _jacobianOplusXi.transpose()*information()*_jacobianOplusXi;
    }

public:
    const Eigen::Vector3d Xw;
    const int cam_idx;
};

class EdgeStereo : public g2o::BaseBinaryEdge<3,Eigen::Vector3d,g2o::VertexSBAPointXYZ,VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeStereo(int cam_idx_=0): cam_idx(cam_idx_){}

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    void computeError(){
        const g2o::VertexSBAPointXYZ* VPoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
        const Eigen::Vector3d obs(_measurement);
        _error = obs - VPose->estimate().ProjectStereo(VPoint->estimate(),cam_idx);
    }


    virtual void linearizeOplus();

    Eigen::Matrix<double,3,9> GetJacobian(){
        linearizeOplus();
        Eigen::Matrix<double,3,9> J;
        J.block<3,3>(0,0) = _jacobianOplusXi;
        J.block<3,6>(0,3) = _jacobianOplusXj;
        return J;
    }

    Eigen::Matrix<double,9,9> GetHessian(){
        linearizeOplus();
        Eigen::Matrix<double,3,9> J;
        J.block<3,3>(0,0) = _jacobianOplusXi;
        J.block<3,6>(0,3) = _jacobianOplusXj;
        return J.transpose()*information()*J;
    }

public:
    const int cam_idx;
};


class EdgeStereoOnlyPose : public g2o::BaseUnaryEdge<3,Eigen::Vector3d,VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeStereoOnlyPose(const Eigen::Vector3f &Xw_, int cam_idx_=0):
        Xw(Xw_.cast<double>()), cam_idx(cam_idx_){}

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    void computeError(){
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);
        const Eigen::Vector3d obs(_measurement);
        _error = obs - VPose->estimate().ProjectStereo(Xw, cam_idx);
    }

    virtual void linearizeOplus();

    Eigen::Matrix<double,6,6> GetHessian(){
        linearizeOplus();
        return _jacobianOplusXi.transpose()*information()*_jacobianOplusXi;
    }

public:
    const Eigen::Vector3d Xw; // 3D point coordinates
    const int cam_idx;
};

class EdgeInertial : public g2o::BaseMultiEdge<9,Vector9d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeInertial(IMU::Preintegrated* pInt);

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    void computeError();
    virtual void linearizeOplus();

    /// route E, goal (F): the factor's residual Jacobian, 9 x 24, in the same column layout as
    /// GetHessian()/linearizeOplus() below.  It is computed from the edge's own state and members and never goes
    /// through g2o's `_jacobianOplus` Maps: those are Eigen Maps onto the optimizer's Jacobian workspace, they
    /// are bound only inside initializeOptimization(), and the marginalisation reads this factor before that.
    Eigen::Matrix<double, 9, 24> EstimateJacobian() const;

    /// Per-vertex block layout of EstimateJacobian(): pose(6) vel(3) bg(3) ba(3) of KF i, then pose(6) vel(3).
    static const int kJacobianColumnOffset[6];
    static const int kJacobianBlockWidth[6];

    Eigen::Matrix<double,24,24> GetHessian(){
        linearizeOplus();
        Eigen::Matrix<double,9,24> J;
        J.block<9,6>(0,0) = _jacobianOplus[0];
        J.block<9,3>(0,6) = _jacobianOplus[1];
        J.block<9,3>(0,9) = _jacobianOplus[2];
        J.block<9,3>(0,12) = _jacobianOplus[3];
        J.block<9,6>(0,15) = _jacobianOplus[4];
        J.block<9,3>(0,21) = _jacobianOplus[5];
        return J.transpose()*information()*J;
    }

    /// The factor's own Jacobian for one vertex, in the same order the vertices were set.
    Eigen::MatrixXd Jacobian(int i) const {
        const Eigen::Matrix<double, 9, 24> J = EstimateJacobian();
        return J.block(0, kJacobianColumnOffset[i], 9, kJacobianBlockWidth[i]);
    }

    Eigen::Matrix<double,18,18> GetHessianNoPose1(){
        linearizeOplus();
        Eigen::Matrix<double,9,18> J;
        J.block<9,3>(0,0) = _jacobianOplus[1];
        J.block<9,3>(0,3) = _jacobianOplus[2];
        J.block<9,3>(0,6) = _jacobianOplus[3];
        J.block<9,6>(0,9) = _jacobianOplus[4];
        J.block<9,3>(0,15) = _jacobianOplus[5];
        return J.transpose()*information()*J;
    }

    Eigen::Matrix<double,9,9> GetHessian2(){
        linearizeOplus();
        Eigen::Matrix<double,9,9> J;
        J.block<9,6>(0,0) = _jacobianOplus[4];
        J.block<9,3>(0,6) = _jacobianOplus[5];
        return J.transpose()*information()*J;
    }

    const Eigen::Matrix3d JRg, JVg, JPg;
    const Eigen::Matrix3d JVa, JPa;
    IMU::Preintegrated* mpInt;
    const double dt;
    Eigen::Vector3d g;
};


// Edge inertial whre gravity is included as optimizable variable and it is not supposed to be pointing in -z axis, as well as scale
class EdgeInertialGS : public g2o::BaseMultiEdge<9,Vector9d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // EdgeInertialGS(IMU::Preintegrated* pInt);
    EdgeInertialGS(IMU::Preintegrated* pInt);

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    void computeError();
    virtual void linearizeOplus();

    const Eigen::Matrix3d JRg, JVg, JPg;
    const Eigen::Matrix3d JVa, JPa;
    IMU::Preintegrated* mpInt;
    const double dt;
    Eigen::Vector3d g, gI;

    Eigen::Matrix<double,27,27> GetHessian(){
        linearizeOplus();
        Eigen::Matrix<double,9,27> J;
        J.block<9,6>(0,0) = _jacobianOplus[0];
        J.block<9,3>(0,6) = _jacobianOplus[1];
        J.block<9,3>(0,9) = _jacobianOplus[2];
        J.block<9,3>(0,12) = _jacobianOplus[3];
        J.block<9,6>(0,15) = _jacobianOplus[4];
        J.block<9,3>(0,21) = _jacobianOplus[5];
        J.block<9,2>(0,24) = _jacobianOplus[6];
        J.block<9,1>(0,26) = _jacobianOplus[7];
        return J.transpose()*information()*J;
    }

    Eigen::Matrix<double,27,27> GetHessian2(){
        linearizeOplus();
        Eigen::Matrix<double,9,27> J;
        J.block<9,3>(0,0) = _jacobianOplus[2];
        J.block<9,3>(0,3) = _jacobianOplus[3];
        J.block<9,2>(0,6) = _jacobianOplus[6];
        J.block<9,1>(0,8) = _jacobianOplus[7];
        J.block<9,3>(0,9) = _jacobianOplus[1];
        J.block<9,3>(0,12) = _jacobianOplus[5];
        J.block<9,6>(0,15) = _jacobianOplus[0];
        J.block<9,6>(0,21) = _jacobianOplus[4];
        return J.transpose()*information()*J;
    }

    Eigen::Matrix<double,9,9> GetHessian3(){
        linearizeOplus();
        Eigen::Matrix<double,9,9> J;
        J.block<9,3>(0,0) = _jacobianOplus[2];
        J.block<9,3>(0,3) = _jacobianOplus[3];
        J.block<9,2>(0,6) = _jacobianOplus[6];
        J.block<9,1>(0,8) = _jacobianOplus[7];
        return J.transpose()*information()*J;
    }



    Eigen::Matrix<double,1,1> GetHessianScale(){
        linearizeOplus();
        Eigen::Matrix<double,9,1> J = _jacobianOplus[7];
        return J.transpose()*information()*J;
    }

    Eigen::Matrix<double,3,3> GetHessianBiasGyro(){
        linearizeOplus();
        Eigen::Matrix<double,9,3> J = _jacobianOplus[2];
        return J.transpose()*information()*J;
    }

    Eigen::Matrix<double,3,3> GetHessianBiasAcc(){
        linearizeOplus();
        Eigen::Matrix<double,9,3> J = _jacobianOplus[3];
        return J.transpose()*information()*J;
    }

    Eigen::Matrix<double,2,2> GetHessianGDir(){
        linearizeOplus();
        Eigen::Matrix<double,9,2> J = _jacobianOplus[6];
        return J.transpose()*information()*J;
    }
};



class EdgeGyroRW : public g2o::BaseBinaryEdge<3,Eigen::Vector3d,VertexGyroBias,VertexGyroBias>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeGyroRW(){}

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    void computeError(){
        const VertexGyroBias* VG1= static_cast<const VertexGyroBias*>(_vertices[0]);
        const VertexGyroBias* VG2= static_cast<const VertexGyroBias*>(_vertices[1]);
        _error = VG2->estimate()-VG1->estimate();
    }

    virtual void linearizeOplus(){
        _jacobianOplusXi = -Eigen::Matrix3d::Identity();
        _jacobianOplusXj.setIdentity();
    }

    Eigen::Matrix<double,6,6> GetHessian(){
        linearizeOplus();
        Eigen::Matrix<double,3,6> J;
        J.block<3,3>(0,0) = _jacobianOplusXi;
        J.block<3,3>(0,3) = _jacobianOplusXj;
        return J.transpose()*information()*J;
    }

    /// route E, goal (F): the marginalisation needs this factor's own Jacobian to rebuild the gradient term.
    ///
    /// Computed here from the edge's own definition, deliberately **not** by calling linearizeOplus(): g2o's
    /// `_jacobianOplus*` members are Eigen Maps onto the optimizer's Jacobian workspace, they are only bound
    /// inside `initializeOptimization()`, and until then the edge's own linearizeOplus() writes through a null
    /// pointer.  The marginalisation runs on edges whose maps are not bound, so it must not go through them.
    /// which: 0 = the first vertex, 1 = the second.
    Eigen::MatrixXd Jacobian(int which) const {
        const Eigen::Matrix3d Xi = -Eigen::Matrix3d::Identity();
        const Eigen::Matrix3d Xj = Eigen::Matrix3d::Identity();
        return which == 0 ? Eigen::MatrixXd(Xi) : Eigen::MatrixXd(Xj);
    }

    Eigen::Matrix3d GetHessian2(){
        linearizeOplus();
        return _jacobianOplusXj.transpose()*information()*_jacobianOplusXj;
    }
};


class EdgeAccRW : public g2o::BaseBinaryEdge<3,Eigen::Vector3d,VertexAccBias,VertexAccBias>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeAccRW(){}

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    void computeError(){
        const VertexAccBias* VA1= static_cast<const VertexAccBias*>(_vertices[0]);
        const VertexAccBias* VA2= static_cast<const VertexAccBias*>(_vertices[1]);
        _error = VA2->estimate()-VA1->estimate();
    }

    virtual void linearizeOplus(){
        _jacobianOplusXi = -Eigen::Matrix3d::Identity();
        _jacobianOplusXj.setIdentity();
    }

    Eigen::Matrix<double,6,6> GetHessian(){
        linearizeOplus();
        Eigen::Matrix<double,3,6> J;
        J.block<3,3>(0,0) = _jacobianOplusXi;
        J.block<3,3>(0,3) = _jacobianOplusXj;
        return J.transpose()*information()*J;
    }

    /// route E, goal (F): the marginalisation needs this factor's own Jacobian to rebuild the gradient term.
    ///
    /// Computed here from the edge's own definition, deliberately **not** by calling linearizeOplus(): g2o's
    /// `_jacobianOplus*` members are Eigen Maps onto the optimizer's Jacobian workspace, they are only bound
    /// inside `initializeOptimization()`, and until then the edge's own linearizeOplus() writes through a null
    /// pointer.  The marginalisation runs on edges whose maps are not bound, so it must not go through them.
    /// which: 0 = the first vertex, 1 = the second.
    Eigen::MatrixXd Jacobian(int which) const {
        const Eigen::Matrix3d Xi = -Eigen::Matrix3d::Identity();
        const Eigen::Matrix3d Xj = Eigen::Matrix3d::Identity();
        return which == 0 ? Eigen::MatrixXd(Xi) : Eigen::MatrixXd(Xj);
    }

    Eigen::Matrix3d GetHessian2(){
        linearizeOplus();
        return _jacobianOplusXj.transpose()*information()*_jacobianOplusXj;
    }
};

class ConstraintPoseImu
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConstraintPoseImu(const Eigen::Matrix3d &Rwb_, const Eigen::Vector3d &twb_, const Eigen::Vector3d &vwb_,
                       const Eigen::Vector3d &bg_, const Eigen::Vector3d &ba_, const Matrix15d &H_):
                       Rwb(Rwb_), twb(twb_), vwb(vwb_), bg(bg_), ba(ba_), H(H_)
    {
        H = (H+H)/2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,15,15> > es(H);
        Eigen::Matrix<double,15,1> eigs = es.eigenvalues();
        for(int i=0;i<15;i++)
            if(eigs[i]<1e-12)
                eigs[i]=0;
        H = es.eigenvectors()*eigs.asDiagonal()*es.eigenvectors().transpose();
    }

    Eigen::Matrix3d Rwb;
    Eigen::Vector3d twb;
    Eigen::Vector3d vwb;
    Eigen::Vector3d bg;
    Eigen::Vector3d ba;
    Matrix15d H;
};

class EdgePriorPoseImu : public g2o::BaseMultiEdge<15,Vector15d>
{
public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgePriorPoseImu(ConstraintPoseImu* c);

        virtual bool read(std::istream& is){return false;}
        virtual bool write(std::ostream& os) const{return false;}

        void computeError();
        virtual void linearizeOplus();

        Eigen::Matrix<double,15,15> GetHessian(){
            linearizeOplus();
            Eigen::Matrix<double,15,15> J;
            J.block<15,6>(0,0) = _jacobianOplus[0];
            J.block<15,3>(0,6) = _jacobianOplus[1];
            J.block<15,3>(0,9) = _jacobianOplus[2];
            J.block<15,3>(0,12) = _jacobianOplus[3];
            return J.transpose()*information()*J;
        }

    /// route E, goal (F): see EdgeInertial::Jacobian.  The residual is
    /// (Log(R*Rc'), R'*(t-tc'), v-vc, bg-bgc, ba-bac), so in the perturbation coordinates the Jacobian is the
    /// identity on the four blocks (6 + 3 + 3 + 3 = 15 columns).
    Eigen::MatrixXd Jacobian(int) const {
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(15, 15);
        J.block<6, 6>(0, 0).setIdentity();
        J.block<3, 3>(6, 6).setIdentity();
        J.block<3, 3>(9, 9).setIdentity();
        J.block<3, 3>(12, 12).setIdentity();
        return J;
    }

        Eigen::Matrix<double,9,9> GetHessianNoPose(){
            linearizeOplus();
            Eigen::Matrix<double,15,9> J;
            J.block<15,3>(0,0) = _jacobianOplus[1];
            J.block<15,3>(0,3) = _jacobianOplus[2];
            J.block<15,3>(0,6) = _jacobianOplus[3];
            return J.transpose()*information()*J;
        }
        Eigen::Matrix3d Rwb;
        Eigen::Vector3d twb, vwb;
        Eigen::Vector3d bg, ba;
};

// Priors for biases
class EdgePriorAcc : public g2o::BaseUnaryEdge<3,Eigen::Vector3d,VertexAccBias>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgePriorAcc(const Eigen::Vector3f &bprior_):bprior(bprior_.cast<double>()){}

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    void computeError(){
        const VertexAccBias* VA = static_cast<const VertexAccBias*>(_vertices[0]);
        _error = bprior - VA->estimate();
    }
    virtual void linearizeOplus();

    Eigen::Matrix<double,3,3> GetHessian(){
        linearizeOplus();
        return _jacobianOplusXi.transpose()*information()*_jacobianOplusXi;
    }

    /// route E, goal (F): see EdgeInertial::Jacobian (computed from the edge's own state, never through the
    /// g2o workspace Maps, which are unbound outside the optimization loop).
    Eigen::MatrixXd Jacobian(int) const { return Eigen::MatrixXd(-Eigen::Matrix3d::Identity()); }

    const Eigen::Vector3d bprior;
};

class EdgePriorGyro : public g2o::BaseUnaryEdge<3,Eigen::Vector3d,VertexGyroBias>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgePriorGyro(const Eigen::Vector3f &bprior_):bprior(bprior_.cast<double>()){}

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    void computeError(){
        const VertexGyroBias* VG = static_cast<const VertexGyroBias*>(_vertices[0]);
        _error = bprior - VG->estimate();
    }
    virtual void linearizeOplus();

    Eigen::Matrix<double,3,3> GetHessian(){
        linearizeOplus();
        return _jacobianOplusXi.transpose()*information()*_jacobianOplusXi;
    }

    /// route E, goal (F): see EdgeInertial::Jacobian (computed from the edge's own state, never through the
    /// g2o workspace Maps, which are unbound outside the optimization loop).
    Eigen::MatrixXd Jacobian(int) const { return Eigen::MatrixXd(-Eigen::Matrix3d::Identity()); }

    const Eigen::Vector3d bprior;
};

/// A prior on one body pose in the local tangent: reference = (LogSO3(Rwb), twb).  Used by the tight VIO
/// as the per-frame VO absolute-pose anchor, which the Schur prior alone cannot supply (the marginalized
/// window is otherwise a free-running VIO and drifts over a 60 s drive).
class EdgePriorPose : public g2o::BaseUnaryEdge<6,Eigen::Matrix<double,6,1>,VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit EdgePriorPose(const Eigen::Matrix<double,6,1> &reference): reference_(reference) {}

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    void computeError(){
        const VertexPose* VP = static_cast<const VertexPose*>(_vertices[0]);
        const Eigen::Vector3d r_ref = reference_.head<3>();
        const Eigen::Vector3d t_ref = reference_.tail<3>();
        const Eigen::Vector3d er = LogSO3(ExpSO3(r_ref).transpose() * VP->estimate().Rwb);
        const Eigen::Vector3d et = ExpSO3(r_ref).transpose() * (VP->estimate().twb - t_ref);
        _error << er, et;
    }

    /// The same manifold derivative as EdgeMarginalizationPrior: rotation J_r^-1(er), translation R_ref' R.
    Eigen::Matrix<double,6,6> computeJacobian() const {
        const VertexPose* VP = static_cast<const VertexPose*>(_vertices[0]);
        const Eigen::Vector3d r_ref = reference_.head<3>();
        const Eigen::Matrix3d R_rel = ExpSO3(r_ref).transpose() * VP->estimate().Rwb;
        const Eigen::Vector3d er = LogSO3(R_rel);
        Eigen::Matrix<double,6,6> J = Eigen::Matrix<double,6,6>::Identity();
        J.block<3,3>(0,0) = InverseRightJacobianSO3(er);
        J.block<3,3>(3,3) = R_rel;
        return J;
    }

    virtual void linearizeOplus(){ _jacobianOplusXi = computeJacobian(); }

    /// route E, goal (F): see EdgeInertial::Jacobian.
    Eigen::MatrixXd Jacobian(int) const { return computeJacobian(); }

    const Eigen::Matrix<double,6,1>& reference() const { return reference_; }

private:
    Eigen::Matrix<double,6,1> reference_;
};

// route E (long-run drift): a prior on one keyframe's velocity, in world coordinates.
//
// Why this exists: the full-drive measurements show the trajectory staying well scaled to ~280 s and then
// diverging in the final low-speed section, where the reference brakes to 0.14 m/s and the estimate accumulates
// hundreds of metres of spurious travel.  That regime is exactly where the accelerometer bias and the gravity
// direction stop being separable, so an inertial constraint cannot fix it.  The visual bundle adjustment still
// tracks the pose there, so the displacement between neighbouring keyframes is a reliable independent estimate of
// the velocity -- and this edge anchors the velocity state to it.  The value is supplied per keyframe by the
// caller; the switch that enables it defaults to 0.
class EdgePriorVelocity : public g2o::BaseUnaryEdge<3,Eigen::Vector3d,VertexVelocity>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgePriorVelocity(const Eigen::Vector3d &vprior_):vprior(vprior_){}

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    void computeError(){
        const VertexVelocity* VV = static_cast<const VertexVelocity*>(_vertices[0]);
        _error = VV->estimate() - vprior;
    }
    virtual void linearizeOplus(){
        _jacobianOplusXi = Eigen::Matrix3d::Identity();
    }

    /// route E, goal (F): see EdgeInertial::Jacobian (computed from the edge's own state, never through the
    /// g2o workspace Maps, which are unbound outside the optimization loop).
    Eigen::MatrixXd Jacobian(int) const { return Eigen::MatrixXd(Eigen::Matrix3d::Identity()); }

    const Eigen::Vector3d vprior;
};

// route E, goal (F): the dense sliding-window prior produced by MarginalizationInfo and re-applied here.
//
// Why not EdgePriorPoseImu (which the plan first proposed): that edge is unary on exactly one keyframe's
// pose+velocity+biases (15x15).  Schur-complementing the oldest keyframe out of a window in which the
// random-walk factors (EdgeGyroRW/EdgeAccRW) couple consecutive keyframes leaves the surviving keyframes
// coupled, so the prior is dense over 15*(number of surviving keyframes) dimensions and its off-diagonal blocks
// are real information.  A 15x15 unary edge per keyframe could only carry the block-diagonal part.
//
// The edge dimension must be a compile-time maximum because g2o sizes the Jacobian workspace from
// Edge::dimension() (JacobianWorkspace::updateSize, Thirdparty/g2o/g2o/core/jacobian_workspace.cpp:60), and a
// Dynamic edge reports -1, which disables that workspace.  Only the first usedDim() rows/columns are touched;
// the rest of _error and _information stay zero.
//
// kMaxPriorStates is the number of keyframes the prior can span (15 local coordinates each: pose 6 + velocity 3
// + gyro bias 3 + accel bias 3).  It is bounded from above by Eigen's stack allocation limit of 128 KB for a
// fixed-size matrix: kMaxPriorDim * kMaxPriorDim * 8 <= 131072 gives kMaxPriorDim <= 128.  Optimizer clips the
// window size to this same bound, so a prior can never be built larger than the edge can carry.
static const int kMaxPriorDim = 126;                              // 8 keyframes x 15 coordinates, 8 bytes each

/// Dense prior on a set of window states: the exact quadratic form the eliminated measurements impose.
///
/// computeError() evaluates  e = residual - (local(x) - local(reference))  at zero cost, i.e. the prior is
/// exactly quadratic and does not spend an extra linearisation of the eliminated factors.  The Jacobian with
/// respect to the pose local coordinates is the identity, which is the same convention EdgePriorPoseImu uses at
/// its linearisation point (see its linearizeOplus in src/G2oTypes.cc).
class EdgeMarginalizationPrior : public g2o::BaseMultiEdge<kMaxPriorDim,Eigen::Matrix<double,kMaxPriorDim,1> >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// `stateLocalSizes` runs parallel to the vertices that are set with setVertex(), in the same order, and gives
    /// the local size of each (6 for a pose, 3 for velocity/bias).  `info` is the prior information, `shift` the
    /// edge's centre relative to `reference` (the prior's residual is the NEGATIVE gradient, so shift =
    /// +info^+ * residual), and `reference` the state values at the linearisation point.  The cost is
    /// 0.5*(delta-shift)' * info * (delta-shift), delta = local(x)-reference.
    EdgeMarginalizationPrior(const Eigen::MatrixXd& info, const Eigen::VectorXd& shift,
                             const Eigen::VectorXd& reference, const std::vector<int>& stateLocalSizes)
    {
        _usedDim = 0;
        for (std::size_t i = 0; i < stateLocalSizes.size(); ++i) _usedDim += stateLocalSizes[i];
        _localSizes = stateLocalSizes;

        resize(stateLocalSizes.size());   // BaseEdge<kMaxPriorDim,...> sizes _error at compile time

        _error.setZero();
        _shift.setZero();
        _reference.setZero();
        _information.setZero(kMaxPriorDim, kMaxPriorDim);
        const int copy = std::min(_usedDim, kMaxPriorDim);
        if (info.rows() >= copy && info.cols() >= copy && shift.size() >= copy && reference.size() >= copy) {
            _information.block(0, 0, copy, copy) = info.block(0, 0, copy, copy);
            _shift.head(copy) = shift.head(copy);
            _reference.head(copy) = reference.head(copy);
        }
    }

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    /// Local size of one attached state, in setVertex() order.
    int stateLocalSize(int i) const { return _localSizes[i]; }
    int usedDim() const { return _usedDim; }
    /// The analytic Jacobian block of one vertex, for the unit test and the run log.
    Eigen::MatrixXd jacobian(int i) const { return _jacobianOplus[i]; }

    void computeError();
    virtual void linearizeOplus();

    /// The analytic Jacobian written to a local matrix (rows: prior error, columns: the concatenated local
    /// updates in setVertex() order).  linearizeOplus() copies it into g2o's workspace Maps; tests and the
    /// marginalisation read it directly because those Maps are only bound inside initializeOptimization().
    Eigen::MatrixXd computeJacobian() const;

private:
    std::vector<int> _localSizes;
    int _usedDim = 0;
    Eigen::Matrix<double,kMaxPriorDim,1> _shift = Eigen::Matrix<double,kMaxPriorDim,1>::Zero();
    Eigen::Matrix<double,kMaxPriorDim,1> _reference = Eigen::Matrix<double,kMaxPriorDim,1>::Zero();
};

class Edge4DoF : public g2o::BaseBinaryEdge<6,Vector6d,VertexPose4DoF,VertexPose4DoF>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Edge4DoF(const Eigen::Matrix4d &deltaT){
        dTij = deltaT;
        dRij = deltaT.block<3,3>(0,0);
        dtij = deltaT.block<3,1>(0,3);
    }

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    void computeError(){
        const VertexPose4DoF* VPi = static_cast<const VertexPose4DoF*>(_vertices[0]);
        const VertexPose4DoF* VPj = static_cast<const VertexPose4DoF*>(_vertices[1]);
        _error << LogSO3(VPi->estimate().Rcw[0]*VPj->estimate().Rcw[0].transpose()*dRij.transpose()),
                 VPi->estimate().Rcw[0]*(-VPj->estimate().Rcw[0].transpose()*VPj->estimate().tcw[0])+VPi->estimate().tcw[0] - dtij;
    }

    // virtual void linearizeOplus(); // numerical implementation

    Eigen::Matrix4d dTij;
    Eigen::Matrix3d dRij;
    Eigen::Vector3d dtij;
};

} //namespace ORB_SLAM2

#endif // G2OTYPES_H
