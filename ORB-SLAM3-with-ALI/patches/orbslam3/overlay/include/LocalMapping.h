// ali-orbslam3 modification notice (2026-11-03).
// Derived from ORB-SLAM3 (GPLv3).  ALI additions: see CHANGES.md section 1.
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


#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Atlas.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "Settings.h"

#include <mutex>


namespace ORB_SLAM3
{

class System;
class Tracking;
class LoopClosing;
class Atlas;

class LocalMapping
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LocalMapping(System* pSys, Atlas* pAtlas, const float bMonocular, bool bInertial, const string &_strSeqName=std::string());

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);
    void EmptyQueue();

    // Thread Synch
    void RequestStop();
    void RequestReset();
    void RequestResetActiveMap(Map* pMap);
    // route E (long-run drift): 1 = re-estimate the metric scale and the IMU biases every few seconds
    // (ScaleRefinement), instead of only in upstream's six narrow windows early in the run.  Set from System.cc,
    // which owns the settings object.  Default 0 = upstream behaviour.
    static void SetPeriodicScaleRefinement(int enabled);

    // ALI (Anisotropic Landmark Initialization).  When enabled, a map point created by triangulation
    // is refined with the anisotropic pixel covariance of its two keypoints (GeometricTools::
    // TriangulateALI) instead of taking the unweighted midpoint estimate.  Set once at construction
    // from System.cc; default false = stock ORB-SLAM3.
    static void SetALI(bool enabled);
    // How many map points have been created through the ALI path in this process.  Diagnostics only.
    static long unsigned int ALITriangulations();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    bool IsInitializing();
    double GetCurrKFTime();
    KeyFrame* GetCurrKF();

    std::mutex mMutexImuInit;

    Eigen::MatrixXd mcovInertial;
    Eigen::Matrix3d mRwg;
    Eigen::Vector3d mbg;
    Eigen::Vector3d mba;
    double mScale;
    double mInitTime;
    double mCostTime;

    unsigned int mInitSect;
    unsigned int mIdxInit;
    unsigned int mnKFs;
    double mFirstTs;
    int mnMatchesInliers;

    // For debugging (erase in normal mode)
    int mInitFr;
    int mIdxIteration;
    string strSequence;

    bool mbNotBA1;
    bool mbNotBA2;
    bool mbBadImu;
    /// Whether a stationary stretch may destroy the active map.
    ///
    /// `Run` resets the map when the last two keyframe displacements are under 2 cm and the inertial
    /// initialisation has accumulated under 10 s of motion, on the reasoning that the IMU cannot initialise
    /// without movement.  Resetting cannot help a platform that is still not moving, and on a road vehicle it
    /// destroys a map that is tracking perfectly well while the vehicle waits.  Measured on 4Seasons
    /// recording_2020-12-22 mono-inertial, whose five stationary stretches include 50.8 s and 68.5 s: 88 map
    /// resets over the drive against 0 on a recording whose longest stop is 2.6 s.  See the setting
    /// `IMU.resetIfNotEnoughMotion`; true is upstream.
    bool mbResetIfNotEnoughMotion = true;

    /// Seconds of accumulated motion required before the first and second inertial alignment (VIBA 1 and 2).
    ///
    /// Upstream uses 5 and 15.  The alignment solves for the map scale from the IMU preintegration against
    /// the visual trajectory, and five seconds of a straight road is very little to observe a scale from:
    /// measured on 4Seasons recording_2020-10-07 mono-inertial, one drive landed on a scale four times too
    /// small at t = 16.4 s, after which the pose could not keep up with the vehicle, the inliers fell from 48
    /// to under 15 and the map was destroyed.  Six other drives of the same recording landed on a good value,
    /// so the estimate is a lottery and the observation window is the only lever on it.  See the settings
    /// `IMU.firstAlignmentSeconds` and `IMU.secondAlignmentSeconds`; 5 and 15 are upstream.
    double mViba1Motion = 5.0;
    double mViba2Motion = 15.0;

    /// Let the inertial initialisation estimate the map scale in STEREO-inertial mode too.
    ///
    /// Upstream fixes the scale vertex for stereo, on the reasoning that the stereo baseline already made the
    /// map metric.  That reasoning fails when the map's scale does not agree with the IMU's: the inertial
    /// factors then pull against every stereo measurement for the rest of the run.  GREAT campus01 is such a
    /// case -- the stereo map comes out at roughly one seventh of the ground-truth scale while the IMU is
    /// metric.  With this on, `InertialOptimization` treats the scale as free (as it always does for mono)
    /// and `InitializeIMU` applies the estimated scale to the map.  Off is upstream behaviour.
    bool mbEstimateScale = false;

    /// Minimum keyframes for the inertial initialisation, 0 = upstream (10).  See `imu_init_min_keyframes`.
    int mInitMinKeyFrames = 0;

    /// Whether the inertial initialisation (and everything that follows it) may shape the map.  See
    /// `imu_use_inertial_factors`; false keeps the map visual and metric.
    bool mbUseInertialFactors = true;

    bool mbWriteStats;

    // not consider far points (clouds)
    bool mbFarPoints;
    float mThFarPoints;

#ifdef REGISTER_TIMES
    vector<double> vdKFInsert_ms;
    vector<double> vdMPCulling_ms;
    vector<double> vdMPCreation_ms;
    vector<double> vdLBA_ms;
    vector<double> vdKFCulling_ms;
    vector<double> vdLMTotal_ms;


    vector<double> vdLBASync_ms;
    vector<double> vdKFCullingSync_ms;
    vector<int> vnLBA_edges;
    vector<int> vnLBA_KFopt;
    vector<int> vnLBA_KFfixed;
    vector<int> vnLBA_MPs;
    int nLBA_exec;
    int nLBA_abort;
#endif
protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();
    void KeyFrameCulling();

    System *mpSystem;

    bool mbMonocular;
    bool mbInertial;

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas* mpAtlas;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    void InitializeIMU(float priorG = 1e2, float priorA = 1e6, bool bFirst = false);
    void ScaleRefinement();

    bool bInitializing;

    Eigen::MatrixXd infoInertial;
    int mNumLM;
    int mNumKFCulling;

    float mTinit;

    int countRefinement;

    //DEBUG
    ofstream f_lm;

    };

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
