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


#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "MapDrawer.h"
#include "System.h"
#include "ImuTypes.h"
#include "Settings.h"

#include "GeometricCamera.h"

#include <mutex>
#include <unordered_set>

namespace ORB_SLAM3
{

class Viewer;
class FrameDrawer;
class Atlas;
class LocalMapping;
class LoopClosing;
class System;
class Settings;

class Tracking
{  

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Atlas* pAtlas,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, Settings* settings, const string &_nameSeq=std::string());

    ~Tracking();

    // Parse the config file
    bool ParseCamParamFile(cv::FileStorage &fSettings);
    bool ParseORBParamFile(cv::FileStorage &fSettings);
    bool ParseIMUParamFile(cv::FileStorage &fSettings);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    Sophus::SE3f GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp, string filename);
    Sophus::SE3f GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, string filename);
    Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename);

    void GrabImuData(const IMU::Point &imuMeasurement);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);
    void SetStepByStep(bool bSet);
    bool GetStepByStep();

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);

    void UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame);
    KeyFrame* GetLastKeyFrame()
    {
        return mpLastKeyFrame;
    }

    void CreateMapInAtlas();
    //std::mutex mMutexTracks;

    //--
    void NewDataset();
    int GetNumberDataset();
    /// The number of inliers TrackLocalMap finished the last frame with.
    ///
    /// This is the number that decides the tracking verdict: monocular needs 30 to stay OK, and 50 while a
    /// relocalisation is recent, and falling below it is what turns a frame into RECENTLY_LOST and, three
    /// seconds later, throws the whole map away and starts another.  It is not the same number as the
    /// matched-map-point count that System::GetTrackedMapPoints exposes -- outlier rejection happens after
    /// matching, and it is the rejections that decide.
    int GetMatchesInliers();

    //DEBUG
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder="");
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map* pMap);

    float GetImageScale();

#ifdef REGISTER_LOOP
    void RequestStop();
    bool isStopped();
    void Release();
    bool stopRequested();
#endif

public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        RECENTLY_LOST=3,
        LOST=4,
        OK_KLT=5
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    Frame mLastFrame;

    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<Sophus::SE3f> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // frames with estimated pose
    int mTrackedFr;
    bool mbStep;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset(bool bLocMap = false);
    void ResetActiveMap(bool bLocMap = false);

    float mMeanTrack;
    bool mbInitWith3KFs;
    double t0; // time-stamp of first read frame
    double t0vis; // time-stamp of first inserted keyframe
    double t0IMU; // time-stamp of IMU initialization
    bool mFastInit = false;
    /// `IMU.resetIfNotEnoughMotion`: whether the local mapper may reset the map over a stationary stretch.
    /// Applied in `SetLocalMapper`, because the local mapper does not exist yet when the settings are read.
    bool mResetIfNotEnoughMotion = true;
    /// `IMU.firstAlignmentSeconds` / `IMU.secondAlignmentSeconds`: how much accumulated motion the inertial
    /// alignment waits for.  Applied in `SetLocalMapper` for the same reason as the flag above.
    double mViba1Motion = 5.0;
    double mViba2Motion = 15.0;
    /// Minimum mean disparity, in pixels, between the two frames the MONOCULAR initialiser is allowed to
    /// start from.  Below it the reference frame is kept and the tracker waits for more motion.
    ///
    /// Why it exists: the monocular initialiser triangulates its first map from two views, and two views
    /// taken a few centimetres apart triangulate a map whose points are metres to tens of metres out with
    /// an error of the same order.  The pose comes out usable-looking, `TrackLocalMap` then fails on the
    /// next frames, and the failure reads as a tracker problem rather than as a bad first map -- measured
    /// on 4Seasons, "Fail to track local map!" from the first frames of every map, five maps in thirty
    /// seconds.  Eight pixels at this camera (fx = 501.5, 800x400) is roughly half a metre of translation
    /// at 30 m of scene depth, i.e. a few frames of driving rather than one.
    ///
    /// The sibling project carries this as a patch gated on the JO_PARALLAX_GATE environment variable, and
    /// that patch is ported here WITHOUT the environment variable: which gates are active is an input, and
    /// inputs belong in the configuration file.  A value of zero or less disables the gate, so a run can
    /// still ask for upstream's behaviour.
    double mMinInitialParallax = 8.0;
    /// Which statistic of the matched-point disparity `mMinInitialParallax` is compared against.
    ///
    /// false = the mean, which this port has always used and what the 4Seasons results were measured
    /// with.  true = the median, which asks the question the gate was meant to ask -- did half the
    /// matched points move by this much -- and is what a scene with a large sky needs, because there the
    /// mean is dominated by points at infinity whose disparity is zero.
    bool mInitialParallaxMedian = false;

    /// Whether a monocular-inertial run may relocalise while the IMU is uninitialised.
    ///
    /// false = upstream: the RECENTLY_LOST branch sets bOK=false and waits for the LOST timeout, so a map
    /// that loses tracking before the inertial unit is up is destroyed five seconds later.  true = the
    /// inertial branch gets the non-inertial branch's Relocalization() call.  See the note on the setting.
    bool mRelocalizeBeforeImuInit = false;

    /// Whether the per-frame pose solve uses the inertial variant once the map is IMU-initialised.
    ///
    /// true = upstream.  false keeps the tracker visual and the IMU on the map side.  See the note on
    /// `imu_inertial_pose_tracking` in gio_conf.h: the two solvers disagree by whatever the map's metric
    /// scale disagrees with the IMU by, and on GREAT campus01 that is a factor of about seven.
    bool mbInertialPoseTracking = true;

    /// Whether the inertial initialisation may estimate the map scale in stereo-inertial mode.  See
    /// `LocalMapping::mbEstimateScale` and the `imu_estimate_scale` key.
    bool mEstimateImuScale = false;

    /// Minimum keyframes the inertial initialisation must see (0 = upstream's 10).
    int mInitMinKeyFrames = 0;

    /// Whether the inertial factors may shape the map (upstream true).  See `imu_use_inertial_factors`.
    bool mUseInertialFactors = true;

    /// Matched-inlier count a monocular-inertial frame must reach while the map is not yet IMU-initialised.
    ///
    /// Upstream hardcodes 50 here and 15 once the IMU is up.  The 50 assumes the initialisation produces a
    /// map dense enough that tracking it yields well over 50 inliers; on a car dataset it does not, and the
    /// count lands just under it.  Measured on SMART_PNT 20240718 with 8000 features per frame, the frame
    /// after each visual initialisation returns 21..49 inliers and dies, twelve cycles in a row, while the
    /// one initialisation that crossed 50 grew the map from 2 keyframes and 589 points to 74 and 18558 over
    /// 21.7 s.  The pure-monocular branch uses 30 for the same question, and before the inertial unit is up
    /// a monocular-inertial map IS a monocular map, which is why 30 is a defensible value.  50 is upstream.
    int mMinMatchesInliersPreInit = 50;


    vector<MapPoint*> GetLocalMapMPS();

    bool mbWriteStats;

    /// The size of the local map the last frame was tracked against.
    ///
    /// TrackLocalMap does not search the whole map -- it searches the points of the keyframes covisible
    /// with the current frame's reference keyframe.  So the number that decides whether a frame yields 30
    /// inliers is the LOCAL map's size, not the global one, and the two can differ by an order of
    /// magnitude.  When a monocular run is failing with a healthy-looking global map, this is where the
    /// shortage is.  Diagnostics only.
    void GetLocalMapSize(int* local_keyframes, int* local_points) {
      if (local_keyframes != nullptr) *local_keyframes = static_cast<int>(mvpLocalKeyFrames.size());
      if (local_points != nullptr) *local_points = static_cast<int>(mvpLocalMapPoints.size());
    }


#ifdef REGISTER_TIMES
    void LocalMapStats2File();
    void TrackStats2File();
    void PrintTimeStats();

    vector<double> vdRectStereo_ms;
    vector<double> vdResizeImage_ms;
    vector<double> vdORBExtract_ms;
    vector<double> vdStereoMatch_ms;
    vector<double> vdIMUInteg_ms;
    vector<double> vdPosePred_ms;
    vector<double> vdLMTrack_ms;
    vector<double> vdNewKF_ms;
    vector<double> vdTrackTotal_ms;
#endif

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    //void CreateNewMapPoints();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();
    bool PredictStateIMU();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // Perform preintegration from last frame
    void PreintegrateIMU();

    // Reset IMU biases and compute frame velocity
    void ResetFrameIMU();

    bool mbMapUpdated;

    // Imu preintegration from last frame
    IMU::Preintegrated *mpImuPreintegratedFromLastKF;

    // Queue of IMU measurements between frames
    std::list<IMU::Point> mlQueueImuData;

    // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
    std::vector<IMU::Point> mvImuFromLastFrame;
    std::mutex mMutexImuQueue;

    // Imu calibration parameters
    IMU::Calib *mpImuCalib;

    // Last Bias Estimation (at keyframe creation)
    IMU::Bias mLastBias;

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    bool mbReadyToInitializate;
    bool mbSetInit;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    bool bStepByStep;

    //Atlas
    Atlas* mpAtlas;

    //Calibration matrix
    cv::Mat mK;
    Eigen::Matrix3f mK_;
    cv::Mat mDistCoef;
    float mbf;
    float mImageScale;

    float mImuFreq;
    double mImuPer;
    // route E patch: may a failed TrackLocalMap discard the active map while the
    // IMU is uninitialised?  IMU.resetMapIfNotInitialized: 0 holds it off.
    bool mResetMapIfImuNotInit = true;
    bool mInsertKFsLost;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    int mnFirstImuFrameId;
    int mnFramesToResetIMU;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;
    double mTimeStampLost;
    double time_recently_lost;

    unsigned int mnFirstFrameId;
    unsigned int mnInitialFrameId;
    unsigned int mnLastInitFrameId;

    bool mbCreatedMap;

    //Motion Model
    bool mbVelocity{false};
    Sophus::SE3f mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;

    //int nMapChangeIndex;

    int mnNumDataset;

    ofstream f_track_stats;

    ofstream f_track_times;
    double mTime_PreIntIMU;
    double mTime_PosePred;
    double mTime_LocalMapTrack;
    double mTime_NewKF_Dec;

    GeometricCamera* mpCamera, *mpCamera2;

    int initID, lastID;

    Sophus::SE3f mTlr;

    void newParameterLoader(Settings* settings);

#ifdef REGISTER_LOOP
    bool Stop();

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;
#endif

public:
    cv::Mat mImRight;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
