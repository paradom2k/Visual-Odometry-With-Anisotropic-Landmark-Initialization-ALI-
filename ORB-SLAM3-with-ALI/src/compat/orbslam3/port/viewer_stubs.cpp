// Definitions for the two viewer-side classes the core references but that route_e does not compile
// (`Viewer.cc` / `MapDrawer.cc` need Pangolin and would draw).  The core touches a small surface:
//
//   Viewer      ctor, Run, RequestFinish, isFinished, RequestStop, isStopped, Release
//   MapDrawer   ctor, SetCurrentCameraPose
//
// `System` is constructed with `bUseViewer = false`, so the viewer thread body never runs; these bodies only
// have to exist and to leave the flags in a state that lets the core's shutdown loops finish.
#include "Viewer.h"
#include "MapDrawer.h"

namespace ORB_SLAM3 {

Viewer::Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking* pTracking,
               const string& strSettingPath, Settings* settings)
    : mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpTracker(pTracking),
      mbFinishRequested(false), mbFinished(true), mbStopRequested(false), mbStopped(false) {
  (void)strSettingPath;
  (void)settings;
}

void Viewer::Run() { mbFinished = true; }

void Viewer::RequestFinish() { mbFinishRequested = true; }

bool Viewer::isFinished() { return mbFinished; }

void Viewer::RequestStop() { mbStopRequested = true; }

bool Viewer::isStopped() { return mbStopped; }

void Viewer::Release() { mbFinished = true; }

MapDrawer::MapDrawer(Atlas* pAtlas, const string& strSettingPath, Settings* settings)
    : mpAtlas(pAtlas) {
  (void)strSettingPath;
  (void)settings;
}

void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f& Twc) { mCameraPose = Twc; }

}  // namespace ORB_SLAM3
