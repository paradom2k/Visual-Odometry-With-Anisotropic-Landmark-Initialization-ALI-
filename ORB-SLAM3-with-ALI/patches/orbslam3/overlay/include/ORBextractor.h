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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>


namespace ORB_SLAM3
{

// route E (corner frontend A/B): this was a file-static function inside ORBextractor.cc.  It is declared here so
// that Frame::ExtractORB can compute descriptors on externally supplied keypoints with this tree's own descriptor
// code, rather than reaching for OpenCV's ORB and getting different descriptor conventions.  Announcing it changes
// nothing for the existing call site inside ORBextractor.cc.
void computeDescriptors(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                        const std::vector<cv::Point>& pattern);

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    int operator()( cv::InputArray _image, cv::InputArray _mask,
                    std::vector<cv::KeyPoint>& _keypoints,
                    cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;

    // route E (corner frontend A/B): exposes the descriptor sampling pattern that the constructor builds, so that
    // computeDescriptors can be reused on externally supplied keypoints.  A narrow accessor rather than making
    // `pattern` public, because nothing else outside the class has any business reading it.
    const std::vector<cv::Point>& DescriptorPattern() const { return pattern; }

    // route E (corner frontend A/B): the intensity-centroid orientation for one externally supplied keypoint, using
    // the same umax table and patch radius the extractor uses internally.  Needed because ORB's BRIEF pattern is
    // rotated: a keypoint supplied with angle = -1 gets descriptors sampled at the wrong rotation.  Returned rather
    // than written into a keypoint so that the caller can decide what to do with it.
    float ComputeOrientation(const cv::Mat& image, const cv::Point2f& point) const;

    // route E (corner frontend A/B): external keypoints for the next extraction, or null to detect normally.
    //
    // They are held on the extractor rather than passed in from outside because the substitution happens inside
    // operator() -- see the comment there.  Keeping them here means the caller sets them on the same object that
    // will consume them, with no file-scope shared state.
    void SetExternalKeypoints(const std::vector<cv::KeyPoint>* keypoints) { mpExternalKeypoints = keypoints; }
    void ClearExternalKeypoints() { mpExternalKeypoints = nullptr; }

    // route E (corner frontend A/B): compute orientations and descriptors for externally supplied keypoints.
    //
    // This lives here rather than in the caller because the per-level coordinate convention is this class's, and it
    // is easy to get wrong from outside: `computeDescriptors` must be given the pyramid level, in which a keypoint's
    // coordinate is `pt / scale + EDGE_THRESHOLD` (that is: the pyramid is a bordered canvas and the keypoint is
    // described in canvas coordinates), and the coordinate must be divided back before the descriptors are returned
    // to a caller that thinks in image coordinates.  The keypoints are taken and left in image coordinates.
    //
    // NOTE: this builds its own pyramid.  Prefer leaving the keypoints on the extractor and letting operator() do the
    // work, because a caller that also runs the detector pays for the pyramid twice per frame on a reused extractor.
    void ComputeDescriptorsFor(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                               cv::Mat& descriptors);

protected:

    void ComputePyramid(cv::Mat image);
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::Point> pattern;

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    /// route E (corner frontend A/B): external keypoints for the next extraction (see SetExternalKeypoints).
    const std::vector<cv::KeyPoint>* mpExternalKeypoints = nullptr;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM

#endif

