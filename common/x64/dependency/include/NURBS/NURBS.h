/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   NURBS.h
 * @brief  Header of NURBS fitting method.
 ********************************************************************************
*/

#ifndef NURBS_H_
#define NURBS_H_

#include <numeric>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <utility>
#include <limits>
#include <queue>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/graph/graph_concepts.hpp>
#include "PerformanceAnalysis/PerformAnalysisInterface.h"

#include "LogWrapper/LogWrapper.h"
#include "CommunicateDef/RdbV2SGeometry.h"
#include "CommunicateDef/RdbV2SRoadObject.h"
#include "algoInterface/IRoad.h"

namespace roadDBCore
{

/**
 * A class which implements NURBS fitting method
 */
class NURBS
{

    /**
     * Configuration parameters for NURBS fitting method
     */
public:
    struct ConfigParam
    {
        ConfigParam(const int32_t order = 3,
                    const int32_t maxNumPoints = INT_MAX,
                    const int32_t minNumCtrPoints = 7,
                    const float minCtrPointsRatio = 1.f / 300,
                    const float maxCtrPointsRatio = 1.f / 10,
                    const float translation = 1.0f,
                    const float errorThresh = 0.5f)
         : order(order),
           maxNumPoints(maxNumPoints),
           minNumCtrPoints(minNumCtrPoints),
           minCtrPointsRatio(minCtrPointsRatio),
           maxCtrPointsRatio(maxCtrPointsRatio),
           translation(translation),
           errorThresh(errorThresh)
         {
         }

         bool isValid() const
         {
             bool bValid = true;

             if (minCtrPointsRatio >= maxCtrPointsRatio ||
                 minCtrPointsRatio <  FLT_EPSILON ||
                 maxCtrPointsRatio <  FLT_EPSILON ||
                 minCtrPointsRatio >  1.0f ||
                 maxCtrPointsRatio >  1.0f)
             {
                 SDOR_LOG_WARN << "The ratio of the control points are invalid.";
                 bValid = false;
             }
             else if (minNumCtrPoints < 3)
             {
                 SDOR_LOG_WARN << "The minimum number of the control points must be large than 3.";
                 bValid = false;
             }
             else if (translation < FLT_EPSILON)
             {
                 SDOR_LOG_WARN << "The translation must be positive.";
                 bValid = false;
             }
             else if (errorThresh < FLT_EPSILON)
             {
                 SDOR_LOG_WARN << "The error threshold must be large than 1.";
                 bValid = false;
             }
             else
             {
             }

             return bValid;
         }

         const int32_t  order;
         const int32_t  maxNumPoints;           // Maximum number of points to fit.
                                                // If the input points are too many, this process will occupy too much memory and cpu.
         const int32_t  minNumCtrPoints;        // Minimum number of points to fit
         const float    minCtrPointsRatio;      // Used to calculate the original control points.
                                                // First of all the number of control points is input points divided by minCtrPointsRatio.
         const float    maxCtrPointsRatio;      // The max number of control points.
         const float    translation;            // Used to translate the sigmoid function along x axis
         const float    errorThresh;            // Threshold of error.
    };

private:
    template <class Point3_T>
    class DouglasPeucker
    {
    public:
        explicit DouglasPeucker() : pPointSetRef_(nullptr), mTotalPointNum_(0)
        {
        }

        ~DouglasPeucker(){}

        bool getDPresult(const std::vector<Point3_T> &vPoints,
                         const float &tolerance,
                         std::vector<int32_t> &vResult)
        {
            MONITOR_FUNCTION_PERFORMANCE("NURBS")

            bool bRet = true;
            if (vPoints.empty() || tolerance < FLT_EPSILON)
            {
                SDOR_LOG_WARN << "The parameter is error.";
                bRet = false;
            }
            else
            {
                pPointSetRef_ = &vPoints;
                mTotalPointNum_ = static_cast<int32_t>(vPoints.size());
                mTags_.resize(mTotalPointNum_, false);

                DouglasPeuckerReduction(0, mTotalPointNum_-1, tolerance);

                vResult.clear();
                //first point
                vResult.emplace_back(0);
                for (int index = 1; index < (mTotalPointNum_ - 1); ++index)
                {
                    if (mTags_[index])
                    {
                        vResult.emplace_back(index);
                    }
                }
                //last point
                vResult.emplace_back(mTotalPointNum_-1);
            }

            return bRet;
        }

        bool getDPresultN(const std::vector<Point3_T> &vPoints,
                          const float &ratio,
                          std::vector<int32_t> &vResult)
        {
            MONITOR_FUNCTION_PERFORMANCE("NURBS")

            bool bRet = true;
            vResult.clear();

            pPointSetRef_ = &vPoints;
            mTotalPointNum_ = static_cast<int32_t>(vPoints.size());

            // key count limitation
            const auto countTol = static_cast<int>(ratio * mTotalPointNum_) - 1;

            if (ratio < 0.0 || ratio > 1.0 || countTol < 2)
            {
                SDOR_LOG_WARN << "The parameter is incorrect.";
                bRet = false;
            }
            else
            {
                mTags_.resize(mTotalPointNum_, false);

                // the first and last is always the key
                mTags_[0] = true;
                mTags_[mTotalPointNum_ - 1] = true;

                if (countTol == 2)
                {
                    vResult.emplace_back(0);
                    vResult.emplace_back(mTotalPointNum_ - 1);
                }
                else
                {
                    int keyCnt = 2;

                    std::priority_queue<SubPolyAlt> queue;

                    SubPolyAlt subPoly(0, mTotalPointNum_ - 1);
                    subPoly.keyInfo = FindKey(subPoly.first, subPoly.last);

                    queue.push(subPoly);

                    while (!queue.empty())
                    {
                        subPoly = queue.top();
                        queue.pop();

                        mTags_[subPoly.keyInfo.index] = true;

                        keyCnt++;
                        if (keyCnt == countTol)
                        {
                            break;
                        }

                        // split the polyline at the key and recurse
                        SubPolyAlt left(subPoly.first, subPoly.keyInfo.index);
                        left.keyInfo = FindKey(left.first, left.last);
                        if (left.keyInfo.index)
                        {
                            queue.push (left);
                        }

                        SubPolyAlt right(subPoly.keyInfo.index, subPoly.last);
                        right.keyInfo = FindKey(right.first, right.last);
                        if (right.keyInfo.index)
                        {
                            queue.push (right);
                        }
                    }

                    for (int index = 0; index < mTotalPointNum_; ++index)
                    {
                        if (mTags_[index])
                        {
                            vResult.emplace_back(index);
                        }
                    }
                }
            }

            return bRet;
        }

    private:
        struct KeyInfo {
            KeyInfo (int indexIn = 0, float dist2In = 0) :
                index(indexIn), dist2(dist2In) {}

            int index;
            float dist2;
        };

        struct SubPolyAlt {
            SubPolyAlt (int firstIn = 0, int lastIn = 0) :
                first (firstIn), last(lastIn) {}

            int first;       //! coord index of the first point
            int last;        //! coord index of the last point
            KeyInfo keyInfo; //! key of this sub poly

            bool operator< (const SubPolyAlt& other) const {
                return keyInfo.dist2 < other.keyInfo.dist2;
            }
        };

        KeyInfo FindKey(const int firstPoint, const int lastPoint)
        {
            KeyInfo keyInfo;

            const auto &point1 = (*pPointSetRef_)[firstPoint];
            const auto &point2 = (*pPointSetRef_)[lastPoint];

            for (int index = firstPoint + 1; index < lastPoint; ++index)
            {
                float distance = PerpendicularDistance(point1, point2, (*pPointSetRef_)[index]);
                if (distance > keyInfo.dist2)
                {
                    keyInfo.dist2 = distance;
                    keyInfo.index = index;
                }
            }

            return keyInfo;
        }

        void DouglasPeuckerReduction(const int firstPoint, const int lastPoint, const float tolerance)
        {
            const auto &keyInfo = FindKey(firstPoint, lastPoint);

            if (keyInfo.index >= 0 && keyInfo.dist2 > tolerance)
            {
                mTags_[keyInfo.index] = true;

                DouglasPeuckerReduction(firstPoint, keyInfo.index, tolerance);
                DouglasPeuckerReduction(keyInfo.index, lastPoint, tolerance);
            }
        }

        inline double PerpendicularDistance(const Point3_T &point1,
                                            const Point3_T &point2,
                                            const Point3_T &point3) noexcept
        {
            const cv::Point3d point1d(point1.x, point1.y, point1.z);
            const cv::Point3d point2d(point2.x, point2.y, point2.z);
            const cv::Point3d point3d(point3.x, point3.y, point3.z);

#if 1
            double ab = std::sqrt(std::pow((point1d.x - point2d.x), 2.0) +
                                  std::pow((point1d.y - point2d.y), 2.0) +
                                  std::pow((point1d.z - point2d.z), 2.0));
            double as = std::sqrt(std::pow((point1d.x - point3d.x), 2.0) +
                                  std::pow((point1d.y - point3d.y), 2.0) +
                                  std::pow((point1d.z - point3d.z), 2.0));
            double bs = std::sqrt(std::pow((point3d.x - point2d.x), 2.0) +
                                  std::pow((point3d.y - point2d.y), 2.0) +
                                  std::pow((point3d.z - point2d.z), 2.0));
            double cos_A = (std::pow(as, 2.0) + std::pow(ab, 2.0) - std::pow(bs, 2.0)) / (2 * ab * as + 1e-10);
            double sin_A = std::sqrt(1 - std::pow(cos_A, 2.0));

            return as * sin_A;
#else
            double A = point2d.z - point1d.z;
            double B = point1d.x - point2d.x;
            double C = point2d.x * point1d.z - point1d.x * point2d.z;

            double denominator = std::sqrt(A * A + B * B), dist(0);

            if (denominator < FLT_EPSILON)
            {
                dist = FLT_MAX;
            }
            else
            {
                dist = std::abs((A * point3d.x + B * point3d.z + C) / denominator);
            }

            return dist;
#endif
        }

        const std::vector<Point3_T> *pPointSetRef_;
        std::vector<bool> mTags_;
        int mTotalPointNum_;
    };

private:
    struct ColOfMatrix
    {
        int32_t start_;
        int32_t end_;
        std::vector<double> vec_;
        ColOfMatrix() : start_(-1), end_(0)
        {
        }
    };

    ConfigParam  configPara_;
    size_t       minNumPoint_;
    int32_t      numPointsToFit_;  // Number of points used to fit NURBS curve

public:
    explicit NURBS(const ConfigParam &configPara) : configPara_(configPara), numPointsToFit_(0)
    {
        minNumPoint_ = configPara_.minNumCtrPoints + 2;
    }

    virtual ~NURBS(){};

private:
    template <class Point3_T>
    bool checkFitData(const std::vector<Point3_T> &vInputPoints, const std::vector<std::pair<int32_t, int32_t>> &vEndIndex)
    {
        MONITOR_FUNCTION_PERFORMANCE("NURBS")

        bool bRet(true);
        if (vInputPoints.size() < minNumPoint_)
        {
            SDOR_LOG_WARN << "The number of points is too less.";
            bRet = false;
        }
        else if (vEndIndex.empty())
        {
            SDOR_LOG_WARN << "The end index is empty.";
            bRet = false;
        }
        else if (!configPara_.isValid())
        {
            SDOR_LOG_WARN << "Configuration parameters are invalid.";
            bRet = false;
        }
        else
        {
            numPointsToFit_ = static_cast<int32_t>(vInputPoints.size());
            for (auto it = vEndIndex.begin(); it < vEndIndex.end() && bRet; ++it)
            {
                if (it->first < 0 || it->first >= numPointsToFit_ ||
                    it->second < 0 || it->second >= numPointsToFit_ ||
                    it->first > it->second)
                {
                    SDOR_LOG_WARN << "The end index is invalid.";
                    SDOR_LOG_INFO << "number of input points is " << vInputPoints.size();
                    SDOR_LOG_INFO << "vEndIndex.size() = " << vEndIndex.size();
                    std::ostringstream outString;
                    for (const auto &endPair : vEndIndex)
                    {
                        outString << endPair.first << ", " << endPair.second << ", ";
                    }
                    SDOR_LOG_INFO << outString;
                    bRet = false;
                }
            }

            for (auto it1 = vEndIndex.begin(), it2 = it1 + 1; it2 < vEndIndex.end() && bRet; ++it1, ++it2)
            {
                if (it1->second > it2->first)
                {
                    SDOR_LOG_WARN << "The end index is invalid.";
                    {
                        std::ostringstream outString;
                        for (auto it = vEndIndex.begin(); it < vEndIndex.end(); ++it)
                        {
                            outString << it->first << ", " << it->second << ", ";
                        }
                        SDOR_LOG_INFO << outString;
                    }
                    bRet = false;
                }
            }
        }

        return bRet;
    }

    //return true for sample successful, return false for sample failure, but not error.
    template <class Point3_T>
    bool sampleAndEnd(const std::vector<Point3_T> &vInputPoints,
                      const std::vector<std::pair<int32_t, int32_t>> &vEndIndex,
                      std::vector<Point3_T> &vSamplePoint,
                      std::vector<std::pair<int32_t, int32_t>> &vSampleEndIdx,
                      float32_t distTh = 0.5f)
    {
        MONITOR_FUNCTION_PERFORMANCE("NURBS")

        std::vector<int> vSampleIdx;
        bool bRet = subSampleByDistance(vInputPoints, distTh, vSampleIdx);
        size_t numSamplePoint = vSampleIdx.size() + static_cast<size_t>(vEndIndex.size() << 1);

        if (bRet && numSamplePoint > minNumPoint_)
        {
            //second of every element is it's type, whether it is end index
            std::vector<std::pair<int, bool>> vIdxAndType, vTmpIdxType;
            vIdxAndType.reserve(numSamplePoint);
            for (auto it = vEndIndex.begin(); it < vEndIndex.end(); ++it)
            {
                vIdxAndType.emplace_back(it->first, true);
                vIdxAndType.emplace_back(it->second, true);
            }

            for (auto it = vSampleIdx.begin(); it < vSampleIdx.end(); ++it)
            {
                vIdxAndType.emplace_back(*it, false);
            }

//         SDOR_LOG_DEBUG << "vEndIndex.size() = " << vEndIndex.size();
//         for (auto elem : vEndIndex)
//         {
//             std::cout << "[" << elem.first << ", " << elem.second << "]" << ", ";
//         }
//         std::cout << std::endl;

            std::stable_sort(vIdxAndType.begin(), vIdxAndType.end(),
                    [](const std::pair<int, bool> &pair1, const std::pair<int, bool> &pair2)
                    {
                        if (pair1.first < pair2.first)
                        {
                            return true;
                        }
                        else if (pair1.first == pair2.first)
                        {
                            return pair1.second;
                        }
                        else
                        {
                            return false;
                        }
                    });

//         SDOR_LOG_DEBUG << "vIdxAndType.size() = " << vIdxAndType.size();
//         for (auto elem : vIdxAndType)
//         {
//             std::cout << "[" << elem.first << ", " << elem.second << "]" << ", ";
//         }
//         std::cout << std::endl;

            int index(0);
            for (auto it = vIdxAndType.begin(); it < vIdxAndType.end(); ++it)
            {
                if (it->second)
                {
                    index = it->first;
                }
                else if (index == it->first)
                {
                    continue;
                }
                else
                {
                }

                vTmpIdxType.emplace_back(*it);
            }

            vIdxAndType.swap(vTmpIdxType);

//         SDOR_LOG_DEBUG << "vIdxAndType.size() = " << vIdxAndType.size();
//         for (auto e : vIdxAndType)
//         {
//             std::cout << "[" << e.first << ", " << e.second << "]" << ", ";
//         }
//         std::cout << std::endl;

            vSamplePoint.clear();
            vSamplePoint.reserve(vIdxAndType.size());
            vSampleEndIdx.clear();
            vSampleEndIdx.reserve(vEndIndex.size());

            std::pair<int32_t, int32_t> tmpPair;
            bool bFirst = true;
            int i = 0;
            for (auto it = vIdxAndType.begin(); it < vIdxAndType.end(); ++it, ++i)
            {
                vSamplePoint.emplace_back(vInputPoints[it->first]);
                if (it->second)
                {
                    if (bFirst)
                    {
                        tmpPair.first = i;
                    }
                    else
                    {
                        tmpPair.second = i;
                        vSampleEndIdx.emplace_back(tmpPair);
                    }
                    bFirst = !bFirst;
                }
            }
        }
        else
        {
            //sub sample is unnecessary
            bRet = false;
        }

        return bRet;
    }

    template <class Point3_T>
    float getRadius(const Point3_T &point1, const Point3_T &point2, const Point3_T &point3) const
    {
        float difference1 = point2.z - point1.z, difference2 = point2.z - point3.z;
        float denominator = (point2.x - point3.x) * difference1 - (point2.x - point1.x) * difference2;

        float item1 = 0.5f * (point3.z - point1.z) * (point2.z - point3.z) * (point2.z - point1.z);
        float item2 = 0.5f * (point1.x * point1.x - point2.x * point2.x) * (point2.z - point3.z);
        float item3 = 0.5f * (point2.x * point2.x - point3.x * point3.x) * (point2.z - point1.z);

        float radius(0);
        if (denominator < FLT_EPSILON && denominator > -FLT_EPSILON) // denominator == 0
        {
//             SDOR_LOG_INFO << "The three points are on a line.";
            radius = FLT_MAX;  //the radius is infinite
        }
        else // denominator != 0
        {
            float x = (item1 + item2 + item3) / denominator, z(0);
            if (difference1 > FLT_EPSILON || difference1 < -FLT_EPSILON) // difference1 ！= 0
            {
                z = -(point2.x - point1.x) * (x - 0.5f * (point2.x + point1.x)) / difference1 + 0.5f * (point2.z + point1.z);
            }
            else if (difference2 > FLT_EPSILON || difference2 < -FLT_EPSILON) // difference2 ！= 0
            {
                z = -(point2.x - point3.x) * (x - 0.5f * (point2.x + point3.x)) / difference2 + 0.5f * (point2.z + point3.z);
            }
            else
            {
            }

            float coeff1 = x - point1.x, coeff2 = z - point1.z;
            radius = std::sqrt(coeff1 * coeff1 + coeff2 * coeff2);
        }

#if 0
        Point3_T direction1 = point2 - point1;
        Point3_T direction2 = point3 - point2;

        if (scalarMultiplyInXOZ(direction1, direction2) > 0.f)
        {
            return radius;
        }
        else
        {
            return radius;
        }
#else
        return radius;
#endif
    }

    template <class Point3_T>
    float getRadius3D(const Point3_T &point1, const Point3_T &point2, const Point3_T &point3) const
    {
        double ab2 = std::pow((point1.x - point2.x), 2.0) +
                     std::pow((point1.y - point2.y), 2.0) +
                     std::pow((point1.z - point2.z), 2.0);
        double ac2 = std::pow((point1.x - point3.x), 2.0) +
                     std::pow((point1.y - point3.y), 2.0) +
                     std::pow((point1.z - point3.z), 2.0);
        double bc2 = std::pow((point3.x - point2.x), 2.0) +
                     std::pow((point3.y - point2.y), 2.0) +
                     std::pow((point3.z - point2.z), 2.0);
        double r2 = ab2 * ac2 * bc2 / (2 * ab2 * ac2 +
                                       2 * ab2 * bc2 +
                                       2 * ac2 * bc2 -
                                       std::pow(ab2, 2) -
                                       std::pow(ac2, 2) -
                                       std::pow(bc2, 2) +
                                       1e-10);

        return std::sqrt(r2);
    }

    template <class Point3_T>
    bool smooth(const std::vector<Point3_T> &vInputPoint, const float &rThresh, std::vector<int> &vIndex) const
    {
        bool bRet = true;
        if (vInputPoint.empty() || rThresh < FLT_EPSILON || vIndex.empty())
        {
            SDOR_LOG_WARN << "vInputPoint.size() = " << vInputPoint.size() << ", rThresh = " << rThresh
                           << ", vIndex.size() = " << vIndex.size();
            bRet = false;
        }
        else if (vIndex.front() != 0 || vIndex.back() != static_cast<int>(vInputPoint.size()) - 1)
        {
            SDOR_LOG_WARN << "The first and last element of vIndex is: " << vIndex.front() << ", " << vIndex.back();
            SDOR_LOG_WARN <<  "vInputPoint.size() = " << vInputPoint.size();
            bRet = false;
        }
        else
        {
            std::vector<int> vRemove;
            do
            {
                vRemove.clear();
                int i = 1;
                for (auto it1 = vIndex.begin(), it2 = it1 + 1, it3 = it2 + 1; it3 < vIndex.end(); ++it2, ++it3, ++i)
                {
                    const auto &point1 = vInputPoint[*it1], &point2 = vInputPoint[*it2], &point3 = vInputPoint[*it3];
                    auto radius2D = getRadius(point1, point2, point3);
                    // auto radius3D = getRadius3D(point1, point2, point3);
                    // SDOR_LOG_DEBUG << "[idx1, idx2, idx3, r2d, r3d] = [" << *it1 << ", " << *it2 << ", " << *it3 << ", " << radius2D << ", " << radius3D << "]";
                    if (radius2D < rThresh)
                    {
                        vRemove.emplace_back(i);
                        ++it2;
                        ++it3;
                    }
                    else
                    {
                        it1 = it2;
                    }
                }

                for (auto it = vRemove.rbegin(); it < vRemove.rend(); ++it)
                {
                    vIndex.erase(vIndex.begin() + *it);
                }
            } while(!vRemove.empty());
        }

        return bRet;
    }

    template <class Point3_T>
    bool splitParamSpace(const std::vector<Point3_T> &vInputPoint,
                         const std::vector<int> &vDPIndex,
                         std::vector<float> &vU,
                         std::vector<float> &vKnot) const
    {
        bool bRet = true;
        if (vInputPoint.empty() || vDPIndex.empty())
        {
            SDOR_LOG_WARN << "vInputPoint.size() = " << vInputPoint.size() << ", vDPIndex.size() = " << vDPIndex.size();
            bRet = false;
        }
        else if (vDPIndex.front() != 0 || vDPIndex.back() != static_cast<int>(vInputPoint.size()) - 1)
        {
            SDOR_LOG_WARN << "The first and last element of vDPIndex is: " << vDPIndex.front() << ", " << vDPIndex.back()
                           <<  ", vInputPoint.size() = " << vInputPoint.size();
            bRet = false;
        }
        else
        {
            int32_t numDistance = static_cast<int>(vInputPoint.size()) - 1;
            std::vector<float> vDistance;
            vDistance.reserve(numDistance);

            float32_t tmpDistance, tmpDiff;
            for (auto first = vInputPoint.begin(), second = first + 1; second < vInputPoint.end(); ++first, ++second)
            {
                tmpDiff = first->x - second->x;
                tmpDistance = tmpDiff * tmpDiff;

                tmpDiff = first->y - second->y;
                tmpDistance += tmpDiff * tmpDiff;

                tmpDiff = first->z - second->z;
                tmpDistance += tmpDiff * tmpDiff;

                vDistance.emplace_back(std::sqrt(tmpDistance));
            }

            float32_t sumDistance = std::accumulate(vDistance.begin(), vDistance.end(), 0.f);

            //Generate the vector U, U is u bar
            vU.clear();
            vU.reserve(vInputPoint.size());

            if (sumDistance < 0.1f)
            {
                SDOR_LOG_WARN << "The line is too short.";
                bRet = false;
            }
            else
            {
                float32_t divisor = 1.f / sumDistance, elem(0);
                for (auto it = vDistance.begin(); it < vDistance.end(); ++it)
                {
                    vU.emplace_back(elem);
                    elem += (*it) * divisor;
                    if (elem - 1.f > FLT_MIN)
                    {
                        elem = 1.f;
                    }
                }
                vU.emplace_back(1.f);
            }

            if (bRet)
            {
                //generate knot
                vKnot.clear();
                vKnot.reserve(vDPIndex.size() + 4u);
                vKnot.emplace_back(0);
                vKnot.emplace_back(0);

                for (auto it = vDPIndex.begin(); it < vDPIndex.end(); ++it)
                {
                    vKnot.emplace_back(vU[*it]);
                }

                vKnot.emplace_back(1);
                vKnot.emplace_back(1);
            }
        }

        return bRet;
    }


public:
    /**
     * Fit dashed curve.
     *
     * @param origPoints      [ IN] points to fit, x and z are the ground, y is the height
     * @param endPointsIndice [ IN] indices of end points of each piece in input points
     * @param nurbsParam      [OUT] parameters of fitted curve
     *
     * @return true on success, fail otherwise
     */
    template <class Point3_T>
    bool fitCurve(const std::vector<Point3_T> &vInputPoint,
                  const std::vector<std::pair<int32_t, int32_t>> &vEndIndex,
                  roadDBCore::NURBS_t &curveParam)
    {
        MONITOR_FUNCTION_PERFORMANCE("NURBS")

#if 0
        {
            std::ofstream out1("points.txt");
            for (auto it = vInputPoint.begin(); it < vInputPoint.end(); ++it)
            {
                out1 << it->x << " " << it->y << " " << it->z << std::endl;;
            }

            std::ofstream out2("ctrPoints.txt");
            for (auto it = vEndIndex.begin(); it < vEndIndex.end(); ++it)
            {
                out2 << it->first << " " << it->second << std::endl;;
            }
        }

        SDOR_LOG_DEBUG << i++;
#endif

        bool bRet = checkFitData(vInputPoint, vEndIndex);
        if (bRet)
        {
            const std::vector<Point3_T> *pInputPoint = &vInputPoint;
            const std::vector<std::pair<int32_t, int32_t>> *pEndIdx = &vEndIndex;

            std::vector<Point3_T> vSamplePoint;
            std::vector<std::pair<int32_t, int32_t>> vSampleEndIdx;

            //subSample
            if (sampleAndEnd(vInputPoint, vEndIndex, vSamplePoint, vSampleEndIdx))
            {
                pInputPoint = &vSamplePoint;
                pEndIdx = &vSampleEndIdx;
                numPointsToFit_ = static_cast<int32_t>(vSamplePoint.size());
            }

            DouglasPeucker<Point3_T> DPobj;
            std::vector<int> vDPIndex;

            std::vector<float> vU;
            std::vector<NURBS::ColOfMatrix> matrixN;
            std::vector<Point3_T> R;

            DPobj.getDPresultN(*pInputPoint, 0.1f, vDPIndex);
            bRet = splitParamSpace(*pInputPoint, vDPIndex, vU, curveParam.vecKnot) &&
                   solveMatrixN(static_cast<int>(curveParam.vecKnot.size()) - 3, curveParam.vecKnot, vU, matrixN) &&
                   generateR(matrixN, *pInputPoint, R) &&
                   solveCtrPoints(*pInputPoint, matrixN, R, curveParam.vecCtrlPoint);

            if (!bRet)
            {
                SDOR_LOG_DEBUG<<"Replace to use Douglas method.";
                DPobj.getDPresult(*pInputPoint, 0.1f, vDPIndex);
                bRet = smooth(*pInputPoint, 10.f, vDPIndex) &&
                       splitParamSpace(*pInputPoint, vDPIndex, vU, curveParam.vecKnot) &&
                       solveMatrixN(static_cast<int>(curveParam.vecKnot.size()) - 3, curveParam.vecKnot, vU, matrixN) &&
                       generateR(matrixN, *pInputPoint, R) &&
                       solveCtrPoints(*pInputPoint, matrixN, R, curveParam.vecCtrlPoint);
            }

#if 0
            std::vector<cv::Point2f> fitPoints;
            if (!inerSample(curveParam, matrixN, fitPoints))
            {
                SDOR_LOG_WARN << "The function inerSample return false.";
                return false;
            }
            else
            {
                std::string filename = "/home/test1234/Desktop/Parallels\ Shared\ Folders/Home/Downloads/debug.txt";
                std::ofstream debug;
                debug.open(filename);
                for (auto it = fitPoints.begin(); it < fitPoints.end(); ++it)
                {
                    debug << it->x << " " << it->y << std::endl;
                }
                debug.close();
            }
#endif

            if (bRet)
            {
                //Generate the endPoints with parameter u
                curveParam.endPoint.clear();
                curveParam.endPoint.reserve(pEndIdx->size() << 1);

                for (auto &endPair : *pEndIdx)
                {
                    if (endPair.first >= static_cast<int32_t>(vU.size()) ||
                        endPair.first < 0 ||
                        endPair.second >= static_cast<int32_t>(vU.size()) ||
                        endPair.second < endPair.first)
                    {
                        SDOR_LOG_WARN << "not valid endPair data.";
                        {
                            std::ostringstream outString;
                            outString << "number of input points is " << vInputPoint.size()
                                      << "and the vecEndIndex is : " << std::endl;
                            for (auto &endPair : *pEndIdx)
                            {
                                outString << endPair.first << ", " << endPair.second << ", ";
                            }
                            SDOR_LOG_INFO << outString;
                        }
                        bRet = false;
                        break;
                    }

                    curveParam.endPoint.emplace_back(vU[endPair.first]);
                    curveParam.endPoint.emplace_back(vU[endPair.second]);
                }
            }

            if (bRet)
            {
                bRet = calcAccuracyLineLength(*pInputPoint, *pEndIdx, vU, curveParam);
            }
        }

        return bRet;
    }


#if 0
    /**
     * Fit dashed curve.
     *
     * @param origPoints      [ IN] points to fit, x and z are the ground, y is the height
     * @param endPointsIndice [ IN] indices of end points of each piece in input points
     * @param nurbsParam      [OUT] parameters of fitted curve
     *
     * @return true on success, fail otherwise
     */
    template <class Point3_T>
    bool fitCurve(const std::vector<Point3_T> &inputPoints,
                  const std::vector<std::pair<int32_t, int32_t>> &vecEndIndex,
                  roadDBCore::NURBS_t &curveParam)
    {
        MONITOR_FUNCTION_PERFORMANCE("NURBS")

        bool bRet = true;
        if (inputPoints.size() < minNumPoint_)
        {
            SDOR_LOG_WARN << "The number of points is too less.";
            bRet = false;
        }
        else if (vecEndIndex.empty())
        {
            SDOR_LOG_WARN << "The end index is empty.";
            bRet = false;
        }
        else if (!configPara_.isValid())
        {
            SDOR_LOG_WARN << "Configuration parameters are invalid.";
            bRet = false;
        }
        else
        {
            numPointsToFit_ = static_cast<int32_t>(inputPoints.size());
            for (auto it = vecEndIndex.begin(); it < vecEndIndex.end() && bRet; ++it)
            {
                if (it->first < 0 || it->first >= numPointsToFit_ ||
                    it->second < 0 || it->second >= numPointsToFit_ ||
                    it->first > it->second)
                {
                    SDOR_LOG_WARN << "The end index is invalid.";
                    bRet = false;
                }
            }

            for (auto it1 = vecEndIndex.begin(), it2 = it1 + 1; it2 < vecEndIndex.end() && bRet; ++it1, ++it2)
            {
                if (it1->second >= it2->first)
                {
                    SDOR_LOG_WARN << "The end index is invalid.";
                    bRet = false;
                }
            }
        }

        //start fitting
        std::vector<float32_t> U;
        std::vector<cv::Point2f> fitPoints;
        if (bRet && fitAndReconstruct(inputPoints, curveParam, U, fitPoints))
        {
            //Generate the endPoints with parameter u
            curveParam.endPoint.clear();
            curveParam.endPoint.reserve(vecEndIndex.size() << 1);

            for (auto &endPair : vecEndIndex)
            {
                if (endPair.first >= static_cast<int32_t>(U.size()) ||
                    endPair.first < 0 ||
                    endPair.second >= static_cast<int32_t>(U.size()) ||
                    endPair.second < endPair.first)
                {
                    SDOR_LOG_WARN << "not valid endPair data.";
                    return false;
                }

                curveParam.endPoint.emplace_back(U[endPair.first]);
                curveParam.endPoint.emplace_back(U[endPair.second]);
            }

            bRet = lineLength(fitPoints, vecEndIndex, curveParam.lineLength, curveParam.paintTotalLength);
        }
        else
        {
            bRet = false;
        }

        return bRet;
    }
#endif

public:
    /**
     * Reconstruct points according to fitted parameter.
     *
     * @param nurbsParams [ IN] parameters of fitted curve
     * @param step        [ IN] arc length between two successive points
     * @param points      [OUT] generated points
     *
     * @return true on success, false otherwise
     */
    template <class Point3_T>
    bool reconstruct(const roadDBCore::NURBS_t &NURBSParam,
                     const float step,
                     std::vector<std::vector<Point3_T>> &vvOutPoints) const
    {
        if (NURBSParam.vecCtrlPoint.empty() || step < FLT_EPSILON || step > NURBSParam.paintTotalLength / 3.0f)
        {
            SDOR_LOG_WARN << "The arguments are invalid in function reconstruct.";
            return false;
        }

        int32_t outputPointNum = static_cast<int32_t>(std::ceil(NURBSParam.paintTotalLength / step));

        // Safety guard
        outputPointNum = std::max(outputPointNum, 2);

        std::vector<int32_t> vecIdx;
        vecIdx.reserve(NURBSParam.endPoint.size());

        if (NURBSParam.endPoint.empty())
        {
            SDOR_LOG_WARN << "The endPoint is empty.";
            return false;
        }
        else if ((NURBSParam.endPoint.size() & 0x1) != 0)
        {
            SDOR_LOG_WARN << "The number of end points must be even.";
            return false;
        }
        else
        {
            for (auto it1 = NURBSParam.endPoint.begin(), it2 = it1 + 1; it2 < NURBSParam.endPoint.end(); ++it1, ++it2)
            {
                if (*it1 > *it2)
                {
                    SDOR_LOG_WARN << "The arguments are invalid in function generateCurve.";
                    return false;
                }
            }

            for (auto it1 = NURBSParam.vecKnot.begin(), it2 = it1 + 1; it2 < NURBSParam.vecKnot.end(); ++it1, ++it2)
            {
                if (*it1 > *it2)
                {
                    SDOR_LOG_WARN << "The arguments are invalid in function generateCurve.";
                    return false;
                }
            }
        }

        float length(0);                //the length of the paint
        auto it = NURBSParam.endPoint.begin();
        while (it < NURBSParam.endPoint.end())
        {
            length -= *(it++);
            length += *(it++);
        }

        float scale = length / (outputPointNum - 1);   // outputPointNum >= 2

        if (scale < FLT_EPSILON)
        {
            SDOR_LOG_DEBUG << "The input data are invalid.";
            return false;
        }

        it = NURBSParam.endPoint.begin();
        float tmp = *(it++);

        std::vector<float32_t> U;
        U.reserve(outputPointNum);
        int32_t i = 0;
        while (it < NURBSParam.endPoint.end())
        {
            U.emplace_back(tmp);
            ++i;
            tmp += scale;
            // if tmp > the second end point of a segment, then set tmp as the first end point of next segment.
            if (tmp > *it)
            {
                U.emplace_back(*(it++));
                vecIdx.emplace_back(++i);

                if (it == NURBSParam.endPoint.end())
                {
                    break;
                }
                else
                {
                    tmp = *(it++);
                }
            }
        }

    #if 0
            std::cout << "U.size() = " << U.size() << std::endl;
            for(std::vector<double>::iterator it = U.begin(); it < U.end(); ++it)
            {
                std::cout << *it << " ";
            }
            std::cout << std::endl;
    #endif

        std::vector<Point3_T> outputPoints;
        if (!outSample(NURBSParam, U, outputPoints))
        {
            SDOR_LOG_WARN << "Sample error";
            return false;
        }

        vvOutPoints.clear();
        vvOutPoints.resize(vecIdx.size());

        auto begin = outputPoints.begin();
        auto first = begin, last = begin;

        for (size_t i = 0; i < vvOutPoints.size(); ++i)
        {
            last = begin + vecIdx[i];
            vvOutPoints[i].insert(vvOutPoints[i].end(), first, last);
            first = last;
        }

        return true;
    }

public:
    bool segmentByLength(const roadDBCore::NURBS_t &curveParam,
                         const float &length,
                         std::vector<roadDBCore::NURBS_t> &vParam)
    {
        bool bRet = true;
        if (curveParam.endPoint.empty() || curveParam.vecKnot.empty() ||
            curveParam.vecCtrlPoint.empty() || curveParam.lineLength < 0 ||
            curveParam.paintTotalLength < 0 || length < FLT_EPSILON ||
            (curveParam.endPoint.size() & 0x1) != 0)
        {
            SDOR_LOG_WARN << "input data are invalid.";
            bRet = false;
        }
        else
        {
            int num = static_cast<int>(std::ceil(curveParam.lineLength / length)); // number of segments
            vParam.resize(num, curveParam);

            float paramLength(0);  // length of the line in parameter space
            auto it = curveParam.endPoint.begin();
            while (it < curveParam.endPoint.end())
            {
                paramLength -= *it++;
                paramLength += *it++;
            }

            float scale = paramLength / num;  //length of each line in parameter space
            float interParam(0);              //segment point in parameter space

            for (int i = 0; i < num; ++i)
            {
                interParam += scale;
                auto &param = vParam[i];
                auto it = param.endPoint.begin();
                while (it < param.endPoint.end())
                {
                    if (*it > interParam) //it point the first point of a segment
                    {
                        param.endPoint.erase(it, param.endPoint.end());
                        break;
                    }

                    ++it;

                    if (*it > interParam) //it point the second point of a segment
                    {
                        *it++ = interParam; // it point the first point of next segment
                        param.endPoint.erase(it, param.endPoint.end());
                        break;
                    }

                    ++it;
                }
            }

            vParam.back().endPoint.back() = curveParam.endPoint.back();
        }
        return bRet;
    }

public:
    /**
     * get on point of the curve by the parameter of u.
     *
     * @param nurbsParams [ IN] parameters of fitted curve
     * @param u           [ IN] parameter
     * @param points      [OUT] generated point
     *
     * @return true on success, false otherwise
     */

    template <class Point3_T>
    bool getOnePoint(const roadDBCore::NURBS_t &curveParam,
                     const float u,
                     Point3_T &point) const
    {
        bool bRet = true;
        if (!curveParam.endPoint.empty() && !curveParam.vecKnot.empty())
        {
            std::vector<float32_t> U;
            U.emplace_back(u);
            std::vector<Point3_T> outPoints;
            if (outSample(curveParam, U, outPoints) && !outPoints.empty())
            {
                point = outPoints.front();
            }
            else
            {
                SDOR_LOG_WARN << "output is empty.";
                bRet = false;
            }
        }
        else
        {
            SDOR_LOG_WARN << "The input data are invalid.";
            bRet = false;
        }

        return bRet;
    }


    /**
    * Reconstruct points according and tangent line to fitted parameter.
    *
    * @param nurbsParams   [ IN] parameters of fitted curve
    * @param step          [ IN] arc length between two successive points
    * @param bXORZ         [ IN] abscissa
    * @param xz1           [ IN] abscissa of the previous tangent line
    * @param xz2           [ IN] abscissa of the rear tangent line
    * @param outputPoints  [OUT] generated points
    *
    * @return true on success, false otherwise
    */
    template <class Point3_T>
    bool generateCurveExt(roadDBCore::NURBS_t param,
                          const float32_t step,
                          const bool bXORZ,
                          const float xz1,
                          const float xz2,
                          std::vector<Point3_T> &outputPoints) const
    {
        MONITOR_FUNCTION_PERFORMANCE("NURBS")

        bool bRet = true;

        if (param.vecCtrlPoint.empty() || step < FLT_EPSILON || (param.endPoint.size() & 0x1) != 0 ||
            param.endPoint.empty() || step > param.paintTotalLength / 3.f)
        {
            SDOR_LOG_DEBUG << "param.vecCtrlPoint.size = " << param.vecCtrlPoint.size();
            SDOR_LOG_DEBUG << "step = " << step;
            SDOR_LOG_DEBUG << "param.paintTotalLength = " << param.paintTotalLength;
            SDOR_LOG_WARN << "The arguments are invalid in function generateCurve.";
            bRet = false;
        }
        else
        {
            auto endPoint1 = param.endPoint.front();
            auto endPoint2 = param.endPoint.back();
            param.endPoint.clear();
            param.endPoint.emplace_back(endPoint1);
            param.endPoint.emplace_back(endPoint2);

            int32_t outputPointNum = static_cast<int32_t>(ceil(param.paintTotalLength / step));

            // Safety guard
            if (outputPointNum < 2)
            {
                SDOR_LOG_WARN << "Expected number of points makes no sense.";
                outputPointNum = 2;
            }
            else if (endPoint1 > endPoint2)
            {
                SDOR_LOG_WARN << "The end Point is invalid.";
                bRet = false;
            }
            else
            {
                for (auto it1 = param.vecKnot.begin(), it2 = it1 + 1; it2 < param.vecKnot.end() && bRet; ++it1, ++it2)
                {
                    if (*it1 > *it2)
                    {
                        SDOR_LOG_WARN << "The arguments are invalid in function generateCurve.";
                        bRet = false;
                    }
                }
            }

            float scale;
            if (bRet)
            {
                //compute the length of the paint in parameter space
                float32_t length = endPoint2 - endPoint1;

                scale = length / (outputPointNum - 1);  // outputPointNum >= 2
                if (scale < FLT_EPSILON)
                {
                    SDOR_LOG_DEBUG << "The input data are invalid.";
                    bRet = false;
                }
            }

            std::vector<float32_t> U;
            U.reserve(outputPointNum);
            if (bRet)
            {
                //compute the vector U
                float32_t tmp = endPoint1;
                while (tmp < endPoint2)
                {
                    U.emplace_back(tmp);
                    tmp += scale;
                }
                U.emplace_back(endPoint2);

                bRet = outSample(param, U, outputPoints);
                outputPointNum = U.size();

                if (outputPointNum < 2)
                {
                    bRet = false;
                }
            }

            Point3_T firstPoint, lastPoint, firstDelta, lastDelta;
            int num1, num2;
            if (bRet)
            {
                firstPoint = outputPoints.front();
                lastPoint = outputPoints.back();

                firstDelta = outputPoints[1] - firstPoint;                 //firstDelta = outputPoints[1] - outputPoints[0]
                lastDelta = lastPoint - outputPoints[outputPointNum - 2];  //lastDelta  = outputPoints[outputPointNum-1] - outputPoints[outputPointNum-2]

                firstDelta = firstDelta * (step / norm(firstDelta));
                lastDelta = lastDelta * (step / norm(lastDelta));

//                 SDOR_LOG_DEBUG << "firstDelta.x = " << firstDelta.x;
//                 SDOR_LOG_DEBUG << "lastDelta.x = " << lastDelta.x;

                if (bXORZ)
                {
                    float deltaX1 = fabs(firstDelta.x), deltaX2 = fabs(lastDelta.x);
                    if (deltaX1 < FLT_EPSILON || deltaX2 < FLT_EPSILON)
                    {
                        SDOR_LOG_WARN << "The component is invalid.";
                        bRet = false;
                    }
                    else
                    {
                        num1 = static_cast<int>(ceil(xz1 / deltaX1));
                        num2 = static_cast<int>(ceil(xz2 / deltaX2));
                    }

//                     SDOR_LOG_DEBUG << "deltaX1 = " << deltaX1;
//                     SDOR_LOG_DEBUG << "deltaX2 = " << deltaX2;
                }
                else
                {
                    float deltaZ1 = fabs(firstDelta.z), deltaZ2 = fabs(lastDelta.z);
                    if (deltaZ1 < FLT_EPSILON || deltaZ2 < FLT_EPSILON)
                    {
                        SDOR_LOG_WARN << "The component is invalid.";
                        bRet = false;
                    }
                    else
                    {
                        num1 = static_cast<int>(ceil(xz1 / deltaZ1));
                        num2 = static_cast<int>(ceil(xz2 / deltaZ2));
                    }

//                     SDOR_LOG_DEBUG << "deltaZ1 = " << deltaZ1;
//                     SDOR_LOG_DEBUG << "deltaZ2 = " << deltaZ2;
                }
            }

//             SDOR_LOG_DEBUG << "num1 = " << num1;
//             SDOR_LOG_DEBUG << "num2 = " << num2;

            if (bRet)
            {
                std::vector<Point3_T> firstExt, lastExt;

                float fNum = static_cast<float>(num1);
                Point3_T tmpPoint = subtract(firstPoint, numericalMultiply(fNum, firstDelta));

                for (int i = 0; i < num1; ++i)
                {
                    tmpPoint = add(tmpPoint, firstDelta);
                    firstExt.emplace_back(tmpPoint);
                }

                tmpPoint = lastPoint;
                for (int i = 0; i < num2; ++i)
                {
                    tmpPoint = add(tmpPoint, lastDelta);
                    lastExt.emplace_back(tmpPoint);
                }

                outputPoints.insert(outputPoints.begin(), firstExt.begin(), firstExt.end());
                outputPoints.insert(outputPoints.end(), lastExt.begin(), lastExt.end());
            }
        }

        return bRet;
    }


        /**
     * Generate points according to fitted parameter.
     *
     * @param nurbsParams [ IN] parameters of fitted curve
     * @param step        [ IN] arc length between two successive points
     * @param points      [OUT] generated points
     * @param bSolid      [ IN] whether force the curve to solid curve
     *
     * @return true on success, false otherwise
     */
    template <class Point3_T>
    bool generateCurve(roadDBCore::NURBS_t NURBSParam,
                       const float32_t step,
                       std::vector<Point3_T> &outputPoints,
                       bool bSolid = false) const
    {
        MONITOR_FUNCTION_PERFORMANCE("NURBS")

        if (isManMadeNURBS(NURBSParam))
        {
            Point3_T sPnt, ePnt;
            sPnt.x = NURBSParam.vecCtrlPoint[0].relLon;
            sPnt.y = NURBSParam.vecCtrlPoint[0].relAlt;
            sPnt.z = NURBSParam.vecCtrlPoint[0].relLat;
            ePnt.x = NURBSParam.vecCtrlPoint[2].relLon;
            ePnt.y = NURBSParam.vecCtrlPoint[2].relAlt;
            ePnt.z = NURBSParam.vecCtrlPoint[2].relLat;
            outputPoints.emplace_back(sPnt);
            outputPoints.emplace_back(ePnt);

            return true;
        }

        if (NURBSParam.vecCtrlPoint.empty() || NURBSParam.lineLength < 0.1 || step < FLT_EPSILON)
        {
            SDOR_LOG_DEBUG << "number of control points = " << NURBSParam.vecCtrlPoint.size();
            SDOR_LOG_DEBUG << "step = " << step;
            SDOR_LOG_DEBUG << "paintTotalLength = " << NURBSParam.paintTotalLength;
            SDOR_LOG_DEBUG << "lineLength = " << NURBSParam.lineLength;
            SDOR_LOG_WARN << "The arguments are invalid in function generateCurve.";
            return false;
        }

        {
            bool bRet = true;
            for (auto it1 = NURBSParam.vecKnot.begin(), it2 = it1 + 1; it2 < NURBSParam.vecKnot.end() && bRet; ++it1, ++it2)
            {
                if (*it2 < *it1)
                {
                    SDOR_LOG_WARN << "the parameter of the curve is incorrect, knot is invalid.";
                    bRet = false;
                }
            }

            if (!bRet)
            {
                std::ostringstream outString;
                outString << "knot.size() = " << NURBSParam.vecKnot.size() << std::endl;
                for (auto it = NURBSParam.vecKnot.begin(); it < NURBSParam.vecKnot.end(); ++it)
                {
                    outString << *it << ", ";
                }
                SDOR_LOG_INFO << outString;
                return false;
            }

            for (auto it1 = NURBSParam.endPoint.begin(), it2 = it1 + 1; it2 < NURBSParam.endPoint.end() && bRet; ++it1, ++it2)
            {
                if (*it2 < *it1)
                {
                    SDOR_LOG_WARN << "the parameter of the curve is incorrect, endPoint is invalid.";
                    bRet = false;
                }
            }

            if (!bRet)
            {
                std::ostringstream outString;
                outString << "endPoint.size() = " << NURBSParam.endPoint.size() << std::endl;
                for (auto it = NURBSParam.endPoint.begin(); it < NURBSParam.endPoint.end(); ++it)
                {
                    outString << *it << ", ";
                }
                SDOR_LOG_INFO << outString;
                return false;
            }
        }

        int32_t outputPointNum(0);
        if (bSolid)
        {
            auto endPoint1 = NURBSParam.endPoint.front();
            auto endPoint2 = NURBSParam.endPoint.back();
            NURBSParam.endPoint.clear();
            NURBSParam.endPoint.emplace_back(endPoint1);
            NURBSParam.endPoint.emplace_back(endPoint2);

            outputPointNum = static_cast<int32_t>(std::ceil(NURBSParam.lineLength / step));
        }
        else
        {
            outputPointNum = static_cast<int32_t>(std::ceil(NURBSParam.paintTotalLength / step));
        }

        // Safety guard
        int number = static_cast<int>(NURBSParam.endPoint.size());
        outputPointNum = std::max(outputPointNum, number);

        if (NURBSParam.endPoint.empty())
        {
            SDOR_LOG_WARN << "The parameter of curve is incorrect, endPoint is empty.";
            return false;
        }
        else if ((NURBSParam.endPoint.size() & 0x1) != 0)
        {
            SDOR_LOG_WARN << "The number of end points must be even.";
            return false;
        }
        else
        {
            for (auto it1 = NURBSParam.endPoint.begin(), it2 = it1 + 1; it2 < NURBSParam.endPoint.end(); ++it1, ++it2)
            {
                if (*it1 > *it2)
                {
                    SDOR_LOG_WARN << "The parameter is invalid in function generateCurve, endPoint is incorrect.";
                    return false;
                }
            }

            for (auto it1 = NURBSParam.vecKnot.begin(), it2 = it1 + 1; it2 < NURBSParam.vecKnot.end(); ++it1, ++it2)
            {
                if (*it1 > *it2)
                {
                    SDOR_LOG_WARN << "The parameter is invalid in function generateCurve, knot is incorrect.";
                    return false;
                }
            }
        }

        //compute the length of the paint in parameter space
        float32_t length(0);
        auto it = NURBSParam.endPoint.begin();
        while (it < NURBSParam.endPoint.end())
        {
            length -= *(it++);
            length += *(it++);
        }

        float32_t scale = length / (outputPointNum - 1);  // outputPointNum >= 2
        float32_t epsilon = std::numeric_limits<float32_t>::epsilon();
        if (scale < epsilon)
        {
            scale = epsilon;
        }

        std::vector<float32_t> U;

        if (!bSolid && NURBSParam.paintTotalLength < 0.1)
        {
            SDOR_LOG_WARN << "Length of the line the too short.";
            U = NURBSParam.endPoint;
        }
        else
        {
            //compute the vector U
            it = NURBSParam.endPoint.begin();
            float32_t tmp = *(it++);
            U.reserve(outputPointNum);
            while (it < NURBSParam.endPoint.end())
            {
                U.emplace_back(tmp);
                tmp += scale;
                // if tmp > the second end point of a segment, then set tmp as the first end point of next segment.
                if (tmp > *it)
                {
                    U.emplace_back(*(it++));
                    if (it == NURBSParam.endPoint.end())
                    {
                        break;
                    }
                    else
                    {
                        tmp = *(it++);
                    }
                }
            }
        }

        if (!outSample(NURBSParam, U, outputPoints))
        {
            SDOR_LOG_WARN << "Sample error";
            return false;
        }

        return true;
    }

    /**
     * Generate points according to fitted parameter.
     *
     * @param nurbsParams  [ IN] parameters of fitted curve
     * @param step         [ IN] arc length between two successive points
     * @param outputPoints [OUT] generated points
     * @param vEndIndex    [OUT] index of the dashed segments, first and second of every element are the indexes of first and last of the segment respectively.
     *
     * @return true on success, false otherwise
     */
    template <class Point3_T>
    bool generateCurveAndIdx(const roadDBCore::NURBS_t &NURBSParam,
                             const float32_t step,
                             std::vector<Point3_T> &outputPoints,
                             std::vector<std::pair<int32_t, int32_t>> &vEndIndex) const
    {
        outputPoints.clear();
        vEndIndex.clear();
        
        if (isManMadeNURBS(NURBSParam))
        {
            Point3_T sPnt, ePnt;
            sPnt.x = NURBSParam.vecCtrlPoint[0].relLon;
            sPnt.y = NURBSParam.vecCtrlPoint[0].relAlt;
            sPnt.z = NURBSParam.vecCtrlPoint[0].relLat;
            ePnt.x = NURBSParam.vecCtrlPoint[2].relLon;
            ePnt.y = NURBSParam.vecCtrlPoint[2].relAlt;
            ePnt.z = NURBSParam.vecCtrlPoint[2].relLat;
            outputPoints.emplace_back(sPnt);
            outputPoints.emplace_back(ePnt);
            vEndIndex.emplace_back(std::make_pair(0,1));

            return true;
        }

        if (NURBSParam.vecCtrlPoint.empty() || step < FLT_EPSILON)
        {
            SDOR_LOG_DEBUG << "number of control points = " << NURBSParam.vecCtrlPoint.size();
            SDOR_LOG_DEBUG << "step = " << step;
            SDOR_LOG_DEBUG << "paintTotalLength = " << NURBSParam.paintTotalLength;
            SDOR_LOG_WARN << "The arguments are invalid in function generateCurve.";
            return false;
        }

        if (NURBSParam.endPoint.empty())
        {
            SDOR_LOG_WARN << "the parameter of the curve is invalid, the endPoint is empty.";
            return false;
        }
        else if ((NURBSParam.endPoint.size() & 0x1) != 0)
        {
            SDOR_LOG_WARN << "The number of end points must be even.";
            return false;
        }
        else
        {
            for (auto it1 = NURBSParam.endPoint.begin(), it2 = it1 + 1; it2 < NURBSParam.endPoint.end(); ++it1, ++it2)
            {
                if (*it1 > *it2)
                {
                    SDOR_LOG_WARN << "The arguments are invalid in function generateCurve.";
                    return false;
                }
            }

            for (auto it1 = NURBSParam.vecKnot.begin(), it2 = it1 + 1; it2 < NURBSParam.vecKnot.end(); ++it1, ++it2)
            {
                if (*it1 > *it2)
                {
                    SDOR_LOG_WARN << "The arguments are invalid in function generateCurve.";
                    return false;
                }
            }
        }

        const auto numEndPoint = static_cast<int32_t>(NURBSParam.endPoint.size());
        const auto outputPointNum = std::max<int32_t>(
                std::ceil(NURBSParam.paintTotalLength / step),
                numEndPoint);

        //compute the length of the paint in parameter space
        float32_t length(0);
        auto it = NURBSParam.endPoint.begin();
        while (it < NURBSParam.endPoint.end())
        {
            length -= *(it++);
            length += *(it++);
        }

        std::vector<float32_t> U;
        U.reserve(outputPointNum);

        float32_t scale = length / (outputPointNum - 1);  // outputPointNum >= 2
        float32_t epsilon = std::numeric_limits<float32_t>::epsilon();
        if (scale < epsilon)
        {
            scale = epsilon;
        }

        // If scale is too small, use endPoint as U directly
        if (NURBSParam.paintTotalLength < 0.1)
        {
            SDOR_LOG_WARN << "The scale is too small: " << scale;

            U = NURBSParam.endPoint;

            vEndIndex.reserve(numEndPoint >> 1);

            // Set vEndIndex
            for (auto i = 0; i < numEndPoint; i += 2)
            {
                vEndIndex.emplace_back(i, i + 1);
            }
        }
        else // Otherwise, compute the vector U
        {
            it = NURBSParam.endPoint.begin();
            float32_t tmp = *(it++);
            U.emplace_back(tmp);

            std::pair<int32_t, int32_t> tmpPair(0, 0);

            bool bGap = false;

            while (it < NURBSParam.endPoint.end())
            {
                // if tmp > the second end point of a segment, then set tmp as the first end point of next segment.
                if (tmp > *it)
                {
                    int32_t idx = static_cast<int32_t>(U.size());
                    tmpPair.second = idx;                  //idx-1 is the index of the last point of this segment

                    if (!bGap)
                    {
                        U.emplace_back(*(it++));
                        vEndIndex.emplace_back(tmpPair);
                    }
                    else
                    {
                        tmp = *(it++);
                        U.emplace_back(tmp);
                        tmpPair.first = idx;   //idx is the index of the first point of the next segment
                    }

                    bGap = !bGap;
                }
                else
                {
                    U.emplace_back(tmp);
                }

                if (it < NURBSParam.endPoint.end())
                {
                    tmp += scale;
                }
            }
        }

        if (!outSample(NURBSParam, U, outputPoints))
        {
            LOG_ERROR << "Sample error";
            return false;
        }

        /*
        // Tweak: merge paint strips like 0-2, 3-x, x+1-y...
        const auto numPair = static_cast<int32_t>(vEndIndex.size());

        if (numPair > 0)
        {
            const auto GAP_TH = 2;

            std::vector<std::pair<int32_t, int32_t>> indexPairsMerged;
            indexPairsMerged.reserve(numPair);

            // Put the first pair in
            indexPairsMerged.emplace_back(vEndIndex.front());

            for (auto i = 1; i < numPair; ++i)
            {
                const auto &curPair = vEndIndex[i];
                auto &lastPair      = indexPairsMerged.back();

                if (curPair.first - lastPair.second <= GAP_TH)
                {
                    // Merge current pair and the last one
                    lastPair.second = curPair.second;
                }
                else
                {
                    indexPairsMerged.emplace_back(curPair);
                }
            }

            vEndIndex = std::move(indexPairsMerged);
        }
        */
        return true;
    }


    /**
     * The sample function used by generateCurve
     *
     * @param inputParams [ IN] parameters of fitted curve
     * @param U           [ IN] It influences the distribution of the fitting points
     * @param outPoints   [OUT] generated points
     *
     * @return true on success, false otherwise
     */
    template <class Point3_T>
    bool outSample(const roadDBCore::NURBS_t &curveParam,
                   const std::vector<float32_t> &U,
                   std::vector<Point3_T> &outPoints) const
    {
        MONITOR_FUNCTION_PERFORMANCE("NURBS")

        if (curveParam.vecCtrlPoint.empty() || U.empty())
        {
            SDOR_LOG_WARN << "The arguments are invalid in function outSample.";
            return false;
        }

        int32_t degree = configPara_.order - 1;

        outPoints.clear();
        outPoints.reserve(U.size());

        int32_t nCP = static_cast<int32_t>(curveParam.vecCtrlPoint.size());

        std::vector<float32_t> N;  //N contain the non-zero elements of each column of matrixN.
        int32_t tmpInd;
        int32_t s = degree;        //s return the position of U[col] in parameter space.

        Point3_T tmpPoint;

        for (size_t col = 0, numPoints = U.size(); col < numPoints; ++col)
        {
            if (!findSpan(nCP-1, s, U[col], curveParam.vecKnot, s))   // nCP - degree + 1 = nCP - 1 when degree = 2
            {
                return false;
            }

            if (!basisFun(s, U[col], degree, curveParam.vecKnot, N))
            {
                return false;
            }

            tmpInd = s - degree;
            reset(tmpPoint);
            for (int32_t i = 0; i <= degree; i++)
            {
                const auto &elemN = N[i];
                const auto &ctrPoint = curveParam.vecCtrlPoint[tmpInd+i];
                tmpPoint.x += elemN * ctrPoint.relLon;
                tmpPoint.y += elemN * ctrPoint.relAlt;
                tmpPoint.z += elemN * ctrPoint.relLat;
            }
            if (!std::isfinite(tmpPoint.x) || !std::isfinite(tmpPoint.y) || !std::isfinite(tmpPoint.z))
            {
                SDOR_LOG_DEBUG << "the point is not valid " << " tmpPoint.x " << tmpPoint.x << " tmpPoint.y " << tmpPoint.y << " tmpPoint.z " << tmpPoint.z;
                continue;
            }

            outPoints.emplace_back(tmpPoint);
        }

        return !outPoints.empty();
    }


    /**
     * This function is to slice the NURBS curve into some segments.
     *
     * @param nurbsParam        [ IN] parameters of input NURBS curve
     * @param vNode             [ IN] the points in which neighborhood is the break point.
     * @param vSecParam         [OUT] parameters of output NRUBS curve
     * @param vSectionPoints    [OUT] point cloud of the output NURBS
     * @param vSecIndex         [OUT] index of these sections that successful slice the NURBS curve
     * @param vSegType          [OUT] the slice type of the sections
     *
     * @return true on success, false otherwise
     */
    template <class Point3_T>
    bool segmentInRedundantSec(const roadDBCore::NURBS_t           &nurbsParam,
                               const std::vector<cv::Point3f>      &vNode,
                               std::vector<roadDBCore::NURBS_t>    &vSecParam,
                               std::vector<std::vector<Point3_T>>  &vSectionPoints,
                               std::vector<int32_t>                &vSecIndex,
                               std::vector<TRUNCATE_TYPE_E>        &vSegType) const
    {
        MONITOR_FUNCTION_PERFORMANCE("NURBS")

        if (vNode.empty() || nurbsParam.vecCtrlPoint.empty() || nurbsParam.vecKnot.empty() ||
            nurbsParam.endPoint.empty() || (nurbsParam.endPoint.size() & 0x1) != 0)
        {
            SDOR_LOG_WARN << "The input data are invalid.";
            SDOR_LOG_INFO << "vNode.size() = " << vNode.size();
            SDOR_LOG_INFO << "number of control points = " << nurbsParam.vecCtrlPoint.size();
            SDOR_LOG_INFO << "nurbsParam.vecKnot.size() = " << nurbsParam.vecKnot.size();
            SDOR_LOG_INFO << "nurbsParam.endPoint.size() = " << nurbsParam.endPoint.size();
            return false;
        }

        float32_t paramLength(0);
        auto it = nurbsParam.endPoint.begin();
        while (it < nurbsParam.endPoint.end())
        {
            paramLength -= *(it++);
            paramLength += *(it++);
        }

        if (paramLength < FLT_EPSILON)
        {
            SDOR_LOG_WARN << "The length of the paint is zero.";
            return false;
        }

        const float32_t step = 0.1f;
        std::vector<float32_t> U;
        std::vector<Point3_T> vPoint;
        std::vector<size_t> vRange;

        if (!generateCurveAndRegion(nurbsParam, step, vPoint, vRange, U))
        {
            SDOR_LOG_WARN << "The function generateCurveAndRegion return false";
            return false;
        }

        std::vector<int32_t> vNodeIndex;
        if (!searchNodeIndexInReport(vNode, vPoint, vNodeIndex, vSecIndex, vSegType))
        {
            SDOR_LOG_WARN << "No corresponding section!";
            return false;
        }

        size_t CPIndex1, CPIndex2;
        auto itCP = nurbsParam.vecCtrlPoint.begin();
        auto itKnot = nurbsParam.vecKnot.begin();

        vSecParam.resize(vSecIndex.size());
        vSectionPoints.resize(vSecIndex.size());

        size_t nodeIndex1, nodeIndex2;
        for (size_t i = 0, j = 0; i < vSecParam.size(); )
        {
            nodeIndex1 = vNodeIndex[j];
            nodeIndex2 = vNodeIndex[j+1];
#if 0
            //this code is bad, we will improve it later.
            auto u = U[nodeIndex2];
            //first point to the second point of a segment
            //second point to the first point of the next segment
            auto first = nurbsParam.endPoint.begin() + 1, second = first + 1;
            for (; second < nurbsParam.endPoint.end(); first += 2, second += 2)
            {
                if (u - *first > FLT_EPSILON && u - *second < -FLT_EPSILON)
                {
                    while((u - *second < -FLT_EPSILON) && nodeIndex2+1 < U.size())
                    {
                        u = U[++nodeIndex2];
                    }
                    break;
                }
            }
#endif

            vSectionPoints[i].assign(vPoint.begin() + nodeIndex1, vPoint.begin() + nodeIndex2 + 1);

            CPIndex1 = vRange[nodeIndex1] - 2;
            CPIndex2 = vRange[nodeIndex2];

            auto pSecParam = &vSecParam[i];
            pSecParam->vecCtrlPoint.assign(itCP + CPIndex1, itCP + CPIndex2 + 1);
            pSecParam->vecKnot.assign(itKnot + CPIndex1, itKnot + CPIndex2 + 4);

            //search the first segment
            auto it = nurbsParam.endPoint.begin(); // it point the first end of the first segment
            while (U[nodeIndex1] - (*it) > -FLT_MIN && it < nurbsParam.endPoint.end())
            {
                it += 2; // it point to the first end of the next segment
            }
            --it;  // it point to the second end of the previous segment
            if (U[nodeIndex1] - (*it) < FLT_MIN)
            {
                if ((*it) - U[vNodeIndex[j+1]] < -FLT_MIN)
                {
                    //in this condition it point to the second end of the next segment
                    pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                    pSecParam->endPoint.emplace_back(*it++);
                    pSecParam->endPoint.emplace_back(*it++);
                }
                else
                {
                    pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                }
            }
            else
            {
                //in this condition it point to the second end of the next segment
                //add a new segment with length that equal to zero
                pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                ++it;
                pSecParam->endPoint.emplace_back(*it++);
            }

            //search the last segment
            while (U[nodeIndex2] - (*it) > FLT_MIN && it < nurbsParam.endPoint.end())
            {
                //in this condition it point to the second end of the next segment
                pSecParam->endPoint.emplace_back(*it++);
                pSecParam->endPoint.emplace_back(*it++);
            }
            --it; // it point to the first end of the current segment
            if ((*it) < U[nodeIndex2])
            {
                //the segment point is inside of the segment line
                pSecParam->endPoint.emplace_back(U[nodeIndex2]);
            }
            else
            {
                //the segment point is outside of the segment line
                pSecParam->endPoint.pop_back();
                //add a new segment with length that equal to zero
                pSecParam->endPoint.emplace_back(U[nodeIndex2]);
                pSecParam->endPoint.emplace_back(U[nodeIndex2]);
            }

            //compute the length of segment in parameter space
            float32_t coeff(0);
            auto itSecParam = pSecParam->endPoint.begin();
            while (itSecParam < pSecParam->endPoint.end())
            {
                coeff -= *(itSecParam++);
                coeff += *(itSecParam++);
            }

            //compute paint length of segment.
            pSecParam->paintTotalLength = nurbsParam.paintTotalLength / paramLength * coeff;

            //compute the length of line in parameter space
            coeff = nurbsParam.endPoint.back() - nurbsParam.endPoint.front();
            if (coeff < FLT_EPSILON)  // if coeff <= 0
            {
                pSecParam->lineLength = 0.f;
            }
            else
            {
                //length of segment = length of line * length of segment in parameter space / length of line in parameter space
                pSecParam->lineLength = nurbsParam.lineLength * (pSecParam->endPoint.back() - pSecParam->endPoint.front()) / coeff;
            }

            //if the segment is too short then discard it.
            if (pSecParam->paintTotalLength < 1.f)
            {
                //if the indexes of a section are reverse order, then discard it.
                vNodeIndex.erase(vNodeIndex.begin() + j, vNodeIndex.begin() + j + 2);
                vSecIndex.erase(vSecIndex.begin() + i);
                vSegType.erase(vSegType.begin() + i);
                vSectionPoints.erase(vSectionPoints.begin() + i);
                vSecParam.erase(vSecParam.begin() + i);
            }
            else
            {
                ++i;
                j += 2;
            }
        }

        return true;
    }


      /**
     * This function is to slice the NURBS curve into some segments.
     *
     * @param nurbsParam        [ IN] parameters of input NURBS curve
     * @param vNode             [ IN] the points in which neighborhood is the break point.
     * @param vSecParam         [OUT] parameters of output NRUBS curve
     * @param vSectionPoints    [OUT] point cloud of the output NURBS
     * @param vSecIndex         [OUT] index of these sections that successful slice the NURBS curve
     * @param vSegType          [OUT] the slice type of the sections
     *
     * @return true on success, false otherwise
     */
    template <class Point3_T>
    bool segmentInRedundantSecByRef(const roadDBCore::NURBS_t           &nurbsParam,
                                    const std::vector<Point3_T>         &vNode,
                                    const std::vector<std::vector<Point3_T>>    &vReference,
                                    const std::vector<std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXY>>> &vpKdtree,
                                    std::vector<roadDBCore::NURBS_t>    &vSecParam,
                                    std::vector<std::vector<Point3_T>>  &vSectionPoints,
                                    std::vector<int32_t>                &vSecIndex,
                                    std::vector<TRUNCATE_TYPE_E>        &vSegType,
                                    std::vector<std::pair<Point3_T, Point3_T>> &vSegPoint) const
    {
        if (vNode.empty() || nurbsParam.vecCtrlPoint.empty() || nurbsParam.vecKnot.empty() ||
            nurbsParam.endPoint.empty() || (nurbsParam.endPoint.size() & 0x1) != 0)
        {
            SDOR_LOG_WARN << "The input data are invalid.";
            SDOR_LOG_DEBUG << "vNode.size() = " << vNode.size();
            SDOR_LOG_DEBUG << "number of control points is  = " << nurbsParam.vecCtrlPoint.size();
            SDOR_LOG_DEBUG << "knot.size() = " << nurbsParam.vecKnot.size();
            SDOR_LOG_DEBUG << "endPoint.size() = " << nurbsParam.endPoint.size();
            return false;
        }

        float32_t paramLength(0);
        auto it = nurbsParam.endPoint.begin();
        while (it < nurbsParam.endPoint.end())
        {
            paramLength -= *(it++);
            paramLength += *(it++);
        }

        if (paramLength < FLT_EPSILON)
        {
            SDOR_LOG_WARN << "The length of the paint is zero.";
            return false;
        }

        std::vector<float32_t> U;
        std::vector<Point3_T> vDensePoint;
        std::vector<size_t> vRange;

        if (!generateCurveAndRegion(nurbsParam, 0.2f, vDensePoint, vRange, U))
        {
            SDOR_LOG_WARN << "The function generateCurveAndRegion return false";
            return false;
        }

        std::vector<int32_t> vNodeIndex;
        if (!searchNodeIndexInReportByRef(vNode, vDensePoint, vReference, vpKdtree, vNodeIndex, vSecIndex))
        {
            SDOR_LOG_WARN << "No corresponding section!";
            return false;
        }

        if (vSecIndex.size() == 1u)
        {
            vSegType.resize(1, TRUNCATE_TYPE_NO_E);
        }
        else if (vSecIndex.size() > 1u)
        {
            vSegType.resize(vSecIndex.size(), TRUNCATE_TYPE_BOTH_E);
            vSegType.front() = TRUNCATE_TYPE_SECOND_E;
            vSegType.back() = TRUNCATE_TYPE_FIRST_E;
        }
        else
        {
        }

        {
            std::ostringstream outString;
            outString << "Display the valid divisions: vSecIndex.size() = " << vSecIndex.size() << std::endl;
            for (auto &i : vSecIndex)
            {
                outString << i << ", ";
            }
            SDOR_LOG_DEBUG << outString;
        }

#if 0
        {
            std::ostringstream outString;
            SDOR_LOG_DEBUG << "first vNodeIndex.size() = " << vNodeIndex.size();
            for (auto &i : vNodeIndex)
            {
                outString << i << ", ";
            }
            SDOR_LOG_DEBUG << outString;
        }
#endif

        size_t CPIndex1, CPIndex2;
        auto itCP = nurbsParam.vecCtrlPoint.begin();
        auto itKnot = nurbsParam.vecKnot.begin();

        vSecParam.resize(vSecIndex.size());
        vSectionPoints.resize(vSecIndex.size());

        size_t i(0), j(0);
        size_t nodeIndex1, nodeIndex2;
        while (i < vSecParam.size())
        {
            nodeIndex1 = vNodeIndex[j];
            nodeIndex2 = vNodeIndex[j+1];

            vSectionPoints[i].assign(vDensePoint.begin() + nodeIndex1, vDensePoint.begin() + nodeIndex2 + 1);

            CPIndex1 = vRange[nodeIndex1] - 2;
            CPIndex2 = vRange[nodeIndex2];

            auto pSecParam = &vSecParam[i];
            pSecParam->vecCtrlPoint.assign(itCP + CPIndex1, itCP + CPIndex2 + 1);
            pSecParam->vecKnot.assign(itKnot + CPIndex1, itKnot + CPIndex2 + 4);

            //search the first segment
            auto it = nurbsParam.endPoint.begin(); // it point the first end of the first segment
            while (U[nodeIndex1] - (*it) > -FLT_MIN && it < nurbsParam.endPoint.end())
            {
                it += 2; // it point to the first end of the next segment
            }
            --it;  // it point to the second end of the previous segment
            if (U[nodeIndex1] - (*it) < FLT_MIN)
            {
                if ((*it) - U[vNodeIndex[j+1]] < -FLT_MIN)
                {
                    //in this condition it point to the second end of the next segment
                    pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                    pSecParam->endPoint.emplace_back(*it++);
                    pSecParam->endPoint.emplace_back(*it++);
                }
                else
                {
                    pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                }
            }
            else
            {
                //in this condition it point to the second end of the next segment
                //add a new segment with length that equal to zero
                pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                ++it;
                pSecParam->endPoint.emplace_back(*it++);
            }

            //search the last segment
            while (U[nodeIndex2] - (*it) > FLT_MIN && it < nurbsParam.endPoint.end())
            {
                //in this condition it point to the second end of the next segment
                pSecParam->endPoint.emplace_back(*it++);
                pSecParam->endPoint.emplace_back(*it++);
            }
            --it; // it point to the first end of the current segment
            if ((*it) < U[nodeIndex2])
            {
                //the segment point is inside of the segment line
                pSecParam->endPoint.emplace_back(U[nodeIndex2]);
            }
            else
            {
                //the segment point is outside of the segment line
                pSecParam->endPoint.pop_back();
                //add a new segment with length that equal to zero
                pSecParam->endPoint.emplace_back(U[nodeIndex2]);
                pSecParam->endPoint.emplace_back(U[nodeIndex2]);
            }

            //compute the length of segment in parameter space
            float32_t coeff(0);
            auto itSecParam = pSecParam->endPoint.begin();
            while (itSecParam < pSecParam->endPoint.end())
            {
                coeff -= *(itSecParam++);
                coeff += *(itSecParam++);
            }

            //compute paint length of segment.
            pSecParam->paintTotalLength = nurbsParam.paintTotalLength / paramLength * coeff;

            //compute the length of line in parameter space
            coeff = nurbsParam.endPoint.back() - nurbsParam.endPoint.front();
            if (coeff < FLT_EPSILON)  // if coeff <= 0
            {
                pSecParam->lineLength = 0.f;
            }
            else
            {
                //length of segment = length of line * length of segment in parameter space / length of line in parameter space
                pSecParam->lineLength = nurbsParam.lineLength * (pSecParam->endPoint.back() - pSecParam->endPoint.front()) / coeff;
            }

            //if the segment is too short then discard it.
            if (pSecParam->paintTotalLength < 1.0f)
            {
                //if the indexes of a section are reverse order, then discard it.
                vNodeIndex.erase(vNodeIndex.begin() + j, vNodeIndex.begin() + j + 2);
                vSecIndex.erase(vSecIndex.begin() + i);
                vSegType.erase(vSegType.begin() + i);
                vSectionPoints.erase(vSectionPoints.begin() + i);
                vSecParam.erase(vSecParam.begin() + i);
            }
            else
            {
                ++i;
                j += 2;
            }
        }

        vSegPoint.clear();
        if (!vNodeIndex.empty())
        {
            vSegPoint.reserve(vNodeIndex.size());
            std::pair<Point3_T, Point3_T> tmpPair;
            auto nodeIt = vNodeIndex.begin();
            tmpPair.first = vDensePoint[*nodeIt++];
            while (nodeIt < vNodeIndex.end()-1)
            {
                tmpPair.second = vDensePoint[*nodeIt++];
                vSegPoint.emplace_back(tmpPair);
                tmpPair.first = vDensePoint[*nodeIt++];
            }
            tmpPair.second = vDensePoint[*nodeIt];
            vSegPoint.emplace_back(tmpPair);
        }

        return true;
    }

/**
* This function is to slice the NURBS curve into some segments.
*
* @param nurbsParam        [ IN] parameters of input NURBS curve
* @param vNode             [ IN] the points in which neighborhood is the break point.
* @param vSegType          [OUT] the slice type of the sections
* @param vSecParam         [OUT] parameters of output NRUBS curve
* @param vSectionPoints    [OUT] point cloud of the output NURBS
*
* @return true on success, false otherwise
*/
template <class Point3_T>
bool segmentInAssortedSec(const roadDBCore::NURBS_t                   &nurbsParam,
                            const std::vector<Point3_T>                 &vNode,
                            const std::vector<int32_t>                  &vSecIndex,
                            std::vector<roadDBCore::NURBS_t>            &vSecParam,
                            std::vector<std::vector<Point3_T>>          &vSectionPoints,
                            std::vector<std::pair<Point3_T, Point3_T>>  &vSegSortPoint,
                            std::vector<TRUNCATE_TYPE_E>                &vSegType) const
{
    if (vNode.empty() || nurbsParam.vecCtrlPoint.empty() || nurbsParam.vecKnot.empty() ||
        nurbsParam.endPoint.empty() || (nurbsParam.endPoint.size() & 0x1) != 0)
    {
        SDOR_LOG_WARN << "The input data are empty.";
        SDOR_LOG_INFO << "vNode.size() = " << vNode.size();
        SDOR_LOG_INFO << "vSecIndex.size() = " << vSecIndex.size();
        SDOR_LOG_INFO << "number of control points = " << nurbsParam.vecCtrlPoint.size();
        SDOR_LOG_INFO << "nurbsParam.vecKnot.size() = " << nurbsParam.vecKnot.size();
        SDOR_LOG_INFO << "nurbsParam.endPoint.size() = " << nurbsParam.endPoint.size();
        return false;
    }

    float32_t paramLength(0);
    auto it = nurbsParam.endPoint.begin();
    while (it < nurbsParam.endPoint.end())
    {
        paramLength -= *it++;
        paramLength += *it++;
    }

    if (paramLength < FLT_EPSILON)
    {
        SDOR_LOG_WARN << "the length of the line is zero or negtive number.";
        return false;
    }

    const float32_t step = 0.1f;
    std::vector<float32_t> U;
    std::vector<Point3_T> vPoint;
    std::vector<size_t> vRange;

    if (!generateCurveAndRegion(nurbsParam, step, vPoint, vRange, U))
    {
        SDOR_LOG_WARN << "The function generateCurveAndRegion return false.";
        return false;
    }

    std::vector<int32_t> vNodeIndex;
    if (!searchNodeIndexInMerge(vNode, vPoint, vSecIndex, vNodeIndex, vSegType))
    {
        SDOR_LOG_WARN << "Data error.";
        return false;
    }

    std::vector<std::pair<Point3_T, Point3_T>> vSegPoint;
    if (!vSecIndex.empty())
    {
        vSegPoint.reserve(vSecIndex.size());
        std::vector<size_t> vSecSort;
        for (size_t i = 0; i < vNodeIndex.size(); i += 2)
        {
            vSecSort.emplace_back(i);
        }

        std::stable_sort(vSecSort.begin(), vSecSort.end(),
                         [vNodeIndex](const size_t &i, const size_t &j)
                         {
                             return (vNodeIndex[i] < vNodeIndex[j]);
                         });

        std::pair<Point3_T, Point3_T> tmpPair;

        //index1 is the index of the first end point of the first section
        //index2 is the index of the second end point of the first section
        size_t index1 = vNodeIndex[vSecSort.front()], index2 = vNodeIndex[vSecSort.front() + 1];
        Point3_T tmpPoint = vPoint[index1];
        for (int32_t i = 1, iEnd = static_cast<int32_t>(vSecSort.size()); i < iEnd; ++i)
        {
            tmpPair.first = tmpPoint;
            tmpPoint = vPoint[index2];
            tmpPair.second = tmpPoint;
            vSegPoint.emplace_back(tmpPair);

            index2 = vNodeIndex[vSecSort[i] + 1];
        }

        tmpPair.first = tmpPoint;
//             index2 = vNodeIndex[vSecSort.back()] + 1;
        tmpPair.second = vPoint[index2];
        vSegPoint.emplace_back(tmpPair);

//             SDOR_LOG_DEBUG << "vSecSort.size() = " << vSecSort.size();
//             SDOR_LOG_DEBUG << "vSegPoint.size() = " << vSegPoint.size();

        vSegSortPoint.clear();
        vSegSortPoint.resize(vSegPoint.size());
        size_t index;
        for (size_t i = 0; i < vSecSort.size(); ++i)
        {
            index = vSecSort[i];
            vSegSortPoint[index >> 1] = vSegPoint[i];
        }

        if (vSegType.size() == 1u)
        {
            vSegType.resize(1, TRUNCATE_TYPE_NO_E);
        }
        else if(vSegType.size() > 1u)
        {
            vSegType.resize(vSegType.size(), TRUNCATE_TYPE_BOTH_E);
            vSegType[vSecSort.front() >> 1] = TRUNCATE_TYPE_SECOND_E;
            vSegType[vSecSort.back() >> 1] = TRUNCATE_TYPE_FIRST_E;
        }
        else
        {
        }
    }

    size_t CPIndex1, CPIndex2;
    size_t nodeIndex1, nodeIndex2;
    auto itCP = nurbsParam.vecCtrlPoint.begin();
    auto itKnot = nurbsParam.vecKnot.begin();

    vSecParam.resize(vSecIndex.size());
    vSectionPoints.resize(vSecIndex.size());

    for (size_t i = 0, j = 0; i < vSecParam.size(); ++i, j += 2)
    {
        nodeIndex1 = vNodeIndex[j];
        nodeIndex2 = vNodeIndex[j+1];

        //this code is bad, we will improve it later.
        auto u = U[nodeIndex2];
        //first point to the second point of a segment
        //second point to the first point of the next segment
        auto first = nurbsParam.endPoint.begin() + 1, second = first + 1;
        for (; second < nurbsParam.endPoint.end(); first += 2, second += 2)
        {
            if (u - *first > FLT_MIN && u - *second < -FLT_MIN)
            {
                while((u - *second < -FLT_MIN) && nodeIndex2+1 < U.size())
                {
                    u = U[++nodeIndex2];
                }
                break;
            }
        }

        vSectionPoints[i].assign(vPoint.begin() + nodeIndex1, vPoint.begin() + nodeIndex2 + 1);

        CPIndex1 = vRange[nodeIndex1] - 2;
        CPIndex2 = vRange[nodeIndex2];

#if 0
        SDOR_LOG_DEBUG << "CPIndex1 = " << CPIndex1;
        SDOR_LOG_DEBUG << "CPIndex2 = " << CPIndex2;
#endif

        auto pSecParam = &vSecParam[i];
        pSecParam->vecCtrlPoint.assign(itCP + CPIndex1, itCP + CPIndex2 + 1);
        pSecParam->vecKnot.assign(itKnot + CPIndex1, itKnot + CPIndex2 + 4);

        //search the first segment
        auto it = nurbsParam.endPoint.begin();
        while (U[nodeIndex1] - *it > -FLT_MIN && it < nurbsParam.endPoint.end())
        {
            it += 2; // it point to the first end of the next segment
        }
        --it;  // it point the second end of the previous segment
        if (U[nodeIndex1] - *it < FLT_MIN)
        {
            if ((*it) - U[nodeIndex2] < -FLT_MIN)
            {
                //in this condition it point the second end of the next segment
                pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                pSecParam->endPoint.emplace_back(*it++);
                pSecParam->endPoint.emplace_back(*it++);
            }
            else
            {
                pSecParam->endPoint.emplace_back(U[nodeIndex1]);
            }
        }
        else
        {
            //in this condition it point to the second end of the next segment
            ++it;
            pSecParam->endPoint.emplace_back(*(it++));
        }

        while (U[nodeIndex2] - *it > FLT_MIN && it < nurbsParam.endPoint.end())
        {
            //in this condition it point to the second end of the next segment
            pSecParam->endPoint.emplace_back(*(it++));
            pSecParam->endPoint.emplace_back(*(it++));
        }
        --it;  // it point to the first end of the current segment
        if (*it - U[nodeIndex2] < FLT_MIN)
        {
            pSecParam->endPoint.emplace_back(U[nodeIndex2]);
        }
        else
        {
            pSecParam->endPoint.pop_back();
        }
        float32_t coeff(0);
        auto itSecParam = pSecParam->endPoint.begin();
        while (itSecParam < pSecParam->endPoint.end())
        {
            coeff -= *(itSecParam++);
            coeff += *(itSecParam++);
        }

        pSecParam->paintTotalLength = nurbsParam.paintTotalLength / paramLength * coeff;
        pSecParam->lineLength = nurbsParam.lineLength * (pSecParam->endPoint.back() - pSecParam->endPoint.front());
    }
    return true;
}

/**
* This function is to slice the NURBS curve into some segments.
*
* @param nurbsParam        [ IN] parameters of input NURBS curve
* @param vNode             [ IN] the points in which neighborhood is the break point.
* @param vSegType          [OUT] the slice type of the sections
* @param vSecParam         [OUT] parameters of output NRUBS curve
* @param vSectionPoints    [OUT] point cloud of the output NURBS
*
* @return true on success, false otherwise
*/
#if 0
template <class Point3_T>
bool segmentByPoints(const roadDBCore::NURBS_t          &nurbsParam,
                     const std::vector<Point3_T>        &vNode,
                     std::vector<roadDBCore::NURBS_t>   *pSecParam,
                     std::vector<std::vector<Point3_T>> *pSectionPoints = nullptr,
                     std::vector<Point3_T>              *pSegPoint = nullptr) const
{
    bool bRet = true;
    if (vNode.empty() || (vNode.size() & 0x1) != 0 ||
        nurbsParam.vecCtrlPoint.empty() || nurbsParam.vecKnot.empty() || nurbsParam.endPoint.empty())
    {
        SDOR_LOG_WARN << "The input data are empty.";
        bRet = false;
    }
    else
    {
        float32_t paramLength(0);
        auto it = nurbsParam.endPoint.begin();
        while (it < nurbsParam.endPoint.end())
        {
            paramLength -= *it++;
            paramLength += *it++;
        }

        const float32_t step = 0.1f;
        std::vector<float32_t> U;
        std::vector<Point3_T> vPoint;
        std::vector<size_t> vRange;

        std::vector<int32_t> vNodeIndex, vSecIndex;
        std::vector<TRUNCATE_TYPE_E> vSegType;
        bRet = generateCurveAndRegion(nurbsParam, step, vPoint, vRange, U) &&
               searchNodeIndexInMerge(vNode, vPoint, vSecIndex, vNodeIndex, vSegType);

        if (vSecIndex.size() != (vNode.size() >> 1) || vNode.size() != vNodeIndex.size())
        {
            SDOR_LOG_WARN << "There are some invalid nodes.";
            bRet = false;
        }
        else
        {
            for (auto it1 = vNodeIndex.begin(), it2 = it1 + 1; it2 < vNodeIndex.end() && bRet; ++it1, ++it2)
            {
                if (*it2 < *it1)
                {
                    SDOR_LOG_WARN << "There are some invalid nodes.";
                    bRet = false;
                }
            }
        }

        if (bRet && pSectionPoints != nullptr)
        {
            pSectionPoints->clear();
            pSectionPoints->resize(vSecIndex.size());

            int i(0);
            for (auto it1 = vNodeIndex.begin(), it2 = it1 + 1; it2 < vNodeIndex.end(); ++it1, ++it2)
            {

            }
        }


        std::pair<Point3_T, Point3_T> tmpPair;

        //index1 is the index of the first end point of the first section
        //index2 is the index of the second end point of the first section
        size_t index1 = vNodeIndex[vSecSort.front()], index2 = vNodeIndex[vSecSort.front() + 1];
        Point3_T tmpPoint = vPoint[index1];
        for (int32_t i = 1, iEnd = static_cast<int32_t>(vSecSort.size()); i < iEnd; ++i)
        {
            tmpPair.first = tmpPoint;
            tmpPoint = vPoint[index2];
            tmpPair.second = tmpPoint;
            vSegPoint.emplace_back(tmpPair);

            index2 = vNodeIndex[vSecSort[i] + 1];
        }

        tmpPair.first = tmpPoint;
//             index2 = vNodeIndex[vSecSort.back()] + 1;
        tmpPair.second = vPoint[index2];
        vSegPoint.emplace_back(tmpPair);

//             SDOR_LOG_DEBUG << "vSecSort.size() = " << vSecSort.size();
//             SDOR_LOG_DEBUG << "vSegPoint.size() = " << vSegPoint.size();

        vSegSortPoint.clear();
        vSegSortPoint.resize(vSegPoint.size());
        size_t index;
        for (size_t i = 0; i < vSecSort.size(); ++i)
        {
            index = vSecSort[i];
            vSegSortPoint[index >> 1] = vSegPoint[i];
        }

        if (vSegType.size() == 1u)
        {
            vSegType[vSecSort.front() >> 1] = TRUNCATE_TYPE_NO_E;
        }
        else
        {
            vSegType[vSecSort.front() >> 1] = TRUNCATE_TYPE_SECOND_E;
            vSegType[vSecSort.back() >> 1] = TRUNCATE_TYPE_FIRST_E;
        }
    }

    size_t CPIndex1, CPIndex2;
    size_t nodeIndex1, nodeIndex2;
    auto itCP = nurbsParam.vecCtrlPoint.begin();
    auto itKnot = nurbsParam.vecKnot.begin();

    vSecParam.resize(vSecIndex.size());
    vSectionPoints.resize(vSecIndex.size());

    for (size_t i = 0, j = 0; i < vSecParam.size(); ++i, j += 2)
    {
        nodeIndex1 = vNodeIndex[j];
        nodeIndex2 = vNodeIndex[j+1];

        //this code is bad, we will improve it later.
        auto u = U[nodeIndex2];
        //first point to the second point of a segment
        //second point to the first point of the next segment
        auto first = nurbsParam.endPoint.begin() + 1, second = first + 1;
        for (; second < nurbsParam.endPoint.end(); first += 2, second += 2)
        {
            if (u - *first > FLT_EPSILON && u - *second < -FLT_EPSILON)
            {
                while((u - *second < -FLT_EPSILON) && nodeIndex2+1 < U.size())
                {
                    u = U[++nodeIndex2];
                }
                break;
            }
        }

        vSectionPoints[i].insert(vSectionPoints[i].end(), vPoint.begin() + nodeIndex1, vPoint.begin() + nodeIndex2 + 1);

        CPIndex1 = vRange[nodeIndex1] - 2;
        CPIndex2 = vRange[nodeIndex2];

#if 0
        SDOR_LOG_DEBUG << "CPIndex1 = " << CPIndex1;
        SDOR_LOG_DEBUG << "CPIndex2 = " << CPIndex2;
#endif

        auto pSecParam = &vSecParam[i];
        pSecParam->vecCtrlPoint.insert(vSecParam[i].vecCtrlPoint.end(), itCP + CPIndex1, itCP + CPIndex2 + 1);
        pSecParam->vecKnot.insert(vSecParam[i].vecKnot.end(), itKnot + CPIndex1, itKnot + CPIndex2 + 4);

        //search the first segment
        auto it = nurbsParam.endPoint.begin();
        while (U[nodeIndex1] - *it > -FLT_EPSILON && it < nurbsParam.endPoint.end())
        {
            it += 2; // it point to the first end of the next segment
        }
        --it;  // it point the second end of the previous segment
        if (U[nodeIndex1] - *it < FLT_EPSILON)
        {
            if ((*it) - U[nodeIndex2] < -FLT_EPSILON)
            {
                //in this condition it point the second end of the next segment
                pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                pSecParam->endPoint.emplace_back(*it++);
                pSecParam->endPoint.emplace_back(*it++);
            }
            else
            {
                pSecParam->endPoint.emplace_back(U[nodeIndex1]);
            }
        }
        else
        {
            //in this condition it point to the second end of the next segment
            ++it;
            pSecParam->endPoint.emplace_back(*(it++));
        }

        while (U[nodeIndex2] - *it > FLT_EPSILON && it < nurbsParam.endPoint.end())
        {
            //in this condition it point to the second end of the next segment
            pSecParam->endPoint.emplace_back(*(it++));
            pSecParam->endPoint.emplace_back(*(it++));
        }
        --it;  // it point to the first end of the current segment
        if (*it - U[nodeIndex2] < FLT_EPSILON)
        {
            pSecParam->endPoint.emplace_back(U[nodeIndex2]);
        }
        else
        {
            pSecParam->endPoint.pop_back();
        }
        float32_t coeff(0);
        auto itSecParam = pSecParam->endPoint.begin();
        while (itSecParam < pSecParam->endPoint.end())
        {
            coeff -= *(itSecParam++);
            coeff += *(itSecParam++);
        }

        if (paramLength < FLT_EPSILON)
        {
            SDOR_LOG_WARN <<"coeff or paramLength size is 0";
            return false;
        }

        pSecParam->paintTotalLength = nurbsParam.paintTotalLength / paramLength * coeff;
        pSecParam->lineLength = nurbsParam.lineLength * (pSecParam->endPoint.back() - pSecParam->endPoint.front());
    }
    return true;
}
#endif

/**
* This function is to slice the NURBS curve into some segments.
*
* @param nurbsParam        [ IN] parameters of input NURBS curve
* @param vNode             [ IN] the points in which neighborhood is the break point.
* @param vSegType          [OUT] the slice type of the sections
* @param vSecParam         [OUT] parameters of output NRUBS curve
* @param vSectionPoints    [OUT] point cloud of the output NURBS
*
* @return true on success, false otherwise
*/
template <class Point3_T>
bool segmentInsideOfSec(const roadDBCore::NURBS_t                   &nurbsParam,
                            const std::vector<Point3_T>                 &vNode,
                            const std::vector<int32_t>                  &vSecIndex,
                            std::vector<roadDBCore::NURBS_t>            &vSecParam,
                            std::vector<std::vector<Point3_T>>          &vSectionPoints,
                            std::vector<std::pair<Point3_T, Point3_T>>  &vSegSortPoint,
                            std::vector<TRUNCATE_TYPE_E>                &vSegType) const
{
    MONITOR_FUNCTION_PERFORMANCE("NURBS")

    if (vNode.empty() || nurbsParam.vecCtrlPoint.empty() || nurbsParam.vecKnot.empty() ||
        nurbsParam.endPoint.empty() || (nurbsParam.endPoint.size() & 0x1) != 0)
    {
        SDOR_LOG_WARN << "The input data are invalid.";
        SDOR_LOG_INFO << "vNode.size() = " << vNode.size();
        SDOR_LOG_INFO << "vSecIndex.size() = " << vSecIndex.size();
        SDOR_LOG_INFO << "number of control points = " << nurbsParam.vecCtrlPoint.size();
        SDOR_LOG_INFO << "nurbsParam.vecKnot.size() = " << nurbsParam.vecKnot.size();
        SDOR_LOG_INFO << "nurbsParam.endPoint.size() = " << nurbsParam.endPoint.size();
        return false;
    }

    float32_t paramLength(0);
    auto it = nurbsParam.endPoint.begin();
    while (it < nurbsParam.endPoint.end())
    {
        paramLength -= *it++;
        paramLength += *it++;
    }

    if (paramLength < FLT_EPSILON)
    {
        SDOR_LOG_WARN << "the length of the line is zero or negative number.";
        return false;
    }

    const float32_t step = 0.1f;
    std::vector<float32_t> U;
    std::vector<Point3_T> vPoint;
    std::vector<size_t> vRange;

    if (!generateCurveAndRegion(nurbsParam, step, vPoint, vRange, U))
    {
        SDOR_LOG_WARN << "The function generateCurveAndRegion return false.";
        return false;
    }

    std::vector<int32_t> vNodeIndex;
    if (!searchNodeIndexInMerge(vNode, vPoint, vSecIndex, vNodeIndex, vSegType))
    {
        SDOR_LOG_WARN << "Data error.";
        return false;
    }

    std::vector<std::pair<Point3_T, Point3_T>> vSegPoint;
    if (!vSecIndex.empty())
    {
        vSegPoint.reserve(vSecIndex.size());
        std::vector<size_t> vSecSort;
        for (size_t i = 0; i < vNodeIndex.size(); i += 2)
        {
            vSecSort.emplace_back(i);
        }

        std::stable_sort(vSecSort.begin(), vSecSort.end(),
                         [vNodeIndex](const size_t &i, const size_t &j)
                         {
                             return (vNodeIndex[i] < vNodeIndex[j]);
                         });

        std::pair<Point3_T, Point3_T> tmpPair;

        //index1 is the index of the first end point of the first section
        //index2 is the index of the second end point of the first section
        size_t index1 = vNodeIndex[vSecSort.front()], index2 = vNodeIndex[vSecSort.front() + 1];
        Point3_T tmpPoint = vPoint[index1];
        for (int32_t i = 1, iEnd = static_cast<int32_t>(vSecSort.size()); i < iEnd; ++i)
        {
            tmpPair.first = tmpPoint;
            //index1 is the index of the first end point of the next section
//                 index1 = vNodeIndex[vSecSort[i]];
            //tmpPoint is the average of second end point of current section
            //and the first end point of the next section
//                 tmpPoint = (vPoint[index2] + vPoint[index1]) * 0.5f;
            tmpPoint = vPoint[index2];
            tmpPair.second = tmpPoint;
            vSegPoint.emplace_back(tmpPair);

            index2 = vNodeIndex[vSecSort[i] + 1];
        }

        tmpPair.first = tmpPoint;
//             index2 = vNodeIndex[vSecSort.back()] + 1;
        tmpPair.second = vPoint[index2];
        vSegPoint.emplace_back(tmpPair);

//             SDOR_LOG_DEBUG << "vSecSort.size() = " << vSecSort.size();
//             SDOR_LOG_DEBUG << "vSegPoint.size() = " << vSegPoint.size();

        vSegSortPoint.clear();
        vSegSortPoint.resize(vSegPoint.size());
        size_t index;
        for (size_t i = 0; i < vSecSort.size(); ++i)
        {
            index = vSecSort[i];
            vSegSortPoint[index >> 1] = vSegPoint[i];
        }

        if (vSegType.size() == 1u)
        {
            vSegType[vSecSort.front() >> 1] = TRUNCATE_TYPE_NO_E;
        }
        else
        {
            vSegType[vSecSort.front() >> 1] = TRUNCATE_TYPE_SECOND_E;
            vSegType[vSecSort.back() >> 1] = TRUNCATE_TYPE_FIRST_E;
        }
    }

    size_t CPIndex1, CPIndex2;
    size_t nodeIndex1, nodeIndex2;
    auto itCP = nurbsParam.vecCtrlPoint.begin();
    auto itKnot = nurbsParam.vecKnot.begin();

    vSecParam.resize(vSecIndex.size());
    vSectionPoints.resize(vSecIndex.size());

    for (size_t i = 0, j = 0; i < vSecParam.size(); ++i, j += 2)
    {
        nodeIndex1 = vNodeIndex[j];
        nodeIndex2 = vNodeIndex[j+1];

//        //this code is bad, we will improve it later.
//        auto u = U[nodeIndex2];
//        //first point to the second point of a segment
//        //second point to the first point of the next segment
//        auto first = nurbsParam.endPoint.begin() + 1, second = first + 1;
//        for (; second < nurbsParam.endPoint.end(); first += 2, second += 2)
//        {
//            if (u - *first > FLT_EPSILON && u - *second < -FLT_EPSILON)
//            {
//                while((u - *second < -FLT_EPSILON) && nodeIndex2+1 < U.size())
//                {
//                    u = U[++nodeIndex2];
//                }
//                break;
//            }
//        }

        vSectionPoints[i].insert(vSectionPoints[i].end(), vPoint.begin() + nodeIndex1, vPoint.begin() + nodeIndex2 + 1);

        CPIndex1 = vRange[nodeIndex1] - 2;
        CPIndex2 = vRange[nodeIndex2];

#if 0
        SDOR_LOG_DEBUG << "CPIndex1 = " << CPIndex1;
        SDOR_LOG_DEBUG << "CPIndex2 = " << CPIndex2;
#endif

        auto pSecParam = &vSecParam[i];
        pSecParam->vecCtrlPoint.insert(vSecParam[i].vecCtrlPoint.end(), itCP + CPIndex1, itCP + CPIndex2 + 1);
        pSecParam->vecKnot.insert(vSecParam[i].vecKnot.end(), itKnot + CPIndex1, itKnot + CPIndex2 + 4);

        //search the first segment
        auto it = nurbsParam.endPoint.begin();
        while (U[nodeIndex1] - *it > -FLT_MIN && it < nurbsParam.endPoint.end())
        {
            it += 2; // it point to the first end of the next segment
        }
        --it;  // it point the second end of the previous segment
        if (U[nodeIndex1] - *it < FLT_MIN)
        {
            if ((*it) - U[nodeIndex2] < -FLT_MIN)
            {
                //in this condition it point the second end of the next segment
                pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                pSecParam->endPoint.emplace_back(*it++);
                pSecParam->endPoint.emplace_back(*it++);
            }
            else
            {
                pSecParam->endPoint.emplace_back(U[nodeIndex1]);
            }
        }
        else
        {
            //in this condition it point to the second end of the next segment
            ++it;
            pSecParam->endPoint.emplace_back(*(it++));
        }

        while (U[nodeIndex2] - *it > FLT_MIN && it < nurbsParam.endPoint.end())
        {
            //in this condition it point to the second end of the next segment
            pSecParam->endPoint.emplace_back(*(it++));
            pSecParam->endPoint.emplace_back(*(it++));
        }
        --it;  // it point to the first end of the current segment
        if (*it - U[nodeIndex2] < FLT_MIN)
        {
            pSecParam->endPoint.emplace_back(U[nodeIndex2]);
        }
        else
        {
            pSecParam->endPoint.pop_back();
        }
        float32_t coeff(0);
        auto itSecParam = pSecParam->endPoint.begin();
        while (itSecParam < pSecParam->endPoint.end())
        {
            coeff -= *(itSecParam++);
            coeff += *(itSecParam++);
        }

        pSecParam->paintTotalLength = nurbsParam.paintTotalLength / paramLength * coeff;
        pSecParam->lineLength = nurbsParam.lineLength * (pSecParam->endPoint.back() - pSecParam->endPoint.front());
    }
    return true;
}

/**
     * This function is to slice the NURBS curve into some segments.
     *
     * @param nurbsParam        [ IN] parameters of input NURBS curve
     * @param vNode             [ IN] the points in which neighborhood is the break point.
     * @param vSegType          [OUT] the slice type of the sections
     * @param vSecParam         [OUT] parameters of output NRUBS curve
     * @param vSectionPoints    [OUT] point cloud of the output NURBS
     *
     * @return true on success, false otherwise
     */
    template <class Point3_T>
    bool segmentInAssortedSecByRef(const roadDBCore::NURBS_t                   &nurbsParam,
                                   const std::vector<Point3_T>                 &vNode,
                                   const std::vector<int32_t>                  &vSecIndex,
                                   const std::vector<std::vector<Point3_T>>    &vReference,
                                   const std::vector<std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXY>>> &vpKdtree,
                                   std::vector<roadDBCore::NURBS_t>            &vSecParam,
                                   std::vector<std::vector<Point3_T>>          &vSectionPoints,
                                   std::vector<std::pair<Point3_T, Point3_T>>  &vSegSortPoint,
                                   std::vector<TRUNCATE_TYPE_E>                &vSegType) const
    {
        if (vNode.empty() || nurbsParam.vecCtrlPoint.empty() || nurbsParam.vecKnot.empty() ||
            nurbsParam.endPoint.empty() || (nurbsParam.endPoint.size() & 0x1) != 0)
        {
            SDOR_LOG_WARN << "The input data are empty.";
            SDOR_LOG_INFO << "vNode.size() = " << vNode.size();
            SDOR_LOG_INFO << "vSecIndex.size() = " << vSecIndex.size();
            SDOR_LOG_INFO << "number of control points = " << nurbsParam.vecCtrlPoint.size();
            SDOR_LOG_INFO << "nurbsParam.vecKnot.size() = " << nurbsParam.vecKnot.size();
            SDOR_LOG_INFO << "nurbsParam.endPoint.size() = " << nurbsParam.endPoint.size();
            return false;
        }

        float32_t paramLength(0);
        auto it = nurbsParam.endPoint.begin();
        while (it < nurbsParam.endPoint.end())
        {
            paramLength -= *it++;
            paramLength += *it++;
        }

        if (paramLength < FLT_EPSILON)
        {
            SDOR_LOG_WARN << "the length of the line is zero or negative.";
            return false;
        }

        const float32_t step = 0.2f;
        std::vector<float32_t> U;
        std::vector<Point3_T> vPoint;
        std::vector<size_t> vRange;

        if (!generateCurveAndRegion(nurbsParam, step, vPoint, vRange, U))
        {
            SDOR_LOG_WARN << "The function generateCurveAndRegion return false.";
            return false;
        }

        std::vector<int32_t> vNodeIndex;
        if (!searchNodeIndexInMergeByRef(vNode, vPoint, vSecIndex, vReference, vpKdtree, vNodeIndex))
        {
            SDOR_LOG_WARN << "Data error.";
            return false;
        }

#if 0
        {
            SDOR_LOG_DEBUG << "second vNodeIndex.size() = " << vNodeIndex.size();
            std::ostringstream outString;
            for (auto &i : vNodeIndex)
            {
                outString << i << ", ";
            }
            SDOR_LOG_DEBUG << outString;
        }
#endif

        vSegSortPoint.clear();
        if (!vSecIndex.empty())
        {
            vSegSortPoint.reserve(vSecIndex.size());

            std::pair<Point3_T, Point3_T> tmpPair;

            //index1 is the index of the first end point of the first section
            //index2 is the index of the second end point of the first section
            size_t index1 = vNodeIndex[0], index2 = vNodeIndex[1];
            Point3_T tmpPoint = vPoint[index1];
            for (int32_t i = 1, iEnd = static_cast<int32_t>(vSecIndex.size()); i < iEnd; ++i)
            {
                tmpPair.first = tmpPoint;
                tmpPoint = vPoint[index2];
                tmpPair.second = tmpPoint;
                vSegSortPoint.emplace_back(tmpPair);
                index2 = vNodeIndex[2*i + 1];
            }

            tmpPair.first = tmpPoint;
            tmpPair.second = vPoint[index2];
            vSegSortPoint.emplace_back(tmpPair);

            if (vSecIndex.size() == 1u)
            {
                vSegType.resize(1, TRUNCATE_TYPE_NO_E);
            }
            else
            {
                vSegType.resize(vSecIndex.size(), TRUNCATE_TYPE_BOTH_E);
                vSegType.front() = TRUNCATE_TYPE_SECOND_E;
                vSegType.back() = TRUNCATE_TYPE_FIRST_E;
            }
        }

        size_t CPIndex1, CPIndex2;
        size_t nodeIndex1, nodeIndex2;
        auto itCP = nurbsParam.vecCtrlPoint.begin();
        auto itKnot = nurbsParam.vecKnot.begin();

        vSecParam.resize(vSecIndex.size());
        vSectionPoints.resize(vSecIndex.size());

        for (size_t i = 0, j = 0; i < vSecParam.size(); ++i, j += 2)
        {
            nodeIndex1 = vNodeIndex[j];
            nodeIndex2 = vNodeIndex[j+1];

            vSectionPoints[i].assign(vPoint.begin() + nodeIndex1, vPoint.begin() + nodeIndex2 + 1);

            CPIndex1 = vRange[nodeIndex1] - 2;
            CPIndex2 = vRange[nodeIndex2];

            auto pSecParam = &vSecParam[i];
            pSecParam->vecCtrlPoint.assign(itCP + CPIndex1, itCP + CPIndex2 + 1);
            pSecParam->vecKnot.assign(itKnot + CPIndex1, itKnot + CPIndex2 + 4);

                //search the first segment
            auto it = nurbsParam.endPoint.begin();
            while (U[nodeIndex1] - *it > -FLT_MIN && it < nurbsParam.endPoint.end())
            {
                it += 2; // it point to the first end of the next segment
            }
            --it;  // it point the second end of the previous segment
            if (U[nodeIndex1] - *it < FLT_MIN)
            {
                if ((*it) - U[nodeIndex2] < -FLT_MIN)
                {
                    //in this condition it point to the second end of the next segment
                    pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                    pSecParam->endPoint.emplace_back(*it++);
                    pSecParam->endPoint.emplace_back(*it++);
                }
                else
                {
                    pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                }
            }
            else
            {
                //in this condition it point to the second end of the next segment
                //add a new segment with length that equal to zero
                pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                ++it;
                pSecParam->endPoint.emplace_back(*(it++));
            }

            while (U[nodeIndex2] - *it > FLT_MIN && it < nurbsParam.endPoint.end())
            {
                //in this condition it point to the second end of the next segment
                pSecParam->endPoint.emplace_back(*(it++));
                pSecParam->endPoint.emplace_back(*(it++));
            }
            --it;  // it point to the first end of the current segment
            if (*it - U[nodeIndex2] < FLT_MIN)
            {
                pSecParam->endPoint.emplace_back(U[nodeIndex2]);
            }
            else
            {
                pSecParam->endPoint.pop_back();
                //add a new segment with length that equal to zero
                pSecParam->endPoint.emplace_back(U[nodeIndex2]);
                pSecParam->endPoint.emplace_back(U[nodeIndex2]);
            }

            float32_t coeff(0);
            auto itSecParam = pSecParam->endPoint.begin();
            while (itSecParam < pSecParam->endPoint.end())
            {
                coeff -= *(itSecParam++);
                coeff += *(itSecParam++);
            }

            pSecParam->paintTotalLength = nurbsParam.paintTotalLength / paramLength * coeff;

            if (pSecParam->paintTotalLength < FLT_EPSILON)
            {
                SDOR_LOG_DEBUG << "nurbsParam.paintTotalLength = " << nurbsParam.paintTotalLength;
                SDOR_LOG_DEBUG << "paramLength = " << paramLength;
                SDOR_LOG_DEBUG << "coeff = " << coeff;

                std::ostringstream outString;
                for (auto &f : nurbsParam.endPoint)
                {
                    outString << f << ", ";
                }
                outString << std::endl;
                outString << U[nodeIndex1] << ", " << U[nodeIndex2] << std::endl;
                SDOR_LOG_INFO << outString;
            }

            pSecParam->lineLength = nurbsParam.lineLength * (pSecParam->endPoint.back() - pSecParam->endPoint.front());
        }
        return true;
    }

    /**
     * This function is to slice the NURBS curve into some segments.
     *
     * @param nurbsParam        [ IN] parameters of input NURBS curve
     * @param vNode             [ IN] the points in which neighborhood is the break point.
     * @param vSegType          [OUT] the slice type of the sections
     * @param vSecParam         [OUT] parameters of output NRUBS curve
     * @param vSectionPoints    [OUT] point cloud of the output NURBS
     *
     * @return true on success, false otherwise
     */
    template <class Point3_T>
    bool segmentInAssortedSecEx(const roadDBCore::NURBS_t                   &nurbsParam,
                                const std::vector<Point3_T>                 &vNode,
                                const std::vector<int32_t>                  &vSecIndex,
                                std::vector<roadDBCore::NURBS_t>            &vSecParam,
                                std::vector<std::vector<Point3_T>>          &vSectionPoints,
                                std::vector<std::pair<Point3_T, Point3_T>>  &vSegSortPoint,
                                std::vector<TRUNCATE_TYPE_E>                &vSegType) const
    {
        MONITOR_FUNCTION_PERFORMANCE("NURBS")

        if (vNode.empty() || nurbsParam.vecCtrlPoint.empty() || nurbsParam.vecKnot.empty() ||
            nurbsParam.endPoint.empty() || (nurbsParam.endPoint.size() & 0x1) != 0 ||
            static_cast<size_t>(vSecIndex.size() << 1) > vNode.size())
        {
            SDOR_LOG_WARN << "The input data are invalid.";
            SDOR_LOG_INFO << "vNode.size() = " << vNode.size();
            SDOR_LOG_INFO << "vSecIndex.size() = " << vSecIndex.size();
            SDOR_LOG_INFO << "number of control points = " << nurbsParam.vecCtrlPoint.size();
            SDOR_LOG_INFO << "nurbsParam.vecKnot.size() = " << nurbsParam.vecKnot.size();
            SDOR_LOG_INFO << "nurbsParam.endPoint.size() = " << nurbsParam.endPoint.size();
            return false;
        }

        float32_t paramLength(0);
        auto it = nurbsParam.endPoint.begin();
        while (it < nurbsParam.endPoint.end())
        {
            paramLength -= *it++;
            paramLength += *it++;
        }

        if (paramLength < FLT_EPSILON)
        {
            SDOR_LOG_WARN << "the length of the line is zero of negative number.";
            return false;
        }

        const float32_t step = 0.2f;
        std::vector<float32_t> U;
        std::vector<Point3_T> vPoint;
        std::vector<size_t> vRange;

        if (!generateCurveAndRegion(nurbsParam, step, vPoint, vRange, U))
        {
            SDOR_LOG_WARN << "The function generateCurveAndRegion return false.";
            return false;
        }

        std::vector<int32_t> vNodeIndex;
        if (!searchNodeIndexInMerge(vNode, vPoint, vSecIndex, vNodeIndex, vSegType))
        {
            SDOR_LOG_WARN << "Data error.";
            return false;
        }

        std::vector<std::pair<Point3_T, Point3_T>> vSegPoint;
        if (!vSecIndex.empty())
        {
            vSegPoint.reserve(vSecIndex.size());
            std::vector<size_t> vSecSort;
            for (size_t i = 0; i < vNodeIndex.size(); i += 2)
            {
                vSecSort.emplace_back(i);
            }

            std::stable_sort(vSecSort.begin(), vSecSort.end(),
                             [vNodeIndex](const size_t &i, const size_t &j)
                             {
                                 return (vNodeIndex[i] < vNodeIndex[j]);
                             });

            std::pair<Point3_T, Point3_T> tmpPair;

            //index1 is the index of the first end point of the first section
            //index2 is the index of the second end point of the first section
            size_t index1 = vNodeIndex[vSecSort.front()], index2 = vNodeIndex[vSecSort.front() + 1];
            Point3_T tmpPoint = vPoint[index1];
            for (int32_t i = 1, iEnd = static_cast<int32_t>(vSecSort.size()); i < iEnd; ++i)
            {
                tmpPair.first = tmpPoint;
                tmpPoint = vPoint[index2];
                tmpPair.second = tmpPoint;
                vSegPoint.emplace_back(tmpPair);

                index2 = vNodeIndex[vSecSort[i] + 1];
            }

            tmpPair.first = tmpPoint;
            tmpPair.second = vPoint[index2];
            vSegPoint.emplace_back(tmpPair);

            vSegSortPoint.clear();
            vSegSortPoint.resize(vSegPoint.size());
            size_t index;
            for (size_t i = 0; i < vSecSort.size(); ++i)
            {
                index = vSecSort[i];
                vSegSortPoint[index >> 1] = vSegPoint[i];
            }

            if (vSegType.size() == 1u)
            {
                vSegType[vSecSort.front() >> 1] = TRUNCATE_TYPE_NO_E;
            }
            else
            {
                vSegType[vSecSort.front() >> 1] = TRUNCATE_TYPE_SECOND_E;
                vSegType[vSecSort.back() >> 1] = TRUNCATE_TYPE_FIRST_E;
            }
        }

        size_t CPIndex1, CPIndex2;
        size_t nodeIndex1, nodeIndex2;
        auto itCP = nurbsParam.vecCtrlPoint.begin();
        auto itKnot = nurbsParam.vecKnot.begin();

        vSecParam.resize(vSecIndex.size());
        vSectionPoints.resize(vSecIndex.size());

        for (size_t i = 0, j = 0; i < vSecParam.size(); ++i, j += 2)
        {
            nodeIndex1 = vNodeIndex[j];
            nodeIndex2 = vNodeIndex[j+1];

            vSectionPoints[i].assign(vPoint.begin() + nodeIndex1, vPoint.begin() + nodeIndex2 + 1);

            CPIndex1 = vRange[nodeIndex1] - 2;
            CPIndex2 = vRange[nodeIndex2];

            auto pSecParam = &vSecParam[i];
            pSecParam->vecCtrlPoint.assign(itCP + CPIndex1, itCP + CPIndex2 + 1);
            pSecParam->vecKnot.assign(itKnot + CPIndex1, itKnot + CPIndex2 + 4);

            //search the first segment
            auto it = nurbsParam.endPoint.begin();
            while (U[nodeIndex1] - *it > -FLT_MIN && it < nurbsParam.endPoint.end())
            {
                it += 2; // it point to the first end of the next segment
            }
            --it;  // it point the second end of the previous segment
            if (U[nodeIndex1] - *it < FLT_MIN)
            {
                if ((*it) - U[nodeIndex2] < -FLT_MIN)
                {
                    //in this condition it point to the second end of the next segment
                    pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                    pSecParam->endPoint.emplace_back(*it++);
                    pSecParam->endPoint.emplace_back(*it++);
                }
                else
                {
                    pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                }
            }
            else
            {
                //in this condition it point to the second end of the next segment
                //add a new segment with length that equal to zero
                pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                pSecParam->endPoint.emplace_back(U[nodeIndex1]);
                ++it;
                pSecParam->endPoint.emplace_back(*(it++));
            }

            while (U[nodeIndex2] - *it > FLT_MIN && it < nurbsParam.endPoint.end())
            {
                //in this condition it point to the second end of the next segment
                pSecParam->endPoint.emplace_back(*(it++));
                pSecParam->endPoint.emplace_back(*(it++));
            }
            --it;  // it point to the first end of the current segment
            if (*it - U[nodeIndex2] < FLT_MIN)
            {
                pSecParam->endPoint.emplace_back(U[nodeIndex2]);
            }
            else
            {
                pSecParam->endPoint.pop_back();
                //add a new segment with length that equal to zero
                pSecParam->endPoint.emplace_back(U[nodeIndex2]);
                pSecParam->endPoint.emplace_back(U[nodeIndex2]);
            }

            float32_t coeff(0);
            auto itSecParam = pSecParam->endPoint.begin();
            while (itSecParam < pSecParam->endPoint.end())
            {
                coeff -= *(itSecParam++);
                coeff += *(itSecParam++);
            }

            pSecParam->paintTotalLength = nurbsParam.paintTotalLength / paramLength * coeff;

            if (pSecParam->paintTotalLength < FLT_EPSILON)
            {
                SDOR_LOG_DEBUG << "nurbsParam.paintTotalLength = " << nurbsParam.paintTotalLength;
                SDOR_LOG_DEBUG << "paramLength = " << paramLength;
                SDOR_LOG_DEBUG << "coeff = " << coeff;

                {
                    std::ostringstream outString;
                    SDOR_LOG_DEBUG << "nurbsParam.endPoint.size() = " << nurbsParam.endPoint.size();
                    for (auto &f : nurbsParam.endPoint)
                    {
                        outString << f << ", ";
                    }
                    SDOR_LOG_DEBUG << outString;
                }

                {
                    std::ostringstream outString;
                    outString << U[nodeIndex1] << ", " << U[nodeIndex2] << std::endl;
                    SDOR_LOG_DEBUG << outString;
                }
            }

            pSecParam->lineLength = nurbsParam.lineLength * (pSecParam->endPoint.back() - pSecParam->endPoint.front());
        }
        return true;
    }



    bool getFirstAndLastPoint(const roadDBCore::NURBS_t &curveParam,
                              Point3f_t &firstPoint,
                              Point3f_t &lastPoint) const
    {
        if (!curveParam.endPoint.empty() && !curveParam.vecKnot.empty())
        {
            std::vector<float32_t> U;
            U.emplace_back(curveParam.endPoint.front());
            U.emplace_back(curveParam.endPoint.back());
            std::vector<cv::Point3f> outPoints;
            outSample(curveParam, U, outPoints);

            const auto &point1 = outPoints.front();
            const auto &point2 = outPoints.back();

            firstPoint.relLon = point1.x;
            firstPoint.relAlt = point1.y;
            firstPoint.relLat = point1.z;

            lastPoint.relLon = point2.x;
            lastPoint.relAlt = point2.y;
            lastPoint.relLat = point2.z;

            return true;
        }
        else
        {
            SDOR_LOG_WARN << "The input data are invalid.";
            return false;
        }
    }


    template <class PointXYZ>
    bool getEndPoints(const roadDBCore::NURBS_t &curveParam,
                      std::vector<PointXYZ> &vecEndPoints) const
    {
        bool bRet = true;
        if (!curveParam.endPoint.empty() && !curveParam.vecKnot.empty())
        {
            bRet = outSample(curveParam, curveParam.endPoint, vecEndPoints);
        }
        else
        {
            SDOR_LOG_WARN << "The input data are invalid.";
            bRet = false;
        }
        return bRet;
    }

    template <class Point3_T>
    bool subSampleByDistance(const std::vector<Point3_T> &vInPoints,
                             const float maxDistance,
                             std::vector<int> &vIndex)
    {
        bool bRet = false;
        if (!vInPoints.empty() && maxDistance > FLT_EPSILON)
        {
            vIndex.clear();
            float distance(0);

            int i = 0;
            for (auto first = vInPoints.begin(), second = first + 1; second < vInPoints.end(); ++first, ++second, ++i)
            {
                distance += normXYZ(*first - *second);

                if (distance > maxDistance)
                {
                    vIndex.emplace_back(i);
                    distance = 0.f;
                }
            }

            bRet = true;
        }

        return bRet;
    }



template <class Point3_T>
bool fitAccuracyCurve(const std::vector<Point3_T> &inputPoints,
                      const std::vector<std::pair<int32_t, int32_t>> &vecEndIndex,
                      const float &maxError,
                      roadDBCore::NURBS_t &curveParam)
{
    MONITOR_FUNCTION_PERFORMANCE("NURBS")

    bool bRet = true;
    if (inputPoints.size() < minNumPoint_)
    {
        SDOR_LOG_WARN << "The number of points is too less.";
        bRet = false;
    }
    else if (vecEndIndex.empty())
    {
        SDOR_LOG_WARN << "The end index is empty.";
        bRet = false;
    }
    else if (!configPara_.isValid())
    {
        SDOR_LOG_WARN << "Configuration parameters are invalid.";
        bRet = false;
    }
    else if (maxError < FLT_EPSILON)
    {
        SDOR_LOG_WARN << "maxError is too less.";
        bRet = false;
    }
    else
    {
        numPointsToFit_ = static_cast<int32_t>(inputPoints.size());
        for (auto it = vecEndIndex.begin(); it < vecEndIndex.end() && bRet; ++it)
        {
            if (it->first < 0 || it->first >= numPointsToFit_ ||
                it->second < 0 || it->second >= numPointsToFit_ ||
                it->first > it->second)
            {
                SDOR_LOG_WARN << "The end index is invalid.";
                SDOR_LOG_DEBUG << "vecEndIndex.size() = " << vecEndIndex.size();
                bRet = false;
            }
        }

        for (auto it1 = vecEndIndex.begin(), it2 = it1 + 1; it2 < vecEndIndex.end() && bRet; ++it1, ++it2)
        {
            if (it1->second > it2->first)
            {
                SDOR_LOG_WARN << "The end index is invalid.";
                bRet = false;
            }
        }
    }

    const std::vector<Point3_T> *pvInputPoint(nullptr);
    const std::vector<std::pair<int32_t, int32_t>> *pvEndIdx(nullptr);

    std::vector<std::pair<int32_t, int32_t>> vSampleEndIdx;
    std::vector<Point3_T> vSamplePoint;

    bool bSample;
    bSample = (numPointsToFit_ > 1000 ? true : false);

    if (bSample)
    {
        std::vector<int> vSampleIdx;
        DouglasPeucker<Point3_T> DPobj;
        DPobj.getDPresult(inputPoints, maxError * 0.5f, vSampleIdx);

        std::vector<int> vIndex;
        subSampleByDistance(inputPoints, 0.5f, vIndex);
        vSampleIdx.insert(vSampleIdx.end(), vIndex.begin(), vIndex.end());

        std::vector<std::pair<int, bool>> vIdxAndType, vTmpIdxType; //second of every is it's type, whether it is end index
        vIdxAndType.reserve(vSampleIdx.size() + vecEndIndex.size() * 2);

        for (auto it = vecEndIndex.begin(); it < vecEndIndex.end(); ++it)
        {
            vIdxAndType.emplace_back(it->first, true);
            vIdxAndType.emplace_back(it->second, true);
        }

        for (auto it = vSampleIdx.begin(); it < vSampleIdx.end(); ++it)
        {
            vIdxAndType.emplace_back(*it, false);
        }

//         SDOR_LOG_DEBUG << "vecEndIndex.size() = " << vecEndIndex.size();
//         for (auto e : vecEndIndex)
//         {
//             std::cout << "[" << e.first << ", " << e.second << "]" << ", ";
//         }
//         std::cout << std::endl;

        std::stable_sort(vIdxAndType.begin(), vIdxAndType.end(),
                  [](const std::pair<int, bool> &pair1, const std::pair<int, bool> &pair2)
                  {
                      if (pair1.first < pair2.first)
                      {
                          return true;
                      }
                      else if (pair1.first == pair2.first)
                      {
                          if (pair1.second)
                          {
                              return true;
                          }
                          else
                          {
                              return false;
                          }
                      }
                      else
                      {
                          return false;
                      }
                  });

//         SDOR_LOG_DEBUG << "vIdxAndType.size() = " << vIdxAndType.size();
//         for (auto e : vIdxAndType)
//         {
//             std::cout << "[" << e.first << ", " << e.second << "]" << ", ";
//         }
//         std::cout << std::endl;

        int index(0);
        for (auto it = vIdxAndType.begin(); it < vIdxAndType.end(); ++it)
        {
            if (it->second)
            {
                index = it->first;
            }
            else if (index == it->first)
            {
                continue;
            }
            else
            {
            }

            vTmpIdxType.emplace_back(*it);
        }

        vIdxAndType.swap(vTmpIdxType);

//         SDOR_LOG_DEBUG << "vIdxAndType.size() = " << vIdxAndType.size();
//         for (auto e : vIdxAndType)
//         {
//             std::cout << "[" << e.first << ", " << e.second << "]" << ", ";
//         }
//         std::cout << std::endl;

        vSampleEndIdx.reserve(vecEndIndex.size());
        vSamplePoint.reserve(vSampleIdx.size());

        std::pair<int32_t, int32_t> tmpPair;
        bool bFirst = true;

        int i = 0;
        for (auto it = vIdxAndType.begin(); it < vIdxAndType.end(); ++it, ++i)
        {
            vSamplePoint.emplace_back(inputPoints[it->first]);

            if (it->second)
            {
                if (bFirst)
                {
                    tmpPair.first = i;
                }
                else
                {
                    tmpPair.second = i;
                    vSampleEndIdx.emplace_back(tmpPair);
                }

                bFirst = !bFirst;
            }
        }

        pvInputPoint = &vSamplePoint;
        pvEndIdx = &vSampleEndIdx;

        numPointsToFit_ = static_cast<int32_t>(vSamplePoint.size());
    }
    else
    {
        if (sampleAndEnd(inputPoints, vecEndIndex, vSamplePoint, vSampleEndIdx, 0.005f))
        {
            pvInputPoint = &vSamplePoint;
            pvEndIdx = &vSampleEndIdx;
            numPointsToFit_ = static_cast<int32_t>(vSamplePoint.size());
        }
        else
        {
            pvInputPoint = &inputPoints;
            pvEndIdx = &vecEndIndex;
        }
    }

    //start fitting
    std::vector<float32_t> U, denseU;
    std::vector<cv::Point3f> fitPoints;
    if (bRet && fitAndReconstructAccuracy(*pvInputPoint, U, denseU, maxError, curveParam, fitPoints))
    {
        //Generate the endPoints with parameter u
        curveParam.endPoint.clear();
        curveParam.endPoint.reserve(pvEndIdx->size() << 1);

        for (auto &endPair : *pvEndIdx)
        {
            if (endPair.first >= static_cast<int32_t>(U.size()) ||
                endPair.first < 0 ||
                endPair.second >= static_cast<int32_t>(U.size()) ||
                endPair.second < endPair.first)
            {
                SDOR_LOG_WARN << "not valid endPair data.";
                return false;
            }

            curveParam.endPoint.emplace_back(U[endPair.first]);
            curveParam.endPoint.emplace_back(U[endPair.second]);
        }

        bRet = calcAccuracyLineLength(*pvInputPoint, *pvEndIdx, denseU, curveParam);
    }
    else
    {
        SDOR_LOG_WARN << "function fitAndReconstructAccuracy return false";
        bRet = false;
    }

    return bRet;
}


private:

template <class Point3_T>
bool searchMinDistance(const std::vector<Point3_T> &vInputPoint,
                       const std::vector<cv::Point3f> &vReSamplePoint,
                       std::vector<float> &vMinDistance) const
{
    bool bRet = true;
    if (vInputPoint.empty() || vReSamplePoint.empty())
    {
        SDOR_LOG_WARN << "The input data are invalid";
        bRet = false;
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if(NULL == cloud)
        {
            SDOR_LOG_WARN << "PointCloud Prt alloc error";
            return false;
        }

        // Generate pointcloud data
        cloud->width = vReSamplePoint.size();
        cloud->height = 1;
        cloud->points.resize (cloud->width * cloud->height);

        auto itCloud = cloud->points.begin();
        auto itReSample = vReSamplePoint.begin();

        for ( ; itCloud < cloud->points.end(); ++itCloud, ++itReSample)
        {
            itCloud->x = itReSample->x;
            itCloud->y = itReSample->y; itCloud->z = itReSample->z;
        }

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);

         // K nearest neighbor search
        const int32_t K = 1;

        std::vector<int32_t> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        std::vector<float>::iterator itOfDistance;
        pcl::PointXYZ searchPoint;

        for (auto it = vInputPoint.begin(); it < vInputPoint.end(); ++it)
        {
            searchPoint.x = it->x;
            searchPoint.y = it->y; searchPoint.z = it->z;

            if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
            {
                vMinDistance.emplace_back(pointNKNSquaredDistance.front());
            }
            else
            {
                SDOR_LOG_WARN << "The function kdtree.nearestKSearch return false";
                bRet = false;
                break;
            }
        }
    }

    return bRet;
}


    bool generateKnot(int32_t numCtrPoitns, const std::vector<float32_t> &U, std::vector<float32_t> &knot) const
    {
        if (numCtrPoitns > 0 && !U.empty() && numCtrPoitns < static_cast<int32_t>(U.size()))
        {
            int32_t degree = configPara_.order - 1;
            int32_t n = numCtrPoitns-1;
            int32_t numKnot = n + degree + 2;

            knot.clear();
            knot.reserve(numKnot);
            knot.resize(degree + 1, 0);    // The first degree+1 elements are zero.

            if(0 == n-degree+1)
            {
                SDOR_LOG_WARN << "n-p+1 value is 0";
                return false;
            }
            float32_t d = static_cast<float32_t>(U.size()) / static_cast<float32_t>(n-degree+1);
            int32_t index;
            float32_t i, alpha, tmp;
            for (int32_t j = 1; j <= n-degree; ++j)
            {
                i = std::floor(j * d);
                alpha = static_cast<float>(j) * d - i;
                index = static_cast<int32_t>(i);
                tmp = (1-alpha) * U[index-1] + alpha * U[index];
                knot.emplace_back(tmp);
            }

            knot.resize(numKnot, 1);  // The last p+1 elements are one.

            return true;
        }
        else
        {
            SDOR_LOG_WARN << "The arguments are invalid in function generateKnot.";
            return false;
        }
    }


    bool solveMatrixN(int32_t numCtrPoints,
                      const std::vector<float32_t> &knot,
                      const std::vector<float32_t> &U,
                      std::vector<NURBS::ColOfMatrix> &matrixN) const
    {
        int32_t numPoints = static_cast<int32_t>(U.size());
        if(numCtrPoints > numPoints || knot.empty() || U.empty())
        {
            SDOR_LOG_WARN << "The arguments is error in function solveMatrixN.";
            return false;
        }

        int32_t degree = configPara_.order - 1;
        matrixN.clear();
        matrixN.resize(numCtrPoints);

        int32_t index, s = degree;
        std::vector<float32_t> N;          //It is a temporary variable
        for (int32_t row = 0; row < numPoints; ++row)
        {
            if (!findSpan(numCtrPoints - 1, s, U[row], knot, s))
            {
                return false;
            }

            if (!basisFun(s, U[row], degree, knot, N))
            {
                return false;
            }

            index = s - degree;
            for (int32_t i = 0; i <= degree; ++i)
            {
                if (matrixN[index + i].start_ == -1)
                {
                    matrixN[index + i].start_ = row;
                }
                matrixN[index + i].vec_.emplace_back(N[i]);
            }
        }

        std::for_each(matrixN.begin(), matrixN.end(), [](NURBS::ColOfMatrix &elem)
                    {
                        elem.end_ = elem.start_ + static_cast<int32_t>(elem.vec_.size());
                    });

        return true;
    }


    template <class Point3_T>
    bool solveCtrPoints(const std::vector<Point3_T> &inputPoints,
                        const std::vector<NURBS::ColOfMatrix> &matrixN,
                        const std::vector<Point3_T> &R,
                        std::vector<roadDBCore::Point3f_t> &ctrPoints) const
    {
        //matrixN represent a matrix
        if (inputPoints.empty() || matrixN.empty() || R.empty())
        {
            SDOR_LOG_WARN << "The arguments are invalid in function solveCtrPoints.";
            return false;
        }

        int32_t cols = static_cast<int32_t>(matrixN.size());  //Every element of matrixN is a column of a matrix
        if (cols-2 != static_cast<int32_t>(R.size()))
        {
            SDOR_LOG_WARN << "The vector R does not match the matrix NRIO";
            return false;
        }

        //calculate (matrixN^T * matrixN)
        cv::Mat squareMatrix = cv::Mat::zeros(cols, cols, CV_32F); // square matrix, squareMatrix = matrixN' * matrixN
        auto itr = matrixN.begin();
        for (int32_t r = 0; r < cols; ++r)  // square matrix
        {
            auto p = squareMatrix.ptr<float32_t>(r);
            float32_t tmp(0);
            for (auto itCol = itr->vec_.begin(); itCol < itr->vec_.end(); ++itCol)
            {
                tmp += (*itCol) * (*itCol);
            }
            p[r] = tmp * 0.5f; // because it calculate a half of the matrix only, this function will sum this matrix and it's transposition
            auto itc = itr + 1;

            for (int32_t c = r+1; c < cols; ++c)  // square matrix
            {
                auto length = itr->end_ - itc->start_;
                if (length <= 0)
                {
                    break;
                }
                else if (length > static_cast<int32_t>(itc->vec_.size()))
                {
                    length = itc->vec_.size();
                }
                else
                {
                }

                tmp = 0;
                auto index = itc->start_ - itr->start_;
                for (int32_t i = 0; i < length; ++i)
                {
                    tmp += itr->vec_[index] * itc->vec_[i];
                    ++index;
                }

                p[c] = tmp;
                ++itc;
            }
            ++itr;
        }

        squareMatrix = squareMatrix + squareMatrix.t();

        cv::Rect center(1, 1, cols-2, cols-2);
        cv::Mat sMatrix = squareMatrix(center);

        //calculate (matrixN^T * matrixN)
        sMatrix = sMatrix.inv(cv::DECOMP_LU);

        //calculate control points. control points = squareMatrix*R
        ctrPoints.clear();
        ctrPoints.reserve(cols);
        cols = sMatrix.cols;

        roadDBCore::Point3f_t tmpPoint, zeroPoint(0, 0, 0);

        auto &point1 = inputPoints.front();
        tmpPoint.relLon = point1.x;
        tmpPoint.relAlt = point1.y;
        tmpPoint.relLat = point1.z;

        ctrPoints.emplace_back(tmpPoint);  //the first control point is the first point of the curve
        for (int32_t i = 0; i < cols; ++i)
        {
            tmpPoint = zeroPoint;
            float32_t *q = sMatrix.ptr<float32_t>(i);
            auto itR = R.begin();
            for (int32_t j = 0; j < cols; ++j)
            {
                const auto &element = q[j];
                tmpPoint.relLon += itR->x * element;
                tmpPoint.relAlt += itR->y * element;
                tmpPoint.relLat += itR->z * element;
                ++itR;
            }
            ctrPoints.emplace_back(tmpPoint);
        }

        auto &point2 = inputPoints.back();
        tmpPoint.relLon = point2.x;
        tmpPoint.relAlt = point2.y;
        tmpPoint.relLat = point2.z;
        ctrPoints.emplace_back(tmpPoint); //the last control point is the last point of the curve

        return true;
    }


    template <class Point3_T>
    bool generateR(const std::vector<NURBS::ColOfMatrix> &matrixN,
                   const std::vector<Point3_T> &inputPoints,
                   std::vector<Point3_T> &R) const
    {
        if (inputPoints.empty() || matrixN.empty())
        {
            SDOR_LOG_WARN << "No input Points in generateR";
            return false;
        }

        // It equal to the number of control Points. Every element of matrixN is a column of a matrix
        int32_t cols = static_cast<int32_t>(matrixN.size());
        if (cols < 2)
        {
            SDOR_LOG_WARN << "data error.";
            return false;
        }
        int32_t rows = static_cast<int32_t>(inputPoints.size());

        std::vector<Point3_T> Rmatrix(rows);
        ColOfMatrix firstCol = matrixN.front(), lastCol = matrixN.back();
        float32_t element1, element2;
        for (int32_t k = 1; k < rows-1; ++k)
        {
            if (k < firstCol.start_ || k >= firstCol.end_)
            {
                element1 = 0;
            }
            else
            {
                element1 = firstCol.vec_[k - firstCol.start_];
            }

            if (k < lastCol.start_ || k >= lastCol.end_)
            {
                element2 = 0;
            }
            else
            {
                element2 = lastCol.vec_[k - lastCol.start_];
            }

            //Rmatrix[k] = inputPoints[k] - element1 * inputPoints.front() - element2 * inputPoints.back()
            Rmatrix[k] = subtract(inputPoints[k], add(numericalMultiply(element1, inputPoints.front()), numericalMultiply(element2, inputPoints.back())));
        }

        R.clear();
        R.reserve(cols-2);

        auto itN = matrixN.begin() + 1;
        int32_t startIdxN, endIdxN, startIdxR;
        Point3_T tmpPoint;
        for (int32_t i = 1; i < cols-1; ++i)
        {
            if (itN->start_ < 1)
            {
                startIdxR = 1;
                startIdxN = 1 - itN->start_;
            }
            else
            {
                startIdxR = itN->start_;
                startIdxN = 0;
            }

            if (itN->end_ > rows-1)
            {
                endIdxN = rows-1 - itN->start_;
            }
            else
            {
                endIdxN = itN->end_ - itN->start_;
            }

            auto it = Rmatrix.begin() + startIdxR;

            reset(tmpPoint);
            for (int32_t j = startIdxN; j < endIdxN; ++j)
            {
                tmpPoint += numericalMultiply(itN->vec_[j], *it);
                ++it;
            }
            R.emplace_back(tmpPoint);
            ++itN;
        }

        return true;
    }


    template <class Point3_T>
    bool generateCurveAndRegion(const roadDBCore::NURBS_t &NURBSParam,
                                const float32_t step,
                                std::vector<Point3_T> &outputPoints,
                                std::vector<size_t> &vRange,
                                std::vector<float32_t> &U) const
    {
        if (NURBSParam.vecCtrlPoint.empty() || step < FLT_EPSILON || NURBSParam.endPoint.empty())
        {
            SDOR_LOG_WARN << "The arguments are invalid in function generateCurve.";
            return false;
        }

        int32_t outputPointNum = static_cast<int32_t>(std::ceil(NURBSParam.lineLength / step));
        outputPointNum = std::max(outputPointNum, 2);

        if (NURBSParam.vecCtrlPoint.empty())
        {
            SDOR_LOG_WARN << "Dash line need end points.";
            return false;
        }
        else if ((NURBSParam.endPoint.size() & 0x1) != 0)
        {
            SDOR_LOG_WARN << "The number of end points must be even.";
            return false;
        }
        else
        {
            float32_t scale = (NURBSParam.endPoint.back() - NURBSParam.endPoint.front()) / (outputPointNum - 1);

            U.clear();
            U.reserve(outputPointNum);

            //fill the vector U with fractional part that is between every pair of endpoints
            //the endPoint is float type.

            float32_t tmp = NURBSParam.endPoint.front();
            while (tmp < NURBSParam.endPoint.back())
            {
                U.emplace_back(tmp);
                tmp += scale;
            }
            U.emplace_back(NURBSParam.endPoint.back());

            if (!outSample(NURBSParam, U, outputPoints, vRange))
            {
                SDOR_LOG_WARN << "Sample error";
                return false;
            }
        }
        return true;
    }


    template <class Point3_T>
    bool fitAndReconstruct(const std::vector<Point3_T> &inputPoints,
                           roadDBCore::NURBS_t &curveParam,
                           std::vector<float32_t> &U,
                           std::vector<cv::Point3f> &fitPoints) const
    {
    #if 0
        SDOR_LOG_DEBUG << "inputPoints.size() = " << inputPoints.size();
        for(std::vector<cv::Point3f>::const_iterator it = inputPoints.begin(); it < inputPoints.end(); ++it)
        {
            SDOR_LOG_DEBUG << it->x << " " << it->y << " " << it->z << " ";
        }
    #endif

        if (!configPara_.isValid())
        {
            SDOR_LOG_WARN << "Configuration parameters are invalid.";
            return false;
        }

        int32_t numPoints = static_cast<int32_t>(inputPoints.size());
        if (numPoints < 2)
        {
            return false;
        }

        //Calculate the distance of points
        int32_t numDistance = numPoints - 1;
        std::vector<float> distance;
        distance.reserve(numDistance);

        float32_t tmpDistance, tmpDiff;
        float32_t squareDist(0);

        for (auto first = inputPoints.begin(), second = first + 1; second < inputPoints.end(); ++first, ++second)
        {
            tmpDiff = first->x - second->x;
            tmpDistance = tmpDiff * tmpDiff;

            tmpDiff = first->z - second->z;
            tmpDistance += tmpDiff * tmpDiff;

    //        distance.emplace_back(tmpDistance);
            distance.emplace_back(sqrt(tmpDistance));
            squareDist += tmpDistance;
        }

        float32_t sumDistance = std::accumulate(distance.begin(), distance.end(), 0.0f);

        //Generate the vector U, U is u bar
        U.clear();
        U.reserve(numPoints);
        if (sumDistance < 0.1f)
        {
            SDOR_LOG_WARN << "sumDistance result is 0";
            return false;
        }
        float32_t divisor = 1 / sumDistance;
        float32_t tmp(0.0f);
        for(int32_t i = 0; i < numDistance; ++i)
        {
            U.emplace_back(tmp);
            tmp += distance[i] * divisor;

            if (tmp - 1.f > FLT_MIN)
            {
                tmp = 1.f;
            }
        }
        U.emplace_back(1.0f);

        int32_t sign = cycleFit(inputPoints, U, squareDist, curveParam, fitPoints);

        return (sign >= 0);
    }


    template <class Point3_T>
    bool fitAndReconstructAccuracy(const std::vector<Point3_T> &inputPoints,
                                   std::vector<float32_t> &U,
                                   std::vector<float> &denseU,
                                   const float &maxError,
                                   roadDBCore::NURBS_t &curveParam,
                                   std::vector<cv::Point3f> &fitPoints) const
    {
        if (!configPara_.isValid())
        {
            SDOR_LOG_WARN << "Configuration parameters are invalid.";
            return false;
        }

        if (numPointsToFit_ < static_cast<int32_t>(minNumPoint_))
        {
            SDOR_LOG_DEBUG << "Number of points is too less.";
            return false;
        }

        //Calculate the distance of points
        int32_t numDistance = numPointsToFit_ - 1;
        std::vector<float> distance;
        distance.reserve(numDistance);

        float32_t tmpDistance, tmpDiff;
        for (auto first = inputPoints.begin(), second = first + 1; second < inputPoints.end(); ++first, ++second)
        {
            tmpDiff = first->x - second->x;
            tmpDistance = tmpDiff * tmpDiff;

            tmpDiff = first->z - second->z;
            tmpDistance += tmpDiff * tmpDiff;

    //        distance.emplace_back(tmpDistance);
            distance.emplace_back(std::sqrt(tmpDistance));
        }

        float32_t sumDistance = std::accumulate(distance.begin(), distance.end(), 0.f);

        //Generate the vector U, U is u bar
        U.clear();
        U.reserve(numPointsToFit_);
        denseU.clear();
        denseU.reserve(numPointsToFit_);
        if (sumDistance < 0.1f)
        {
            SDOR_LOG_WARN << "The line is too short.";
            return false;
        }

        float density = 3.f;
        float32_t divisor = 1.f / sumDistance, miniDivisor = 1.f / density;
        float32_t elem(0), increment(0), miniIncrement(0), tmp(0);

        for (int32_t i = 0; i < numDistance; ++i)
        {
            increment = distance[i] * divisor;
            miniIncrement = increment * miniDivisor;
            tmp = elem;
            for (int32_t j = 0; j < density; ++j)
            {
                denseU.emplace_back(tmp);
                tmp += miniIncrement;
            }
            U.emplace_back(elem);
            elem += increment;

            if (elem - 1.f > FLT_MIN)
            {
                elem = 1.f;
            }
        }

        U.emplace_back(1.f);
        denseU.emplace_back(1.f);

        int32_t sign = cycleFitAccuracy(inputPoints, U, maxError, curveParam, fitPoints);

        return (sign >= 0);
    }

    //return value:  0 for good result, 1 for bad result, -1 for error.
    template <class Point3_T>
    int32_t cycleFit(const std::vector<Point3_T> &inputPoints,
                     const std::vector<float32_t> &U,
                     float squareDist,
                     roadDBCore::NURBS_t &curveParam,
                     std::vector<cv::Point3f> &fitPoints) const
    {
        if (inputPoints.empty() || U.empty())
        {
            SDOR_LOG_WARN << "No input data in function cycleFit";
            return -1;
        }

        //Calculate the original number of control points, need to be improved
        int32_t numPoints = static_cast<int32_t>(inputPoints.size());
        int32_t numCtrPoints = static_cast<int32_t>(std::ceil(numPoints * configPara_.minCtrPointsRatio));

        if (numCtrPoints < configPara_.minNumCtrPoints)
        {
            numCtrPoints = configPara_.minNumCtrPoints;
        }

        //start fitting
        int32_t maxNumCtrPoints = static_cast<int32_t>(numPoints * configPara_.maxCtrPointsRatio);
        float coef, delta;
        std::vector<Point3_T> R;
        std::vector<NURBS::ColOfMatrix> matrixN;

        //Fit the curve and calculate the derivation until the derivation is less then the threshold or the control points equal to the max value
        while(1)
        {
//             SDOR_LOG_INFO << "numCtrPoints = " << numCtrPoints;
            if (!generateKnot(numCtrPoints, U, curveParam.vecKnot))
            {
                SDOR_LOG_WARN << "The function generateKnot return false.";
                return -1;
            }

            if (!solveMatrixN(numCtrPoints, curveParam.vecKnot, U, matrixN))
            {
                SDOR_LOG_WARN << "The function solveMatrixN return false.";
                return -1;
            }

            if (!generateR(matrixN, inputPoints, R))
            {
                SDOR_LOG_WARN << "The function generateR return false.";
                return -1;
            }

            if (!solveCtrPoints(inputPoints, matrixN, R, curveParam.vecCtrlPoint))
            {
                SDOR_LOG_WARN << "The function solveCtrPoints return false.";
                return -1;
            }

            if (!inerSample(curveParam, matrixN, fitPoints))
            {
                SDOR_LOG_WARN << "The function inerSample return false.";
                return -1;
            }

            if (numCtrPoints >= maxNumCtrPoints)
            {
//                 SDOR_LOG_INFO << "The number of control points is more than the max number.";
                return 1;
            }

            if (!checkResult(inputPoints, curveParam, fitPoints, squareDist, coef))
            {
                SDOR_LOG_INFO << "Completed the fitting.";
                return 0;
            }

            delta = (maxNumCtrPoints - numCtrPoints * 0.5f) / (1 + std::exp(configPara_.translation - coef + 1));
            numCtrPoints += static_cast<int32_t>(ceil(delta));

            if (numCtrPoints > maxNumCtrPoints)
            {
                numCtrPoints = maxNumCtrPoints;
            }
        }
    }

    template <class Point3_T>
    int32_t cycleFitAccuracy(const std::vector<Point3_T> &inputPoints,
                             const std::vector<float32_t> &U,
                             const float &maxError,
                             roadDBCore::NURBS_t &curveParam,
                             std::vector<cv::Point3f> &fitPoints) const
    {
        int32_t nRet = 0;
        if (inputPoints.empty() || U.empty())
        {
            SDOR_LOG_WARN << "No input data in function cycleFit";
            nRet = -1;
        }

        //Calculate the original number of control points, need to be improved
        int32_t numPoints = static_cast<int32_t>(inputPoints.size());
        int32_t numCtrPoints = static_cast<int32_t>(std::ceil(numPoints * configPara_.minCtrPointsRatio));

        if (numCtrPoints < configPara_.minNumCtrPoints)
        {
            numCtrPoints = configPara_.minNumCtrPoints;
        }

        //start fitting
        int32_t maxNumCtrPoints = static_cast<int32_t>(numPoints * configPara_.maxCtrPointsRatio);
        if (maxNumCtrPoints < 2 * numCtrPoints)
        {
            maxNumCtrPoints = 2 * numCtrPoints;

            maxNumCtrPoints = maxNumCtrPoints < numPoints ? maxNumCtrPoints : numPoints - 1;
        }

        std::vector<Point3_T> R;
        std::vector<NURBS::ColOfMatrix> matrixN;

        const int32_t maxOutlierNum = numPointsToFit_ * 0.f;
        //Fit the curve and calculate the derivation until the derivation is less then the threshold or the control points equal to the max value
        while (nRet >= 0)
        {
//             SDOR_LOG_INFO << "numCtrPoints = " << numCtrPoints;
            if (!generateKnot(numCtrPoints, U, curveParam.vecKnot))
            {
                SDOR_LOG_WARN << "The function generateKnot return false.";
                nRet = -1;
                break;
            }

            if (!solveMatrixN(numCtrPoints, curveParam.vecKnot, U, matrixN))
            {
                SDOR_LOG_WARN << "The function solveMatrixN return false.";
                nRet = -1;
                break;
            }

            if (!generateR(matrixN, inputPoints, R))
            {
                SDOR_LOG_WARN << "The function generateR return false.";
                nRet = -1;
                break;
            }

            if (!solveCtrPoints(inputPoints, matrixN, R, curveParam.vecCtrlPoint))
            {
                SDOR_LOG_WARN << "The function solveCtrPoints return false.";
                nRet = -1;
                break;
            }

            if (!inerSample(curveParam, matrixN, fitPoints))
            {
                SDOR_LOG_WARN << "The function inerSample return false.";
                nRet = -1;
                break;
            }

            if (numCtrPoints >= maxNumCtrPoints)
            {
//                SDOR_LOG_INFO << "The number of control points is more than the max number.";
                nRet = 1;
                break;
            }

            std::vector<float> vMinDistance;
            if (!searchMinDistance(inputPoints, fitPoints, vMinDistance))
            {
                SDOR_LOG_WARN << "The function searchMinDistance return false.";
                nRet = -1;
                break;
            }

            int32_t counter(0);
            float sumDistance(0);

            for (auto &elem : vMinDistance)
            {
                if (elem > maxError)
                {
                    ++counter;
                }
                sumDistance += elem;
            }

            if (counter <= maxOutlierNum)
            {
                nRet = 0;
                break;
            }

            float delta = (maxNumCtrPoints - numCtrPoints + 5.f) / (1.f + std::exp(configPara_.translation - sumDistance + 1.f));
            numCtrPoints += static_cast<int32_t>(ceil(delta));

            if (numCtrPoints > maxNumCtrPoints)
            {
                numCtrPoints = maxNumCtrPoints;
            }
        }

        return nRet;
    }


    bool inerSample(const roadDBCore::NURBS_t &curveParam,
                    const std::vector<NURBS::ColOfMatrix> &matrixN,
                    std::vector<cv::Point3f> &outPoints) const
    {
        if (curveParam.vecCtrlPoint.empty() || matrixN.empty() || numPointsToFit_ < static_cast<int32_t>(minNumPoint_))
        {
            SDOR_LOG_WARN << "The arguments are invalid in function inerSample.";
            return false;
        }

        auto numCtrPoints = curveParam.vecCtrlPoint.size();

        outPoints.clear();
        outPoints.reserve(numPointsToFit_);

        std::vector<NURBS::ColOfMatrix>::const_iterator itMatrix;
        auto itPoint = curveParam.vecCtrlPoint.begin(), it = itPoint;

        cv::Point3f tmpPoint, zeroPoint(0,0,0);
        int32_t k;
        for (int32_t i = 0; i < numPointsToFit_; ++i)
        {
            tmpPoint = zeroPoint;
            itMatrix = matrixN.begin();
            for (size_t j = 0; j < numCtrPoints; ++j)
            {
                if(itMatrix->start_ > i)
                {
                    break;
                }
                else if (itMatrix->end_ <= i)
                {
                    ++itMatrix;
                    continue;
                }
                else
                {
                    k = i - itMatrix->start_;
                    it = itPoint + j;
                    tmpPoint.x += itMatrix->vec_[k] * it->relLon;
                    tmpPoint.z += itMatrix->vec_[k] * it->relLat; tmpPoint.y += itMatrix->vec_[k] * it->relAlt;
                    ++itMatrix;
                }
            }

            if (!std::isfinite(tmpPoint.x) || !std::isfinite(tmpPoint.y) || !std::isfinite(tmpPoint.z))
            {
                SDOR_LOG_DEBUG << "the point is not valid " << " tmpPoint.x " << tmpPoint.x << " tmpPoint.y " << tmpPoint.y << " tmpPoint.z " << tmpPoint.z;
                continue;
            }

            outPoints.emplace_back(tmpPoint);
        }

        return true;
    }


    template <class Point3_T>
    bool outSample(const roadDBCore::NURBS_t &curveParam,
                   const std::vector<float32_t> &U,
                   std::vector<Point3_T> &outPoints,
                   std::vector<size_t> &vRange) const
    {
        if (curveParam.vecCtrlPoint.empty() || U.empty())
        {
            SDOR_LOG_WARN << "The arguments are invalid in function outSample.";
            return false;
        }

        int32_t degree = configPara_.order - 1;

        outPoints.clear();
        vRange.clear();
        outPoints.reserve(U.size());

        int32_t numPoints = static_cast<int32_t>(U.size());
        int32_t nCP = static_cast<int32_t>(curveParam.vecCtrlPoint.size());

        std::vector<float32_t> N;
        int32_t tmpInd;
        int32_t s = degree;

        Point3_T tmpPoint;
        for (int32_t col = 0; col < numPoints; ++col)
        {
            if (!findSpan(nCP-1, s, U[col], curveParam.vecKnot, s))
            {
                return false;
            }

            if (!basisFun(s, U[col], degree, curveParam.vecKnot, N))
            {
                return false;
            }

            tmpInd = s - degree;
            reset(tmpPoint);
            for (int32_t i = 0; i <= degree; i++)
            {
                auto &elemN = N[i];
                auto &ctrPoint = curveParam.vecCtrlPoint[tmpInd+i];
                tmpPoint.x += elemN * ctrPoint.relLon;
                tmpPoint.y += elemN * ctrPoint.relAlt;
                tmpPoint.z += elemN * ctrPoint.relLat;
            }
            if (!std::isfinite(tmpPoint.x) || !std::isfinite(tmpPoint.y) || !std::isfinite(tmpPoint.z))
            {
                SDOR_LOG_DEBUG << "the point is not valid " << " tmpPoint.x " << tmpPoint.x << " tmpPoint.y " << tmpPoint.y << " tmpPoint.z " << tmpPoint.z;
                continue;
            }

            vRange.emplace_back(s);
            outPoints.emplace_back(tmpPoint);
        }

        return !outPoints.empty();
    }

    //This function is called frequently
    bool basisFun(const int32_t s, const float32_t u, const int32_t p, const std::vector<float32_t> &U, std::vector<float32_t> &N) const
    {
        if (!U.empty())
        {
            std::vector<float32_t> left(p+1, 0.0f);
            std::vector<float32_t> right(p+1, 0.0f);

            N.clear();
            N.resize(p+1, 0.0f);
            N[0] = 1.0f;

            float32_t saved, tmp;
            for (int32_t j = 1; j <= p; ++j)
            {
                left[j] = u - U[s+1-j];
                right[j] = U[s+j] - u;
                saved = 0.0f;

                for (int32_t r = 0; r < j; ++r)
                {
                    tmp = N[r] / (right[r+1] + left[j-r]);
                    N[r] = saved + right[r+1] * tmp;
                    saved = left[j-r] * tmp;
                }

                N[j] = saved;
            }

            return true;
        }
        else
        {
            SDOR_LOG_DEBUG << "Input data are empty";
            return false;
        }
    }

    //This function is called frequently
    bool findSpan(const int32_t n, const int32_t p, float32_t u, const std::vector<float32_t> &knot, int32_t &s) const
    {
        if (n < static_cast<int32_t>(knot.size()) && p <= n)
        {
            if (u < knot[p])
            {
                s = p;
            }
            else if (u > knot[n])
            {
                s = n;
            }
            else
            {
                int32_t low = p;
                int32_t high = n+1;
                int32_t mid = low + (high-low)/2; // mid = floor((high+low)/2)

                while (low + 1 < high) // +1 for low = high -1 case
                {
                    if (u - knot[mid] < -FLT_MIN)
                    {
                        high = mid;
                    }
                    else if (u - knot[mid+1] > -FLT_MIN)
                    {
                        low = mid;
                    }
                    else
                    {
                        break;
                    }

                    mid = low + (high-low)/2;  // mid = floor((high+low)/2)
                }
                s = mid;
            }

            return true;
        }
        else
        {
            SDOR_LOG_WARN << "The arguments are invalid in function findSpan.";
            return false;
        }
    }

    template <class Point3_T>
    bool checkResult(const std::vector<Point3_T>    &inputPoints,
                     const roadDBCore::NURBS_t      &curveParam,
                     const std::vector<cv::Point3f> &fitPoints,
                     const float                    &refSquareDist,
                     float                          &coef) const
    {
        if (fitPoints.size() != inputPoints.size())
        {
            SDOR_LOG_WARN << "The number of fitting points does not match the number of input points in checkResult.";
            return false;
        }
        else if (refSquareDist - configPara_.errorThresh < FLT_EPSILON)
        {
            SDOR_LOG_DEBUG << "Maximum error is too small.";
            return false;
        }
        else
        {
            float error(0), tmp, squareDis;
            auto inputIt = inputPoints.begin();
            auto fitIt = fitPoints.begin();
            while (inputIt < inputPoints.end())
            {
                tmp = inputIt->x - fitIt->x;
                squareDis = tmp * tmp;

                tmp = inputIt->z - fitIt->z;
                squareDis += tmp * tmp;

                error += std::sqrt(squareDis);

                ++inputIt;
                ++fitIt;
            }

            coef = error / refSquareDist - configPara_.errorThresh;

            return true;
        }
    }


    bool lineLength(const std::vector<cv::Point2f> &fitPoints,
                    const std::vector<std::pair<int32_t, int32_t>> &vecEndIndex,
                    float32_t &lineLength,
                    float32_t &paintTotalLength) const
    {
        if (!fitPoints.empty())
        {
            lineLength = 0.0f;
            paintTotalLength = 0.0f;
            float32_t tmp(0);
            int32_t i(0);

            //first point the first point of the segment, and second point the last, not the next of the last
            for (auto &endPair : vecEndIndex)
            {
                while (i < endPair.first)
                {
                    tmp = cv::norm(fitPoints[i+1] - fitPoints[i]);
                    lineLength += tmp;
                    ++i;
                }

                while (i < endPair.second)
                {
                    tmp = cv::norm(fitPoints[i+1] - fitPoints[i]);
                    paintTotalLength += tmp;
                    lineLength += tmp;
                    ++i;
                }
            }

            return true;
        }
        else
        {
            SDOR_LOG_WARN << "The arguments are invalid in function lineLength.";
            return false;
        }
    }

    template <typename PointXYZ>
    bool calcAccuracyLineLength(const std::vector<PointXYZ> &inputPoints,
                                const std::vector<std::pair<int32_t, int32_t>> &vecEndIndex,
                                const std::vector<float> &denseU,
                                roadDBCore::NURBS_t &curveParam) const
    {
        bool bRet = false;
        std::vector<PointXYZ> fitPoints;
        if (!denseU.empty() && !inputPoints.empty())
        {
            bRet = outSample(curveParam, denseU, fitPoints);
        }

        if (bRet)
        {
            std::vector<std::pair<int32_t, int32_t>> vecDenseEndIndex;
            pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);
            if (nullptr == cloud)
            {
                SDOR_LOG_WARN << "PointCloud Ptr alloc error";
                return false;
            }

            // Generate pointcloud data
            cloud->width = fitPoints.size();
            cloud->height = 1;
            cloud->points.resize(cloud->width * cloud->height);

            auto itCloud = cloud->points.begin();
            auto itFit = fitPoints.begin();
            for ( ; itCloud < cloud->points.end(); ++itCloud, ++itFit)
            {
                itCloud->x = itFit->x;
                itCloud->y = itFit->z;
            }

            pcl::KdTreeFLANN<pcl::PointXY> kdtree;
            kdtree.setInputCloud(cloud);

            // K nearest neighbor search
            const int32_t K = 10;

            std::vector<int32_t> pointIdxNKNSearchFitst(K), pointIdxNKNSearchSecond(K);
            std::vector<float> pointNKNSquaredDistanceFirst(K), pointNKNSquaredDistanceSecond(K);

            std::vector<float>::iterator itFirst, itSecond;
            pcl::PointXY searchFirstPoint, searchSecondPoint;

            for (auto itEndIdx = vecEndIndex.begin(); itEndIdx < vecEndIndex.end() && bRet; ++itEndIdx)
            {
                searchFirstPoint.x = inputPoints[itEndIdx->first].x;
                searchFirstPoint.y = inputPoints[itEndIdx->first].z;

                searchSecondPoint.x = inputPoints[itEndIdx->second].x;
                searchSecondPoint.y = inputPoints[itEndIdx->second].z;

                int minIdx(0), idx1(-1), idx2(-1);

                if (kdtree.nearestKSearch(searchFirstPoint, K, pointIdxNKNSearchFitst, pointNKNSquaredDistanceFirst) > 0 &&
                    kdtree.nearestKSearch(searchSecondPoint, K, pointIdxNKNSearchSecond, pointNKNSquaredDistanceSecond) > 0)
                {
                    for (auto it = pointIdxNKNSearchFitst.begin(); it < pointIdxNKNSearchFitst.end(); ++it)
                    {
                        if (*it >= minIdx)
                        {
                            idx1 = *it;
                            minIdx = *it;
                            break;
                        }
                    }

                    if (minIdx < idx2)
                    {
                        SDOR_LOG_DEBUG << "calculte length failure.";
                        bRet = false;
                    }

                    for (auto it = pointIdxNKNSearchSecond.begin(); it < pointIdxNKNSearchSecond.end(); ++it)
                    {
                        if (*it >= minIdx)
                        {
                            idx2 = *it;
                            minIdx = *it;
                            break;
                        }
                    }

                    if (minIdx < idx1)
                    {
                        bRet = false;
                    }

                    vecDenseEndIndex.emplace_back(idx1, idx2);
                }
                else
                {
                    SDOR_LOG_DEBUG << "Search point failure";
                    bRet = false;
                }
            }

            if (!vecDenseEndIndex.empty())
            {
                vecDenseEndIndex.front().first = 0;
                vecDenseEndIndex.back().second = static_cast<int>(fitPoints.size()) - 1;
            }

            if (bRet)
            {
                float lineLength(0), paintTotalLength(0);
                float32_t tmp(0);
                int32_t i(0);

                //first point the first point of the segment, and second point the last, not the next of the last
                for (auto &endPair : vecDenseEndIndex)
                {
                    while (i < endPair.first)
                    {
                        tmp = normXZ(fitPoints[i+1] - fitPoints[i]);
                        lineLength += tmp;
                        ++i;
                    }

                    while (i < endPair.second)
                    {
                        tmp = normXZ(fitPoints[i+1] - fitPoints[i]);
                        paintTotalLength += tmp;
                        lineLength += tmp;
                        ++i;
                    }
                }

                curveParam.lineLength = lineLength;
                curveParam.paintTotalLength = paintTotalLength;
            }

        }

        return bRet;
    }

    //contain first, doesn't contain last.
    template <class Point3_T>
    bool linearFit(const typename std::vector<Point3_T>::const_iterator first,
                   const typename std::vector<Point3_T>::const_iterator last,
                   cv::Point2f &direction) const
    {
        float num = static_cast<float>(last - first);
        if (num < 2.f)
        {
            SDOR_LOG_WARN << "No input data in function linearFit!";
            return false;
        }

        float x_mean(0), z_mean(0);    //record the mean position of a paint.
        for (auto it = first; it < last; ++it)
        {
            x_mean += it->x;
            z_mean += it->z;
        }

        x_mean /= num;
        z_mean /= num;

        float covariance(0), x_variance(0);
        float x_temp, z_temp;

        for (auto it = first; it < last; ++it)
        {
            x_temp = it->x - x_mean;
            z_temp = it->z - z_mean;

            covariance += x_temp * z_temp;
            x_variance += x_temp * x_temp;
        }

        float x_sign(0);
        int32_t halfNum = static_cast<int32_t>(std::floor(num * 0.5f));
        for (auto it = first; it < first + halfNum; ++it)
        {
            x_sign -= it->x;
        }

        for (auto it = last - 1; it > last - 1 - halfNum; --it)
        {
            x_sign += it->x;
        }

        x_sign -= x_mean * halfNum * 2.f;

        //linear fit, (x, y) is the direction.
        float length = std::sqrt(covariance * covariance + x_variance * x_variance);
        if (length < DBL_MIN)
        {
            direction.x = 0.0f;
            direction.y = 0.0f;
        }
        else if (x_sign > FLT_EPSILON)
        {
            direction.x = x_variance / length;
            direction.y = covariance / length;
        }
        else
        {
            direction.x = -x_variance / length;
            direction.y = -covariance / length;
        }

        return true;
    }

    template <typename T>
    std::vector<size_t> sortIndex(const std::vector<T> &v)
    {
        // initialize original index locations
        size_t lengthV = v.sizez();
        std::vector<size_t>  idx(lengthV, 0);
        for (size_t i = 0; i < lengthV; ++i)
        {
            idx[i] = i;
        }

        // sort indexes based on comparing values in v
        std::stable_sort(idx.begin(), idx.end(), [& v](size_t i1, size_t i2) {return v[i1] < v[i2];});
        return idx;
    }

    template <class Point3_T>
    bool searchNodeIndexInReport(const std::vector<Point3_T>    &vNode,
                                 const std::vector<Point3_T>    &vPoint,
                                 std::vector<int32_t>           &vNodeIndex,
                                 std::vector<int32_t>           &vSecIndex,
                                 std::vector<TRUNCATE_TYPE_E>   &vSegType) const
    {
        if ((vNode.size() & 0x1) != 0 || vNode.empty() || vPoint.empty())
        {
            SDOR_LOG_WARN << "The input data are invalid.";
            return false;
        }

        const float minSquareDistance = 50.f * 50.f;  // the minimum distance is 15 meters

        //Search the most closed point in the circle with radius 50 meters for every node
        std::vector<int32_t> vIndex, vMinIndex;
        vIndex.reserve(vNode.size());
        vMinIndex.reserve(vNode.size());

        for (auto &node : vNode)
        {
            float minDistance = FLT_MAX;
            float tmpDistance, tmp;
            int32_t minIdx(-1);
            for (int32_t j = 0, jEnd = static_cast<int32_t>(vPoint.size()); j < jEnd; ++j)
            {
                tmp = node.x - vPoint[j].x;
                tmpDistance = tmp * tmp;

                tmp = node.z - vPoint[j].z;
                tmpDistance += tmp * tmp;
                if (tmpDistance < minDistance)
                {
                    minDistance = tmpDistance;
                    minIdx = j;
                }
            }

            if (minDistance < minSquareDistance)
            {
                vIndex.emplace_back(minIdx);
            }
            else
            {
                vIndex.emplace_back(-1);
            }
            vMinIndex.emplace_back(minIdx);
        }

        vNodeIndex.clear();
        vSecIndex.clear();
        vSegType.clear();
        vNodeIndex.reserve(vIndex.size());

        for (int32_t i = 0, iEnd = static_cast<int32_t>(vIndex.size()); i < iEnd; i += 2)
        {
            if (vIndex[i] < 0 && vIndex[i+1] < 0)
            {
                Point3_T direction1 = subtract(vNode[i], vPoint.front());
                Point3_T direction2 = subtract(vPoint.back(), vNode[i+1]);
                Point3_T direction3 = subtract(vPoint.back(), vPoint.front());
                if (scalarMultiplyInXOZ(direction1, direction2) > 0.f &&
                    scalarMultiplyInXOZ(direction1, direction3) < 0.f)
                {
                    //the line exists inside the section, so the it don't sliced by the nodes of the section
                    vNodeIndex.emplace_back(0);
                    vNodeIndex.emplace_back(vPoint.size()-1);
                    vSegType.emplace_back(TRUNCATE_TYPE_NO_E);
                }
                else
                {
                    //the line don't pass through the section.
                    continue;
                }
            }
            else if (vIndex[i] >= 0 && vIndex[i+1] < 0)
            {
                Point3_T direction1 = subtract(vNode[i+1], vNode[i]);
                Point3_T direction2 = subtract(vPoint.back(), vNode[i]);
                if (scalarMultiplyInXOZ(direction1, direction2) > 0.0f &&
                    vIndex[i] < static_cast<int32_t>(vPoint.size())-1)
                {
                    Point3_T direction3 = subtract(vPoint.back(), vNode[i+1]);
                    if (scalarMultiplyInXOZ(direction1, direction3) < 0.f)
                    {
                        //one of the end points is in the neighborhood of the first node, the other exists inside the section.
                        vNodeIndex.emplace_back(vIndex[i]);
                        vNodeIndex.emplace_back(vPoint.size()-1);
                        vSegType.emplace_back(TRUNCATE_TYPE_FIRST_E);
                    }
                    else
                    {
                        //one of the end points is in the neighborhood of the first node, the other exists outside the section.
                        //there is a large gap in the reported line.
                        //so one need find the last of point of the previous segment, other than the first point of the next segment.
                        Point3_T direction4;
                        int32_t result = -1;
                        for (auto j = vMinIndex[i+1]; j > vIndex[i]; --j)
                        {
                            direction4 = subtract(vPoint[j], vNode[i+1]);
                            if (scalarMultiplyInXOZ(direction1, direction4) < 0.f)
                            {
                                result = j;
                                break;
                            }
                        }

                        if (result >= 0)
                        {
                            vNodeIndex.emplace_back(vIndex[i]);
                            vNodeIndex.emplace_back(result);
                            vSegType.emplace_back(TRUNCATE_TYPE_FIRST_E);
                        }
                        else
                        {
                            continue;
                        }
                    }
                }
                else
                {
                    //the line don't pass through the section, but does the neighboring one.
                    continue;
                }
            }
            else if (vIndex[i] < 0 && vIndex[i+1] >= 0)
            {
                Point3_T direction1 = subtract(vNode[i+1], vNode[i]);
                Point3_T direction2 = subtract(vNode[i+1], vPoint.front());
                if (scalarMultiplyInXOZ(direction1, direction2) > 0.0f && 0 < vIndex[i+1])
                {
                    Point3_T direction3 = subtract(vNode[i], vPoint.front());
                    if(scalarMultiplyInXOZ(direction1, direction3) < 0.0f)
                    {
                        //one of the end points of the line is in the neighborhood of the second node, the other exists inside the section.
                        vNodeIndex.emplace_back(0);
                        vNodeIndex.emplace_back(vIndex[i+1]);
                        vSegType.emplace_back(TRUNCATE_TYPE_SECOND_E);
                    }
                    else
                    {
                        //one of the end points is in the neighborhood of the first node, the other exists outside the section.
                        //there is a large gap in the reported line.
                        //so one need find the last of point of the previous segment, other than the first point of the next segment.
                        Point3_T direction4;
                        int32_t result = -1;
                        for (auto j = vMinIndex[i]; j < vIndex[i+1]; ++j)
                        {
                            direction4 = subtract(vPoint[j], vNode[i]);
                            if (scalarMultiplyInXOZ(direction1, direction4) > 0.f)
                            {
                                result = j;
                                break;
                            }
                        }

                        if (result >= 0)
                        {
                            vNodeIndex.emplace_back(result);
                            vNodeIndex.emplace_back(vIndex[i+1]);
                            vSegType.emplace_back(TRUNCATE_TYPE_SECOND_E);
                        }
                        else
                        {
                            continue;
                        }
                    }
                }
                else
                {
                    //the line don't pass through the section, but does the neighboring one.
                    continue;
                }
            }
            else
            {
                if (vIndex[i] < vIndex[i+1])
                {
                    //the line pass through the section, and it is sliced by both the end points of the section.
                    vNodeIndex.emplace_back(vIndex[i]);
                    vNodeIndex.emplace_back(vIndex[i+1]);
                }
                else
                {
                    //the line line is belong to the reverse flow lane
                    continue;
                }
                vSegType.emplace_back(TRUNCATE_TYPE_BOTH_E);
            }
            vSecIndex.emplace_back(i >> 1);
        }

        //if the indexes of a section are reverse order, then discard it.
        size_t i(0), j(0);
        const int32_t minPointNumber = 10;

        while (i < vNodeIndex.size())  // i = 2 * j
        {
            if (vNodeIndex[i+1] - vNodeIndex[i] < minPointNumber)
            {
                vNodeIndex.erase(vNodeIndex.begin() + i, vNodeIndex.begin() + i + 2);
                vSecIndex.erase(vSecIndex.begin() + j);
                vSegType.erase(vSegType.begin() + j);
            }
            else
            {
                i += 2;
                ++j;
            }
        }

        return (!vSecIndex.empty());
    }

    template <class Point3_T>
    bool searchNodeIndexInMergeByRef(const std::vector<Point3_T>    &vNode,
                                     const std::vector<Point3_T>    &vDensePoint,
                                     const std::vector<int32_t>     &vSecIndex,
                                     const std::vector<std::vector<Point3_T>>  &vReference,
                                     const std::vector<std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXY>>> &vpKdtree,
                                     std::vector<int32_t>           &vNodeIndex) const
    {
        bool bRet = true;;
        if ((vNode.size() & 0x1) != 0 || vNode.empty() || vDensePoint.empty() || vReference.size() != vpKdtree.size() ||
            vReference.empty() || vNode.size() != vReference.size() << 1)
        {
            SDOR_LOG_WARN << "The input data are invalid.";
            bRet = false;
        }
        else
        {
            const float minDistance = 20.f;  // the minimum distance is 15 meters
            const float minSquareDistance = minDistance * minDistance;
//             const float angleThress = cos(M_PI / 6); //30 degree

            // K nearest neighbor search
            const int32_t K = 1;

            std::vector<int32_t> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            pcl::PointXY searchPoint;

            vNodeIndex.clear();
            std::vector<int> vIndex;
            vIndex.reserve(vSecIndex.size() << 1);

            SDOR_LOG_DEBUG << "vDensePoint.size() = " << vDensePoint.size();

            //idxSecIndex s index of vSecIndex, nlenSecIndex is the size of vSecIndex
            int idxSecIndex(0), nlenSecIndex = static_cast<int>(vSecIndex.size());
            for (auto it = vSecIndex.begin(); it < vSecIndex.end() && bRet; ++it, ++idxSecIndex)
            {
                int secIdx = *it;   //secIdx is the index of the section
                const auto pKd = vpKdtree[secIdx];
//                 const auto &reference = vReference[i];

                std::vector<int32_t> vNearIdx(vDensePoint.size(), -1); //the index of nearest point on reference for every point
                for (int j = 0, jEnd = static_cast<int>(vDensePoint.size()); j < jEnd && bRet; ++j)
                {
                    const auto &point = vDensePoint[j];
                    searchPoint.x = point.x;
                    searchPoint.y = point.z;

                    if (pKd->nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
                    {
                        if (pointNKNSquaredDistance.front() < minSquareDistance)
                        {
                            //the nearest point has been searched
                            vNearIdx[j] = pointIdxNKNSearch.front();
                        }
                    }
                    else
                    {
                        SDOR_LOG_WARN << "The function kdtree.nearestKSearch return false";
                        bRet = false;
                    }
                }//end for j

                if (bRet)
                {
                    int index1(-1), index2(-1);
                    int k = 0;
                    //search the first valid point
                    for (auto it = vNearIdx.begin(); it < vNearIdx.end(); ++it, ++k)
                    {
                        if (*it >= 0)
                        {
                            index1 = k;
                            break;
                        }
                    }

                    k = static_cast<int>(vNearIdx.size()) - 1;
                    //search the last valid point
                    for (auto it = vNearIdx.rbegin(); it < vNearIdx.rend(); ++it, --k)
                    {
                        if (*it >= 0)
                        {
                            index2 = k;
                            break;
                        }
                    }

                    vIndex.emplace_back(index1);
                    vIndex.emplace_back(index2);
                } //end if (bRet)
            } //end for i

#if 0
            {
                SDOR_LOG_DEBUG << "vIndex.size() = " << vIndex.size();
                std::ostringstream outString;
                for (auto it = vIndex.begin(); it < vIndex.end(); ++it)
                {
                    outString << *it << ", ";
                }
                SDOR_LOG_DEBUG << outString;
            }
#endif

            //----------------adjust index of segment point---------------------
            //deal with the first point of the first section
            if (bRet)
            {
                const int index = vSecIndex.front();  //index of the first section
                const auto &node = vNode[index << 1];  //the first node index of the first section
                float minDis(FLT_MAX);
                int idx = searchNearestPoint(node, vDensePoint, minDis);
                if (minDis < minDistance && idx >= 0)
                {
                    vIndex[0] = idx;
                }
            }

            for (int i = 1; i < static_cast<int>(vIndex.size())-1 && bRet; i += 2)  // don't deal with the first and last section
            {
                auto &index1 = vIndex[i];       //index1 represent the last point of this section
                auto &index2 = vIndex[i+1];     //index2 represent the first point of next section
                if (index1 < 0 && index2 > 0)
                {
                    index1 = index2;
                }
                else if (index1 > 0 && index2 < 0)
                {
                    index2 = index1;
                }
                else if (index1 < 0 && index2 < 0)
                {
                    SDOR_LOG_WARN << "There is a bad line.";
                    bRet = false;
                }
                else if (index1 != index2) //index1 >= 0 and index2 >= 0
                {
                    const int &index = vSecIndex[i >> 1];  // index of section
                    const auto &node = vNode[index*2 + 1];

                    float minDis(FLT_MAX);
                    int idx = searchNearestPoint(node, vDensePoint, minDis);

                    if (minDis > minDistance)
                    {
                        idx = (index1 + index2) >> 1;  //idx = (index1 + index2) / 2
                    }

                    index1 = idx;
                    index2 = idx;
                }
                else
                {
                }
            }// end for i

            //deal with the last point of the last section
            if (bRet)
            {
                const int index = vSecIndex.back();    // index of the last section
                const auto &node = vNode[index*2 + 1];  // the second node index of the last section
                float minDis(FLT_MAX);
                int idx = searchNearestPoint(node, vDensePoint, minDis);
                if (minDis < minDistance && idx >= 0)
                {
                    vIndex.back() = idx;
                }
            }

            {
                std::ostringstream outString;
                outString << "Display the indexes of the segment points on the merge Line" << std::endl;
                outString << "vIndex.size() = " << vIndex.size() << std::endl;
                for (auto it = vIndex.begin(); it < vIndex.end(); ++it)
                {
                    outString << *it << ", ";
                }
                SDOR_LOG_DEBUG << outString;
            }

            for (int i = 0; i < nlenSecIndex && bRet; ++i)
            {
                const auto &index1 = vIndex[2*i];
                const auto &index2 = vIndex[2*i+1];

                if (index1 >= 0 && index2 >= 0)
                {
                    vNodeIndex.emplace_back(index1);
                    vNodeIndex.emplace_back(index2);
                }
                else if (index1 >= 0 && index2 < 0)
                {
                    vNodeIndex.emplace_back(index1);
                    vNodeIndex.emplace_back(static_cast<int>(vDensePoint.size()) - 1);
                }
                else if (index1 < 0 && index2 >= 0)
                {
                    vNodeIndex.emplace_back(0);
                    vNodeIndex.emplace_back(index2);
                }
                else
                {
                    vNodeIndex.emplace_back(0);
                    vNodeIndex.emplace_back(static_cast<int>(vDensePoint.size()) - 1);
                }
            }// end for i
        }

        //check whether the output is valid
        for (auto it1 = vNodeIndex.begin(), it2 = it1 + 1; it2 < vNodeIndex.end() && bRet; ++it1, ++it2)
        {
            if (*it2 < *it1)
            {
                SDOR_LOG_WARN << "the line is invalid.";
                bRet = false;
            }
        }
        return bRet;
    }

    template <class Point3_T>
    int searchNearestPoint(const Point3_T &node, const std::vector<Point3_T> &vDensePoint, float &minDis) const
    {
        minDis = FLT_MAX;
        int idx(-1);
        float distance(0);
        for (int j = 0, jEnd = static_cast<int>(vDensePoint.size()); j < jEnd; ++j)
        {
            auto &point = vDensePoint[j];
            distance = normXZ(point - node);

            if (distance < minDis)
            {
                minDis = distance;
                idx = j;
            }
        }

#if 0
        {
            std::ostringstream outString;
            outString << "Display the indexes of the nearest points of nodes and the distance " << std::endl;
            outString << "idx = " << idx << ", minDis = " << minDis;
            SDOR_LOG_INFO << outString;
        }
#endif
        return idx;
    }


public:

    template <class Point3_T>
    bool searchNodeIndexInMerge(const std::vector<Point3_T> &vNode,
                                const std::vector<Point3_T> &vPoint,
                                const std::vector<int32_t>  &vSecIndex,
                                std::vector<int32_t>        &vNodeIndex,
                                std::vector<TRUNCATE_TYPE_E>   &vSegType) const
    {

        if ((vNode.size() & 0x1) != 0)
        {
            SDOR_LOG_WARN << "The number of node must be even.";
            return false;
        }

        vNodeIndex.clear();
        vNodeIndex.reserve(vSecIndex.size() << 1);

        std::vector<uint8_t> vValid;
        vValid.reserve(vNodeIndex.size());

        const float minSquareDistance = 55.f * 55.f;

        Point3_T tmpPoint;
        int32_t idx;

        for (size_t i = 0; i < vSecIndex.size(); ++i)
        {
            idx = vSecIndex[i] << 1;
            float minDistance1 = std::numeric_limits<float>::max();
            float minDistance2 = std::numeric_limits<float>::max();
            float tmpDistance;
            int32_t minIdx1(-1), minIdx2(-1);

            for (size_t j = 0; j < vPoint.size(); ++j)
            {
                //tmpDistance = the distance between vNode[idx] and vPoint[j]
                tmpPoint = subtract(vNode[idx], vPoint[j]);
                tmpDistance = scalarMultiplyInXOZ(tmpPoint, tmpPoint);
                if (tmpDistance < minDistance1)
                {
                    minDistance1 = tmpDistance;
                    minIdx1 = j;
                }

                //tmpDistance = the distance between vNode[idx+1] and vPoint[j]
                tmpPoint = subtract(vNode[idx+1], vPoint[j]);
                tmpDistance = scalarMultiplyInXOZ(tmpPoint, tmpPoint);
                if (tmpDistance < minDistance2)
                {
                    minDistance2 = tmpDistance;
                    minIdx2 = j;
                }
            }

            vNodeIndex.emplace_back(minIdx1);
            vNodeIndex.emplace_back(minIdx2);

            if (minDistance1 < minSquareDistance)
            {
                vValid.emplace_back(1);
            }
            else
            {
                vValid.emplace_back(0);
            }

            if (minDistance2 < minSquareDistance)
            {
                vValid.emplace_back(1);
            }
            else
            {
                vValid.emplace_back(0);
            }
        }

        //safety guard
        auto first = vNodeIndex.begin(), second = vNodeIndex.begin() + 1;
        while (second < vNodeIndex.end())
        {
            if ((*first) > (*second))
            {
                SDOR_LOG_WARN << "The nodes are inverse to the line.";
                return false;
            }
            first += 2;
            second += 2;
        }

        vSegType.clear();
        vSegType.reserve(vSecIndex.size());
        for (size_t i = 0; i < vValid.size(); i += 2)
        {
            if (vValid[i] > 0 && vValid[i + 1] > 0)
            {
                vSegType.emplace_back(TRUNCATE_TYPE_BOTH_E);
            }
            else if (vValid[i] == 0 && vValid[i + 1] > 0)
            {
                vSegType.emplace_back(TRUNCATE_TYPE_SECOND_E);
            }
            else if (vValid[i] > 0 && vValid[i + 1] == 0)
            {
                vSegType.emplace_back(TRUNCATE_TYPE_FIRST_E);
            }
            else
            {
                vSegType.emplace_back(TRUNCATE_TYPE_NO_E);
            }
        }

        return (!vNodeIndex.empty());
    }


    template <class Point3_T>
    bool searchNodeIndexInReportByRef(const std::vector<Point3_T>    &vNode,
                                      const std::vector<Point3_T>    &vDensePoint,
                                      const std::vector<std::vector<Point3_T>>  &vReference,
                                      const std::vector<std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXY>>> &vpKdtree,
                                      std::vector<int32_t>           &vNodeIndex,
                                      std::vector<int32_t>           &vSecIndex) const
    {
        bool bRet = true;
        if ((vNode.size() & 0x1) != 0 || vNode.empty() || vReference.size() != vpKdtree.size() ||
            vReference.empty() || vNode.size() != (vReference.size() << 1))
        {
            SDOR_LOG_WARN << "The input data are invalid.";
            SDOR_LOG_DEBUG << "vNode.size = " << vNode.size() << ", vPoint.size: "
                           << ", vReference.size: " << vReference.size() << ", vpKdtree.size: " << vpKdtree.size();
            bRet = false;
        }
        else
        {
            const float minDistance = 20.f;  // the minimum distance is 15 meters
            const float minSquareDistance = minDistance * minDistance;  // square of the mininum distance
            const float angleThress = cos(M_PI / 5); //36 degree

            // K nearest neighbor search
            const int32_t K = 1;
            std::vector<int32_t> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            pcl::PointXY searchPoint;

            vNodeIndex.clear();
            vSecIndex.clear();

            //vIndex is the index of the nearest point
            std::vector<int> vIndex;
            bool bFirstSection(true);

            for (int i = 0, iEnd = static_cast<int>(vReference.size()); i < iEnd && bRet; ++i)
            {
                //i is the index of the section
                const auto &pKd = vpKdtree[i];
                const auto &reference = vReference[i];

                //search match the points on referece for every point on paint by distance
                std::vector<int32_t> vNearIdx(vDensePoint.size(), -1); //the index of nearest point on reference for every point
                for (int j = 0, jEnd = static_cast<int>(vDensePoint.size()); j < jEnd && bRet; ++j)
                {
                    const auto &point = vDensePoint[j];
                    searchPoint.x = point.x;
                    searchPoint.y = point.z;

                    if (pKd->nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
                    {
                        if (pointNKNSquaredDistance.front() < minSquareDistance)
                        {
                            vNearIdx[j] = pointIdxNKNSearch.front();
                        }
                    }
                    else
                    {
                        SDOR_LOG_WARN << "The function kdtree.nearestKSearch return false";
                        bRet = false;
                    }
                }

#if 0
                {
                    SDOR_LOG_DEBUG << "i = " << i << ", vNearIdx.size() = " << vNearIdx.size();
                    std::ostringstream outString;
                    for (auto it = vNearIdx.begin(); it < vNearIdx.end(); ++it)
                    {
                        outString << *it << ", ";
                    }
                    SDOR_LOG_DEBUG << outString;
                }
#endif

                //the index is ordred
                for (auto it1 = vNearIdx.begin(), it2 = it1 + 1, it3 = it2 + 1; it3 < vNearIdx.end(); ++it1, ++it2, ++it3)
                {
                    if (*it1 > *it2 || (*it2 > *it3 && *it3 >= 0))
                    {
                        *it2 = -1;
                    }
                }

                //cancel some matches by directions of paint and reference.
                if (bRet)
                {
                    Point3_T refDir, paintDir;
                    float paintDivider(0), refDivider(0);
                    for (int j = 0, jEnd = static_cast<int>(vDensePoint.size()) - 1; j < jEnd; ++j)
                    {
                        auto &index1 = vNearIdx[j]; // referece, it need to be modified
                        int k = j + 1;
                        int index2 = vNearIdx[k];

                        if (index1 < 0 || index2 < 0)
                        {
                            // if index2 < 0, then j is isolated
                            index1 = -1;
                            continue;
                        }

                        while (index1 == index2 && k < jEnd)
                        {
                            index2 = vNearIdx[++k];
                        }

                        if (index2 < 0)
                        {
                            // there exist some disturb points, remove them.
                            std::for_each(vNearIdx.begin() + j, vNearIdx.begin() + k, [](int &elem){ elem = -1; });
                            continue;
                        }
                        else if (k > jEnd)
                        {
                            // index1 is the last point of the referece, so remove the points after j in paint.
                            std::for_each(vNearIdx.begin() + j, vNearIdx.end(), [](int &elem){ elem = -1; });
                        }
                        else
                        {
                        }

                        paintDir = subtract(vDensePoint[k], vDensePoint[j]); // direction of road
                        refDir = subtract(reference[index2], reference[index1]); // direction of reference

                        paintDivider = normXZ(paintDir);
                        refDivider = normXZ(refDir);

                        if (paintDivider < FLT_EPSILON || refDivider < FLT_EPSILON)
                        {
                            index1 = -1;
                        }
                        else
                        {
                            //normalization
                            paintDir = paintDir / paintDivider;
                            refDir = refDir / refDivider;

                            if (scalarMultiplyInXOZ(paintDir, refDir) < angleThress)
                            {
                                index1 = -1;
                            }
                        }
                    } // end for j

                    if (paintDivider < FLT_EPSILON || refDivider < FLT_EPSILON ||
                        scalarMultiplyInXOZ(paintDir, refDir) < angleThress)
                    {
                        vNearIdx.back() = -1;
                    }

                }// end if (bRet)

#if 0
                {
                    SDOR_LOG_DEBUG << "vNearIdx.size() = " << vNearIdx.size();
                    std::ostringstream outString;
                    for (auto it = vNearIdx.begin(); it < vNearIdx.end(); ++it)
                    {
                        outString << *it << ", ";
                    }
                    SDOR_LOG_DEBUG << outString;
                }
#endif

                //record the segemnt point
                if (bRet)
                {
                    int index1(-1), index2(-1);
                    std::vector<std::pair<int, int>> pairSegIdx; //it may contain some valid segemnts, but in most cases it contains only one.
                    bool bValid(true);  //true for search first positive, false for last positive
                    for (auto itFirst = vNearIdx.begin(), it = itFirst; it < vNearIdx.end(); ++it)
                    {
                        if (bValid && *it >= 0)
                        {
                            //for searching the first positive value
                            index1 = std::distance(itFirst, it);
                            bValid = false;
                        }

                        if (!bValid && *it < 0)
                        {
                            //for searching the last positive value
                            index2 = std::distance(itFirst, it) - 1;
                            bValid = true;
                            pairSegIdx.emplace_back(index1, index2); //add one valid segment
                        }
                    }

                    if (!bValid && index1 >= 0)
                    {
                        //in some cases elements of the latter part of vNearIdx are positive.
                        index2 = static_cast<int>(vNearIdx.size()) -1;
                        pairSegIdx.emplace_back(index1, index2); //add one valid segment
                    }

                    //Now pairSegIdx contains some pair, but one need only one.
                    //So we need select the longest one.
                    //However we are not sure whether we have sliced a segment with noise into two segments
                    //So first of all we check them once
                    for (auto it1 = pairSegIdx.begin(), it2 = it1 + 1; it2 < pairSegIdx.end(); )
                    {
                        int diff = it2->first - it1->second;
                        if (diff >= 0 && diff < 10)
                        {
                            it1->second = it2->second;
                            it2 = pairSegIdx.erase(it2);
                            it1 = it2 - 1;
                        }
                        else
                        {
                            ++it1;
                            ++it2;
                        }
                    }

                    //selece the longest segment
                    index1 = -1;
                    index2 = -2;
                    for (auto it = pairSegIdx.begin(); it < pairSegIdx.end(); ++it)
                    {
                        if (it->second - it->first > index2 - index1)
                        {
                            index1 = it->first;
                            index2 = it->second;
                        }
                    }

//                     SDOR_LOG_DEBUG << "index1 = " << index1 << ", index2 = " << index2 << ", iEnd = " << iEnd << ", i = " << i;

                    if (index1 > 0 || index2 > 0)
                    {
                        vIndex.emplace_back(index1);
                        vIndex.emplace_back(index2);
                        vSecIndex.emplace_back(i);
                        bFirstSection = false;
                    }
                    else if (!bFirstSection)
                    {
                        //there is some erros in this seciton, so remove the residual sections
                        break;
                    }
                    else
                    {
                    }
                }//end if
            }// end for i

#if 0
            {
                SDOR_LOG_DEBUG << "vIndex.size() = " << vIndex.size();
                std::ostringstream outString;
                for (auto it = vIndex.begin(); it < vIndex.end(); ++it)
                {
                    outString << *it << ", ";
                }
                SDOR_LOG_DEBUG << outString;
            }
#endif

            if (bRet && !vSecIndex.empty())
            {
                //deal with the first point of the first section
                const int index = vSecIndex.front();  //index of the first section
                const auto &node = vNode[index << 1];  //the first node index of the first section
                float minDis(FLT_MAX);
                int idx = searchNearestPoint(node, vDensePoint, minDis);
                if (minDis < minDistance && idx >= 0)
                {
                    vIndex[0] = idx;
                }
            }

            // adjust the index of segment Point
            for (int i = 0; i < static_cast<int>(vSecIndex.size())-1 && bRet; )
            {
                auto &index1 = vIndex[(i << 1) + 0x1]; //index of the second node of this section, vIndex[2 * i + 1]
                auto &index2 = vIndex[(i << 1) + 0x2]; //index of the first node of next section, vIndex[2 * i + 2]

                if (index1 < 0 && index2 >= 0)
                {
                    index1 = index2;
                }
                else if (index1 >= 0 && index2 < 0)
                {
                    index2 = index1;
                }
                else if (index1 < 0 && index2 < 0)
                {
                    SDOR_LOG_WARN << "The trajectory cant' be searched by the paint in section " << vSecIndex[i];
                    bRet = false;
                }
                else if (index1 != index2)
                {
                    const int &index = vSecIndex[i];  // index of section

                    //the second node of the section and the first node of next section.
                    const auto &node = vNode[(index << 1) + 0x1];  //vNode[2 * index + 1]

                    float minDis(FLT_MAX);
                    int idx = searchNearestPoint(node, vDensePoint, minDis);

                    if (minDis > minDistance)
                    {
//                         idx = (index1 + index2) >> 1;
                        SDOR_LOG_WARN << "The nodes don't match the trajectory.";
                        SDOR_LOG_DEBUG << "Minumum distance between node and trajectory is " << minDis;
                        SDOR_LOG_INFO << "index if division is " << index;
                        bRet = false;
                    }

                    index1 = idx;
                    index2 = idx;

                    if (index1 <= vIndex[i << 1])
                    {
                        //remove the i-th section
                        vSecIndex.erase(vSecIndex.begin() + i);
                        vIndex.erase(vIndex.begin() + 2*i, vIndex.begin() + 2*i + 2);// erase elements from 2*i to 2*i + 2
                    }
                    else if (index2 >= vIndex[2 * i + 3])
                    {
                        //remove the (i+1)th section
                        vSecIndex.erase(vSecIndex.begin() + i+1);
                        vIndex.erase(vIndex.begin() + 2*(i+1), vIndex.begin() + 2*(i+1) + 2);
                        ++i;
                    }
                    else
                    {
                        ++i;
                    }
                }//end if
                else
                {
                    ++i;
                }
            }//end for i

            if (bRet && !vSecIndex.empty())
            {
                //deal with the last point of the last section
                const int index = vSecIndex.back();    // index of the last section
                const auto &node = vNode[index*2 + 1];  // the second node index of the last section
                float minDis(FLT_MAX);
                int idx = searchNearestPoint(node, vDensePoint, minDis);
                if (minDis < minDistance && idx >= 0)
                {
                    vIndex.back() = idx;
                }
            }

#if 0
            {
                std::ostringstream outString;
                outString << "Display the indexes of the segment points on the reported line, vIndex.size() = " << vIndex.size() << std::endl;
                for (auto it = vIndex.begin(); it < vIndex.end(); ++it)
                {
                    outString << *it << ", ";
                }
                SDOR_LOG_DEBUG << outString;
            }
#endif

            for (size_t i = 0; i < vSecIndex.size() && bRet;)
            {
                auto index1 = vIndex[2*i];
                auto index2 = vIndex[2*i+1];

                if (index1 <= index2 || index2 < 0)
                {
                    if (index1 >= 0 && index2 >= 0)
                    {
                        vNodeIndex.emplace_back(index1);
                        vNodeIndex.emplace_back(index2);
                    }
                    else if (index1 >= 0 && index2 < 0)
                    {
                        vNodeIndex.emplace_back(index1);
                        vNodeIndex.emplace_back(static_cast<int>(vDensePoint.size()) - 1);
                    }
                    else if (index1 < 0 && index2 >= 0)
                    {
                        vNodeIndex.emplace_back(0);
                        vNodeIndex.emplace_back(index2);
                    }
                    else
                    {
                        vNodeIndex.emplace_back(0);
                        vNodeIndex.emplace_back(static_cast<int>(vDensePoint.size()) - 1);
                    }

                    ++i;
                }
                else
                {
                    SDOR_LOG_DEBUG << "index1 = " << index1 << ", index2 = " << index2;
                    vSecIndex.erase(vSecIndex.begin() + i);
                }
            }//end for i
        }//end else

//         SDOR_LOG_DEBUG << "vSecIndex.size() = " << vSecIndex.size();
//         SDOR_LOG_DEBUG << "vNodeIndex.size() = " << vNodeIndex.size();

#if 1
        //delete some data
        std::vector<int> vRemoveIdx;
        for (size_t i = 0; i < vSecIndex.size() && bRet; ++i)
        {
            if (vNodeIndex[2*i] >= vNodeIndex[2*i+1])
            {
                vRemoveIdx.emplace_back(i);
            }
        }

        if (!vRemoveIdx.empty())
        {
            std::ostringstream outString;
            outString << "The segment result of Reported line in these divisions is invalid: "
                      << ", vRemoveIdx.size() = " << vRemoveIdx.size() << std::endl;
            for (auto it = vRemoveIdx.begin(); it < vRemoveIdx.end(); ++it)
            {
                outString << *it << ", ";
            }
            SDOR_LOG_DEBUG << outString;
        }

        for (auto it = vRemoveIdx.rbegin(); it != vRemoveIdx.rend(); ++it)
        {
            vNodeIndex.erase(vNodeIndex.begin() + 2 * (*it) + 1);
            vNodeIndex.erase(vNodeIndex.begin() + 2 * (*it));
            vSecIndex.erase(vSecIndex.begin() + *it);
        }
#endif

        //check whether the output is valid
        for (auto it1 = vNodeIndex.begin(), it2 = it1 + 1; it2 < vNodeIndex.end() && bRet; ++it1, ++it2)
        {
            if (*it2 < *it1)
            {
                SDOR_LOG_WARN << "The vNodeIndex is error.";
                {
                    std::ostringstream outString;
                    outString << "Display the indexes of the segment points on the reported line, vIndex.size() = " << vNodeIndex.size() << std::endl;
                    for (auto it = vNodeIndex.begin(); it < vNodeIndex.end(); ++it)
                    {
                        outString << *it << ", ";
                    }
                    SDOR_LOG_WARN << outString;
                }
                bRet = false;
            }
        }
        return bRet;
    }

    template <class Point3_T>
    bool getEdges(const std::vector<Point3_T> &reference, Point3_T &tailEdge, Point3_T &headEdge) const
    {
        // The tube is shown below
        /*
              ^                                     ^
  tailEdge    |              reference              | headEdge
  tailPointer |------------------------------------>| headPointer
              |                                     |
              |                                     |   */

        const auto num = static_cast<int32_t>(reference.size());
        reset(headEdge);  //set as (0, 0, 0)
        reset(tailEdge);

        if (num >= 2)
        {
           Point3_T headDirect, tailDirect;

           if (num > 10)
           {
                headDirect = reference[num - 1] - reference[num - 10];
                tailDirect = reference[9] - reference[0];
           }
           else
           {
                headDirect = reference[num - 1] - reference[num -2];
                tailDirect = reference[1] - reference[0];
           }

           //y is height, it is useless, don't care it
           headEdge.x = -headDirect.z;
           headEdge.z =  headDirect.x;
           tailEdge.x = -tailDirect.z;
           tailEdge.z =  tailDirect.x;

           return true;
        }
        else
        {
            SDOR_LOG_WARN << "vDensePoint.size() = " << num;

            return false;
        }
    }

    template <class Point3_T>
    bool inTube(const Point3_T &point,
                const Point3_T &tailEdge,
                const Point3_T &headEdge,
                const Point3_T &tailPoint,
                const Point3_T &headPoint) const
    {
        const auto tailPointer = point - tailPoint;
        const auto headPointer = point - headPoint;

        return std::isgreaterequal(crossInXOZ(tailPointer, tailEdge), 0.0) &&
               std::isgreaterequal(crossInXOZ(headEdge, headPointer), 0.0);
    }

    // The following graph depicts two kinds of in/out status for tail edge
    //
    //     tailEdge                               <--------------------+
    //        ^                                            in          |  out
    //        |                          or                            |
    //   out  |    in                                                  V
    //        +------------------>                                  tailEdge


    // The following graph depicts two kinds of in/out status for head edge
    //
    //                       headEdge             <---------------------
    //                           ^            out |        in
    //                           |       or       |
    //                in         | out            V
    //        ------------------->             headEdge

    template <typename PointXYZ>
    inline bool isInTailEdge(const PointXYZ &point,
                             const PointXYZ &tailEdge,
                             const PointXYZ &tailPoint) const
    {
        return std::isgreaterequal(crossInXOZ(subtract(point, tailPoint), tailEdge), 0.0);
    }

    template <typename PointXYZ>
    inline bool isOutTailEdge(const PointXYZ &point,
                              const PointXYZ &tailEdge,
                              const PointXYZ &tailPoint) const
    {
        return !isInTailEdge(point, tailEdge, tailPoint);
    }

    template <typename PointXYZ>
    inline bool isInHeadEdge(const PointXYZ &point,
                             const PointXYZ &headEdge,
                             const PointXYZ &headPoint) const
    {
        return std::islessequal(crossInXOZ(subtract(point, headPoint), headEdge), 0.0);
    }

    template <typename PointXYZ>
    inline bool isOutHeadEdge(const PointXYZ &point,
                              const PointXYZ &headEdge,
                              const PointXYZ &headPoint) const
    {
        return !isInHeadEdge(point, headEdge, headPoint);
    }

    using IndexConstIt = std::vector<int32_t>::const_iterator;
    using IndexIt      = std::vector<int32_t>::iterator;

    template <class PointXYZ>
    bool checkMatchPiece(const std::vector<PointXYZ> &points,
                         const std::vector<PointXYZ> &reference,
                         IndexConstIt pointIdxStartIt,
                         IndexConstIt pointIdxEndIt,
                         IndexConstIt refIdxStartIt,
                         IndexConstIt refIdxEndIt,
                         const std::vector<int32_t> &pointIndices,
                         const std::vector<int32_t> &refIndices,
                         const float32_t angleTH) const
    {
        // A lambda to check whether points and reference share the same direction
        auto checkDirectionPiece([this](const std::vector<PointXYZ> &points,
                                        const std::vector<PointXYZ> &reference,
                                        IndexConstIt pointIdxStartIt,
                                        IndexConstIt pointIdxEndIt,
                                        IndexConstIt refIdxStartIt,
                                        IndexConstIt refIdxEndIt,
                                        const float32_t angleTH)
        {
            // Get the first and last matched reference indices
            const auto firstMatchedRefIdx = *refIdxStartIt;
            const auto lastMatchedRefIdx  = *std::prev(refIdxEndIt);

            // If two many points match the same reference point, then it's better
            // not to keep this piece
            const auto NUM_SAME_MATCH_MAX = 10;
            if (firstMatchedRefIdx == lastMatchedRefIdx &&
                std::distance(pointIdxStartIt, pointIdxEndIt) > NUM_SAME_MATCH_MAX)
            {
                return false;
            }

            const auto refIdxLow  = std::min(firstMatchedRefIdx, lastMatchedRefIdx);
            const auto refIdxHigh = std::max(firstMatchedRefIdx, lastMatchedRefIdx);
            const auto &refStart  = reference[refIdxLow];
            const auto &refEnd    = reference[refIdxHigh];

            // Get the corresponding iterators to the first and last matched points
            const auto &pointStart = points[*pointIdxStartIt];
            const auto &pointEnd   = points[*std::prev(pointIdxEndIt)];

            // Check direction of matched points and reference
            return this->checkDirection(pointStart, pointEnd, refStart, refEnd, angleTH);
        });

        // Number of point matched
        const auto numPoint = std::distance(pointIdxStartIt, pointIdxEndIt);

        const auto NUM_POINT_TOTAL = 1000U; // point number threshold for piece-wise match
        const auto NUM_POINT_PIECE = 500U;  // number of point for each piece

        auto itPointStart = pointIdxStartIt;
        auto itRefStart   = refIdxStartIt;

        std::vector<int8_t> dirFlags; // direction flag for all pieces

        // If too many points are matched, in order to calculate direction
        // more precisely, divide matching part to small pieces
        if (numPoint >= NUM_POINT_TOTAL)
        {
            const auto numPiece = static_cast<int32_t>(numPoint / NUM_POINT_PIECE);

            if (numPiece > 1)
            {
                // Calculate directions of the first n - 1 pieces
                for (auto i = 0; i < numPiece - 1; ++i)
                {
                    auto itPointEnd = std::next(itPointStart, NUM_POINT_PIECE);
                    auto itRefEnd   = std::next(itRefStart,   NUM_POINT_PIECE);

                    dirFlags.emplace_back(checkDirectionPiece(points,
                                                              reference,
                                                              itPointStart,
                                                              itPointEnd,
                                                              itRefStart,
                                                              itRefEnd,
                                                              angleTH));
                    itPointStart = itPointEnd;
                    itRefStart   = itRefEnd;
                }
            }
        }

        // Last piece or first piece
        dirFlags.emplace_back(checkDirectionPiece(points,
                                                  reference,
                                                  itPointStart,
                                                  pointIdxEndIt,
                                                  itRefStart,
                                                  refIdxEndIt,
                                                  angleTH));

        const auto ok = std::count(dirFlags.cbegin(), dirFlags.cend(), 1)
                      > (static_cast<int32_t>(dirFlags.size()) >> 1);
        return ok;
    }

    template <typename PointXYZ>
    bool rematch(const std::vector<PointXYZ> &points,
                 const std::vector<PointXYZ> &reference,
                 const int32_t pointIdxStart,
                 const int32_t pointIdxEnd,
                 const int32_t refIdxStart,
                 const int32_t refIdxEnd,
                 const float32_t matchThreshold,
                 std::vector<int32_t> &pointIndicesMatched,
                 std::vector<int32_t> &refIndicesMatched) const
    {
        if (pointIdxStart >= pointIdxEnd ||
            pointIdxEnd   >= static_cast<int32_t>(points.size()) ||
            refIdxStart   >  refIdxEnd)
        {
            return false;
        }

        // Find the correct match for each point
        for (auto i = pointIdxStart; i < pointIdxEnd; ++i)
        {
            for (auto j = refIdxStart; j < refIdxEnd; ++j)
            {
                const auto distance = normXZ(subtract(points[i], reference[j]));

                if (distance < matchThreshold)
                {
                    pointIndicesMatched.emplace_back(i);
                    refIndicesMatched.emplace_back(j);

                    break;
                }
            }
        }

        return true;
    }

    template <typename PointXYZ>
    void tweakInvalidMatches(const std::vector<PointXYZ> &points,
                             const std::vector<PointXYZ> &reference,
                             std::vector<int32_t> &pointIndicesMatched,
                             std::vector<int32_t> &refIndicesMatched) const
    {
        const auto numRefIdx   = refIndicesMatched.size();
        const auto numPointIdx = pointIndicesMatched.size();

        if (numRefIdx == 0           ||
            numRefIdx != numPointIdx ||
            points.empty()           ||
            reference.empty())
        {
            return;
        }

        std::vector<int32_t> refIdxAdjDiffs(numRefIdx, 0);

        // Compute adjacent difference of matched reference indices
        std::adjacent_difference(refIndicesMatched.cbegin(),
                                 refIndicesMatched.cend(),
                                 refIdxAdjDiffs.begin());

        const auto DIFF_TH = 10; // A threshold used to find jump in matched reference indices;

        // A lambda to test whether a difference is a big change
        auto isJumpDiff([](const int32_t diff) noexcept
                        {
                            return std::abs(diff) >= DIFF_TH;
                        });

        // Find all jump differences

        // Note that the first difference is always the first index itself
        std::vector<IndexConstIt> jumpDiffIts;
        for (auto itDiff = std::next(refIdxAdjDiffs.cbegin()); itDiff != refIdxAdjDiffs.cend(); ++itDiff)
        {
            if (isJumpDiff(*itDiff))
            {
                jumpDiffIts.emplace_back(itDiff);
            }
        }

        // No wrong matches found, no need for special tweak
        if (jumpDiffIts.empty())
        {
            return;
        }

        // Add begin and end to facilitate the loop below
        jumpDiffIts.insert(jumpDiffIts.begin(), refIdxAdjDiffs.cbegin());
        jumpDiffIts.emplace_back(refIdxAdjDiffs.cend());

        // For simplicity, we take a copy strategy rather than modify the
         // original indices containers
        std::vector<int32_t> pointIndicesTweaked; // valid point matches
        std::vector<int32_t> refIndicesTweaked;   // valid indices matches
        pointIndicesTweaked.reserve(numRefIdx);
        refIndicesTweaked.reserve(numRefIdx);

        const auto ANG_TH = std::cos(M_PI / 5);   // threshold for same direction
        const auto MATCH_DIST_TH = 30.0F;         // distance threshold for match

        const auto numJumpDiffIt = jumpDiffIts.size();
        for (auto i = 1U; i < numJumpDiffIt; ++i)
        {
            // Compute offsets of start and end iterators to reference
            const auto offsetStart = std::distance(refIdxAdjDiffs.cbegin(), jumpDiffIts[i - 1]);
            const auto offsetEnd   = std::distance(refIdxAdjDiffs.cbegin(), jumpDiffIts[i]);

            // Compute start and end reference and point iterators according to offsets
            auto refIdxItStart   = std::next(refIndicesMatched.cbegin(), offsetStart);
            auto refIdxItEnd     = std::next(refIndicesMatched.cbegin(), offsetEnd);
            auto pointIdxItStart = std::next(pointIndicesMatched.cbegin(), offsetStart);
            auto pointIdxItEnd   = std::next(pointIndicesMatched.cbegin(), offsetEnd);

            // Check whether the piece of matching should be kept
            const auto keep = checkMatchPiece(points,
                                              reference,
                                              pointIdxItStart,
                                              pointIdxItEnd,
                                              refIdxItStart,
                                              refIdxItEnd, // refIdxItEnd is excluded
                                              pointIndicesMatched,
                                              refIndicesMatched,
                                              ANG_TH);

            if (keep)
            {
                refIndicesTweaked.insert(refIndicesTweaked.cend(), refIdxItStart, refIdxItEnd);
                pointIndicesTweaked.insert(pointIndicesTweaked.cend(), pointIdxItStart, pointIdxItEnd);
            }
            else
            {
                // Re-match the piece
                // End of point and reference are excluded
                rematch(points,
                        reference,
                        *pointIdxItStart,
                        (pointIdxItEnd != pointIndicesMatched.cend() ? *pointIdxItEnd : pointIndicesMatched.back() + 1) ,
                        (refIdxItStart != refIndicesMatched.cbegin() ? *std::prev(refIdxItStart) : 0),
                        (refIdxItEnd   != refIndicesMatched.cend()   ? *refIdxItEnd : reference.size()),
                        MATCH_DIST_TH,
                        pointIndicesTweaked,
                        refIndicesTweaked);
            }
        }

        // Update the output indices
        pointIndicesMatched.swap(pointIndicesTweaked);
        refIndicesMatched.swap(refIndicesTweaked);
    }

    template <class Point3_T>
    bool searchPointByDist(const std::vector<Point3_T> &vDensePoint,
                           const std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXY>> pKdtree,
                           const std::vector<Point3_T> &reference,
                           std::vector<int> &vPaintIdx,
                           std::vector<int> &vRefIdx,
                           std::vector<float> &vNearestDis) const
    {
        vPaintIdx.clear();
        vNearestDis.clear();
        vRefIdx.clear();

        const auto DIST_TH   = 30.0F; // distance threshold for matching
        const auto DIST_TH_2 = DIST_TH * DIST_TH;

        const auto numPaintPoint = static_cast<int>(vDensePoint.size());
        const auto numRefPoint   = static_cast<int>(reference.size());

        if (numPaintPoint > 1 && numRefPoint > 1)
        {
            vNearestDis.reserve(numPaintPoint);
            vPaintIdx.reserve(numRefPoint);
            vRefIdx.reserve(numRefPoint);

            // K nearest neighbor search
            const int32_t K = 1;
            std::vector<int32_t> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            pcl::PointXY searchPoint;

            // Get indices of paint points that are close enough to reference
            for (int i = 0; i < numPaintPoint; ++i)
            {
                const Point3_T &point = vDensePoint[i];
                searchPoint.x = point.x;
                searchPoint.y = point.z;

                if (pKdtree->nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
                {
                    float sqrDis = pointNKNSquaredDistance.front();

                    if ((sqrDis < DIST_TH_2))
                    {
                        vNearestDis.emplace_back(std::sqrt(sqrDis));

                        vPaintIdx.emplace_back(i);
                        vRefIdx.emplace_back(pointIdxNKNSearch.front());
                    }
                }
                else
                {
                    SDOR_LOG_WARN << "The function kdtree.nearestKSearch return false";
                    break;
                }
            }

            // Tweak: For segmentations that use geometry division as references,
            // paints may incorrectly matched since the references may be badly curved.
            // This tweak is used to replace the invalid matches between reference and
            // paint with correct ones
            tweakInvalidMatches(vDensePoint, reference, vPaintIdx, vRefIdx);
        }
        else
        {
            SDOR_LOG_WARN << "reference.size() = " << reference.size();
            SDOR_LOG_WARN << "vDensePoint.size() = " << vDensePoint.size();
        }

        return !vPaintIdx.empty() && (vPaintIdx.size() == vRefIdx.size());
    }

    template <class Point3_T>
    bool checkDirection(const Point3_T &pStart,
                        const Point3_T &pEnd,
                        const Point3_T &refStart,
                        const Point3_T &refEnd,
                        const float &angleThress) const
    {
        Point3_T pDirection = pEnd - pStart;
        Point3_T refDirection = refEnd - refStart;

        float pLength = normXZ(pDirection);
        float refLength = normXZ(refDirection);

        if (pLength < FLT_EPSILON || refLength < FLT_EPSILON)
        {
            return true;
        }
        else
        {
            float cosAngle = scalarMultiplyInXOZ(pDirection, refDirection) / (pLength * refLength);
            return (cosAngle > angleThress);
        }
    }

    template <class ForwardIter>
    void getAverageAndVariance(const ForwardIter first,
                               const ForwardIter last,
                               float &average,
                               float &variance) const
    {
        int num = std::distance(first, last);
        if (num > 1)
        {
            average = std::accumulate(first, last, 0.f);
            average = average / num;

            double varianceSum = 0;
            for (auto it = first; it < last; ++it)
            {
                float tmp = *it - average;
                varianceSum += tmp * tmp;
            }

            variance = varianceSum / (num - 1);
        }
        else
        {
            average = 0;
            variance = std::numeric_limits<float>::max();
        }
    }

    template <typename ForwardIter>
    float getLength(ForwardIter first, ForwardIter last) const
    {
        float length(0.0f);

        for (auto it1 = first; it1 != last; ++it1)
        {
            const auto it2 = std::next(it1);

            if (it2 != last)
            {
                length += normXZ(*it1 - *it2);
            }
        }

        return length;
    }

    template <class Point3_T>
    void removeInvalidSegment(const std::vector<Point3_T> &vDensePoint,
                              const std::vector<float> &vNearSqrDis,
                              std::vector<std::pair<int, int>> &vEndIdx) const
    {
        const auto numIdxPair = vEndIdx.size();

        if (numIdxPair > 1)
        {
            const float minLength = 0.f;
            const float maxVariance = 10.f;
            const float maxAverageDiff = 2.f;

            std::vector<float> vAverage(numIdxPair, 0);
            std::vector<float> vVariance(numIdxPair, 0);
            std::vector<int> vRemove;

            for (auto i = 0u; i < numIdxPair; ++i)
            {
                std::pair<int, int> &seg = vEndIdx[i];
                getAverageAndVariance(vNearSqrDis.begin() + seg.first,
                                      vNearSqrDis.begin() + seg.second,
                                      vAverage[i],
                                      vVariance[i]);
            }

            auto itOptimal = std::min_element(vVariance.begin(), vVariance.end());
            int nOptimal = std::distance(vVariance.begin(), itOptimal);

            for (auto i = 0u; i < numIdxPair; ++i)
            {
                std::pair<int, int> &seg = vEndIdx[i];
                if (getLength(vDensePoint.cbegin() + seg.first, vDensePoint.cbegin() + seg.second) < minLength)
                {
//                     SDOR_LOG_DEBUG << "This segment is too short.";
                    vRemove.emplace_back(i);
                    continue;
                }

                if (vVariance[i] > maxVariance)
                {
//                     SDOR_LOG_DEBUG << "The variance is " << vVariance[i];
//                     SDOR_LOG_DEBUG << "[" << seg.first << ", " << seg.second << "]";
//                     SDOR_LOG_DEBUG << "index of referece is " << index;
                    vRemove.emplace_back(i);
                    continue;
                }

                if (std::abs(vAverage[i] - vAverage[nOptimal]) > maxAverageDiff)
                {
//                     SDOR_LOG_DEBUG << "the diference between average is " << std::abs(vAverage[i] - vAverage[nOptimal]);
//                     SDOR_LOG_DEBUG << "[" << seg.first << ", " << seg.second << "]";
//                     SDOR_LOG_DEBUG << "index of referece is " << index;
//                     SDOR_LOG_DEBUG << "optimal is [" << vEndIdx[nOptimal].first << " " << vEndIdx[nOptimal].second << "]";
                    vRemove.emplace_back(i);
                    continue;
                }
            }

            for (auto it = vRemove.rbegin(); it < vRemove.rend(); ++it)
            {
                vEndIdx.erase(vEndIdx.begin() + *it);
            }
        }
    }

    template <typename PointXYZ, typename ForwardIterator>
    inline bool
    isRefHeadMatched(
        const std::vector<PointXYZ> &reference,
        ForwardIterator itStart,
        ForwardIterator itEnd) const
    {
        const auto DISTINCT_DIST_MIN = 5 * 1e-2;

        if (!reference.empty() && itEnd != itStart)
        {
            const auto itMin = std::min_element(itStart, itEnd);

            // If the first matched reference point is too close to the head, then
            // the head itself is considered match
            if (*itMin == 0 ||
                normXZ(subtract(reference.front(), reference[*itMin])) < DISTINCT_DIST_MIN)
            {
                return true;
            }
        }

        return false;
    }

    template <typename PointXYZ, typename ForwardIterator>
    inline bool
    isRefTailMatched(
        const std::vector<PointXYZ> &reference,
        ForwardIterator itStart,
        ForwardIterator itEnd) const
    {
        const auto DISTINCT_DIST_MIN = 5 * 1e-2;

        if (!reference.empty() && itEnd != itStart)
        {
            const auto itMax = std::max_element(itStart, itEnd);

            // If the last matched reference point is too close to the head, then
            // the head itself is considered match
            if (*itMax == static_cast<int32_t>(reference.size() - 1) ||
                 normXZ(subtract(reference.back(), reference[*itMax])) < DISTINCT_DIST_MIN)
            {
                return true;
            }
        }

        return false;
    }

    template <typename PointXYZ>
    void removeRedundantRanges(std::vector<std::pair<int32_t, int32_t>> &ranges,
                               const std::vector<PointXYZ> &points,
                               const std::vector<PointXYZ> &reference,
                               const std::vector<int32_t> &paintIndices,
                               const std::vector<int32_t> &refIndices,
                               const PointXYZ &tailEdge,
                               const PointXYZ &headEdge) const
    {
        // Identical data size is required to maintain correspondence
        if (paintIndices.size() != refIndices.size())
        {
            SDOR_LOG_WARN << "Inconsistent indices size: "
                          << "Paint: "     << paintIndices.size() << ", "
                          << "reference: " << refIndices.size();
            return;
        }

        // The reference is not fully covered, no redundant points matched actually
        if (!isRefHeadMatched(reference, refIndices.cbegin(), refIndices.cend()) &&
            !isRefTailMatched(reference, refIndices.cbegin(), refIndices.cend()))
        {
            return;
        }

        const auto DISTINCT_DIST_MIN = 5.0 * 1e-2;

        auto itRange = ranges.begin();
        while (itRange != ranges.end())
        {
            auto &range = *itRange;

            auto startPaintIdxIt = std::find(paintIndices.cbegin(), paintIndices.cend(), range.first);
            auto endPaintIdxIt   = std::find(paintIndices.cbegin(), paintIndices.cend(), range.second - 1);

            if (startPaintIdxIt == paintIndices.cend() || endPaintIdxIt == paintIndices.cend())
            {
                SDOR_LOG_WARN << "Range and paint indices are inconsistent.";

                continue;
            }

            auto startRefIdxIt = std::next(refIndices.cbegin(),
                                           std::distance(paintIndices.cbegin(), startPaintIdxIt));
            auto endRefIdxIt   = std::next(refIndices.cbegin(),
                                           std::distance(paintIndices.cbegin(), endPaintIdxIt));

            if (isRefHeadMatched(reference, startRefIdxIt, endRefIdxIt + 1))
            {
                auto firstIn = -1;

                // Find the first point that is in tail edge
                for (auto i = range.first; i < range.second; ++i)
                {
                    if (isInTailEdge(points[i], tailEdge, reference.front()))
                    {
                        firstIn = i;
                        break;
                    }
                }

                // There exists a hop from OUT to IN
                if (firstIn > range.first)
                {
                    // Remove the heading out points
                    range.first = firstIn;
                }
                else if (-1 == firstIn) // All points are out
                {
                    // Check whether this range can be removed

                    // In the following case, the corresponding range(denoted by "s" and "e")
                    // should be removed
                    //           ^
                    // s--->---e |
                    //           +----------------------->----
                    //
                    // To take the following situation into account, we only need to Check
                    // the start index of reference
                    //
                    // s----->----e ^
                    //             /
                    //            /
                    //           /
                    //          +------------------------>----
                    //
                    //if (0 == *startRefIdxIt /*&& 0 == *endRefIdxIt*/)
                    if (normXZ(subtract(reference[*startRefIdxIt], reference.front())) < DISTINCT_DIST_MIN)
                    {
                        itRange = ranges.erase(itRange);

                        continue;
                    }
                }
                else
                {
                }
            }

            // Ditto for end
            if (isRefTailMatched(reference, startRefIdxIt, endRefIdxIt + 1))
            {
                auto lastIn = range.second;

                for (auto i = range.second - 1; i >= range.first; --i)
                {
                    if (isInHeadEdge(points[i], headEdge, reference.back()))
                    {
                        lastIn = i;
                        break;
                    }
                }

                if (lastIn < range.second)
                {
                    range.second = lastIn + 1;
                }
                else if (range.second == lastIn) // All points are out
                {
                    // Check whether this range can be removed

                    // In the following case, the corresponding range(denoted by "s" and "e")
                    // should be removed
                    //           ^
                    // e---<---s |
                    //           +-------<--------------------
                    //
                    // Ditto to the tail edge, we only need to check the end index of reference

                    //if (/*numRefPoint - 1 == *startRefIdxIt &&*/numRefPoint - 1 == *endRefIdxIt)
                    if (normXZ(subtract(reference.back(), reference[*endRefIdxIt])) < DISTINCT_DIST_MIN)
                    {
                        itRange = ranges.erase(itRange);

                        continue;
                    }
                }
                else
                {
                }
            }

            ++itRange;
        }
    }

    template <class Point3_T>
    bool searchEndPointIdxByRef(const std::vector<Point3_T> &vDensePoint,
                                const std::vector<std::vector<Point3_T>> &vReference,
                                const std::vector<std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXY>>> &vpKdtree,
                                std::vector<std::vector<std::pair<int, int>>> &vvEndIdx) const
    {
        auto findClosestIndex = [](
        const std::vector<Point3_T> &points,
        const Point3_T              &srcPt,
        const int                   &startIndex,
        const int                   &endIndex)
        {
            int index = startIndex;
            float minDistance = std::numeric_limits<float>::max();

            for (int i = startIndex; i <= endIndex; i++)
            {
                auto distance = (srcPt.x - points[i].x) * (srcPt.x - points[i].x) + (srcPt.z - points[i].z) * (srcPt.z - points[i].z);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    index = i;
                }
            }
            return index; 
        };

        const auto numReference = vReference.size();

        if (vDensePoint.empty() || 0 == numReference || numReference != vpKdtree.size())
        {
            SDOR_LOG_WARN << "The input data are invalid.";
            SDOR_LOG_WARN << "vDensePoint.size = " << vDensePoint.size() << "\n"
                           << "vReference.size = " << numReference << "\n"
                           << "vpKdtree.size() = " << vpKdtree.size();
            return false;
        }
        else
        {
            float angleThress = cos(M_PI / 5);
            //vvEndIdx: outer vector correspond to every division, inner vector correspond to some segments of paints
            //the element pair in vEndIdx, first and second represent indexes of start point and end point respectively
            vvEndIdx.clear();
            vvEndIdx.resize(numReference);

            std::vector<int32_t> vPaintIdx, vRefIdx;
            std::vector<float32_t> vNearSqrDis;

            const auto HUGE_VALUE = std::numeric_limits<float32_t>::max();

            Point3_T prevHeadEdge{};  // previous head edge vector
            Point3_T prevHeadPoint{HUGE_VALUE, HUGE_VALUE, HUGE_VALUE}; // last point of previous reference

            const auto DIST_GAP_MAX = 1.0F;

            // for every division
            for (auto i = 0U; i < numReference; ++i)
            {
                const auto &reference = vReference[i];
                auto &vEndIdx = vvEndIdx[i];

                // Calculate tail and head edge vectors
                Point3_T tailEdge{}, headEdge{};
                if (!getEdges(reference, tailEdge, headEdge))
                {
                    SDOR_LOG_WARN << "Failed to calculate edge vectors for reference " << i;

                    continue;
                }

                // Tweak:
                // If a line is apart from the references, even if the references are closely
                // connected as a contiguous path, there will exist gaps between successive
                // segments.
                // This is caused by the intrinsic of the way that the line is segmented(ends
                // of line are perpendicular to the reference).
                // In order to avoid the gaps, we need to do some tweaks here.

                // Judge whether the previous head edge vector should be used as tail edge vector of
                // the current reference (to avoid gap between successive segments)
                const auto nodeGap = normXZ(subtract(reference.front(), prevHeadPoint));
                if (nodeGap < DIST_GAP_MAX)
                {
                    // If the current reference is close enough to the previous one,
                    // we think they are contiguous, and so should the paint segments be.
                    // To achieve this, we use the head edge vector of the previous
                    // reference as the tail edge vector of the current one
                    tailEdge = prevHeadEdge;
                }

                // Update previous head edge vector and corresponding head point
                prevHeadEdge  = headEdge;
                prevHeadPoint = reference.back();

                const auto succeeded = searchPointByDist(vDensePoint,
                                                         vpKdtree[i],
                                                         reference,
                                                         vPaintIdx,
                                                         vRefIdx,
                                                         vNearSqrDis);
                if (!succeeded)
                {
                    // No log for this failure, for it's normal for
                    // unrelated references
                    continue;
                }

                // NOTICE:
                // When searchPointByDist succeeds, the size of vPaintIdx and that of vRefIdx equal
                // and both are greater than 0

                const auto numRefPoint = static_cast<int32_t>(reference.size());

                auto itPaintIdx1 = vPaintIdx.cbegin(), itPaintIdx2 = std::next(itPaintIdx1);

                auto startPaintIdx = vPaintIdx.front(); // start index of paint of current segment
                auto endPaintIdx   = startPaintIdx;     // end index of paint of current segment
                auto startRefIdx   = vRefIdx.front();   // start index of corresponding reference point of current segment
                auto endRefIdx     = startRefIdx;       // end index of corresponding reference point of current segment

                while (itPaintIdx1 < vPaintIdx.cend() && itPaintIdx2 < vPaintIdx.cend())
                {
                    if (*itPaintIdx2 - *itPaintIdx1 > 1) // current segment finishes
                    {
                        // Update end index of paint and reference
                        endPaintIdx = *itPaintIdx1;
                        endRefIdx   = *std::next(vRefIdx.cbegin(), std::distance(vPaintIdx.cbegin(), itPaintIdx1));

                        // Calculate the low and high indices of matched reference
                        auto refIdxLow  = std::min(startRefIdx, endRefIdx);
                        auto refIdxHigh = std::max(startRefIdx, endRefIdx);

                        // If the low and high indices are equal, the direction can not be computed
                        // so we extend both low and high indices by 1 to both sides
                        if (refIdxLow == refIdxHigh)
                        {
                            refIdxLow  = std::max(refIdxLow  - 1, 0);
                            refIdxHigh = std::min(refIdxHigh + 1, numRefPoint - 1);
                        }

                        auto closestStartPaintIdx = findClosestIndex(vDensePoint,reference[refIdxLow],startPaintIdx,endPaintIdx);
                        auto closestEndPaintIdx = findClosestIndex(vDensePoint,reference[refIdxHigh],startPaintIdx,endPaintIdx);

                        const auto sameDirection = checkDirection(vDensePoint[closestStartPaintIdx],
                                                                  vDensePoint[closestEndPaintIdx],
                                                                  reference[refIdxLow],
                                                                  reference[refIdxHigh],
                                                                  angleThress);
                        if (sameDirection)
                        {
                            // Keep current segment only when it shares the same direction with reference
                            vEndIdx.emplace_back(startPaintIdx, endPaintIdx + 1); // end index is excluded
                        }

                        // Start a new segment
                        startPaintIdx = *itPaintIdx2;
                        startRefIdx   = *std::next(vRefIdx.cbegin(), std::distance(vPaintIdx.cbegin(), itPaintIdx2));
                    }
                    else // current segment continues
                    {
                        // Update (accumulate) end index of paint and reference
                        endPaintIdx = *itPaintIdx2;
                        endRefIdx   = *std::next(vRefIdx.cbegin(), std::distance(vPaintIdx.cbegin(), itPaintIdx2));
                    }

                    ++itPaintIdx1;
                    ++itPaintIdx2;
                }

                // Handle the last segment if any
                if (endPaintIdx > startPaintIdx)
                {
                    // Calculate start and end indices of matched reference
                    auto refIdxLow  = std::min(startRefIdx, endRefIdx);
                    auto refIdxHigh = std::max(startRefIdx, endRefIdx);

                    if (refIdxLow == refIdxHigh)
                    {
                        refIdxLow  = std::max(refIdxLow  - 1, 0);
                        refIdxHigh = std::min(refIdxHigh + 1, numRefPoint - 1);
                    }

                    auto closestStartPaintIdx = findClosestIndex(vDensePoint,reference[refIdxLow],startPaintIdx,endPaintIdx);
                    auto closestEndPaintIdx = findClosestIndex(vDensePoint,reference[refIdxHigh],startPaintIdx,endPaintIdx);

                    const auto sameDirection = checkDirection(vDensePoint[closestStartPaintIdx],
                                                              vDensePoint[closestEndPaintIdx],
                                                              reference[refIdxLow],
                                                              reference[refIdxHigh],
                                                              angleThress);
                    if (sameDirection)
                    {
                        vEndIdx.emplace_back(startPaintIdx, endPaintIdx + 1);
                    }
                }

                // Remove redundant points that are not in perpendicular ranges
                removeRedundantRanges(vEndIdx,
                                      vDensePoint,
                                      reference,
                                      vPaintIdx,
                                      vRefIdx,
                                      tailEdge,
                                      headEdge);

                removeInvalidSegment(vDensePoint, vNearSqrDis, vEndIdx);
            }  //end for i
        }

        // A tweak: In order to remove the one-point gap between the two successive
        // divisions, alter the start point of the latter one to the end point of
        // the former one.

        // NOTICE: Note that the second index of the pair is excluded

        auto canTweakSegments([](const std::pair<int32_t, int32_t> &indexPairPrev,
                                 const std::pair<int32_t, int32_t> &indexPairCurr)
                              {
                                  return (indexPairPrev.second <= indexPairCurr.first &&
                                          indexPairPrev.second + 1 >= indexPairCurr.first);
                              });

        const auto numSegIdxPair = static_cast<int32_t>(vvEndIdx.size());
        for (auto i = 1; i < numSegIdxPair; ++i)
        {
            auto &segPairsCurr       = vvEndIdx[i];
            const auto &segPairsPrev = vvEndIdx[i - 1];

            for (const auto &indexPairPrev : segPairsPrev)
            {
                for (auto &indexPairCurr : segPairsCurr)
                {
                    if (canTweakSegments(indexPairPrev, indexPairCurr))
                    {
                        // Alter current segment index pair, to make it share
                        // the same point with the previous one

                        indexPairCurr.first = std::max(indexPairPrev.second - 1, 0);
                    }
                }
            }
        }

        // Decide whether any segment is segmented
        const auto segmented =
                std::any_of(vvEndIdx.cbegin(),
                            vvEndIdx.cend(),
                            [](const std::vector<std::pair<int32_t, int32_t>> &segmentIndices)
                            {
                                return !segmentIndices.empty();
                            });

        return segmented;
    }


      /**
     * This function is to slice the NURBS curve into some segments.
     *
     * @param nurbsParam        [ IN] parameters of input NURBS curve
     * @param vNode             [ IN] the points in which neighborhood is the break point.
     * @param vSecParam         [OUT] parameters of output NRUBS curve
     * @param vSectionPoints    [OUT] point cloud of the output NURBS
     * @param vSecIndex         [OUT] index of these sections that successful slice the NURBS curve
     * @param vSegType          [OUT] the slice type of the sections
     *
     * @return true on success, false otherwise
     */
    template <class Point3_T>
    bool segmentByRef(const roadDBCore::NURBS_t                        &nurbsParam,
                      const std::vector<std::vector<Point3_T>>         &vReference,
                      const std::vector<std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXY>>> &vpKdtree,
                      std::vector<std::vector<roadDBCore::NURBS_t>>    &vvSecParam,
                      std::vector<std::vector<std::vector<Point3_T>>>  &vvSecPoints,
                      std::vector<std::vector<TRUNCATE_TYPE_E>>        &vvSegType) const
    {
        bool bRet = true;
        if (nurbsParam.vecCtrlPoint.empty() || nurbsParam.vecKnot.empty() ||
            nurbsParam.endPoint.empty() || (nurbsParam.endPoint.size() & 0x1) != 0 ||
            vReference.empty() || vReference.size() != vpKdtree.size())
        {
            SDOR_LOG_WARN << "The input data are invalid.";
            SDOR_LOG_WARN << "number of control points is  = " << nurbsParam.vecCtrlPoint.size();
            SDOR_LOG_WARN << "knot.size() = " << nurbsParam.vecKnot.size();
            SDOR_LOG_WARN << "endPoint.size() = " << nurbsParam.endPoint.size();
            SDOR_LOG_WARN << "vReference.size() = " << vReference.size();
            SDOR_LOG_WARN << "vpKdtree.size() = " << vpKdtree.size();
            bRet = false;
        }
        else
        {
            float32_t paramLength(0);
            auto it = nurbsParam.endPoint.begin();
            while (it < nurbsParam.endPoint.end())
            {
                paramLength -= *(it++);
                paramLength += *(it++);
            }

            if (paramLength < FLT_EPSILON)
            {
                SDOR_LOG_WARN << "The length of the paint is zero.";
                bRet = false;
            }

            std::vector<float32_t> U;
            std::vector<Point3_T> vDensePoint;
            std::vector<size_t> vRange;

            bRet = bRet && generateCurveAndRegion(nurbsParam, 0.2f, vDensePoint, vRange, U);

            std::vector<std::vector<std::pair<int, int>>> vvEndIdx;
            bRet = bRet && searchEndPointIdxByRef(vDensePoint, vReference, vpKdtree, vvEndIdx);

            size_t CPIndex1, CPIndex2;
            auto itCP = nurbsParam.vecCtrlPoint.begin();
            auto itKnot = nurbsParam.vecKnot.begin();

            vvSecParam.clear();
            vvSecPoints.clear();
            vvSegType.clear();
            vvSecParam.resize(vvEndIdx.size());
            vvSecPoints.resize(vvEndIdx.size());
            vvSegType.resize(vvEndIdx.size());

            for (size_t iSec = 0; iSec < vvEndIdx.size(); ++iSec)
            {
                auto &vEndIdx = vvEndIdx[iSec];
                auto &vSecParam = vvSecParam[iSec];
                auto &vSecPoints = vvSecPoints[iSec];
                auto &vSegType = vvSegType[iSec];

                vSecParam.resize(vEndIdx.size());
                vSecPoints.resize(vEndIdx.size());
                vSegType.resize(vEndIdx.size());
                for (size_t iSeg = 0; iSeg < vEndIdx.size(); ++iSeg)
                {
                    std::pair<int, int> &pairEnd = vEndIdx[iSeg];
                    vSecPoints[iSeg].assign(vDensePoint.begin() + pairEnd.first, vDensePoint.begin() + pairEnd.second);

                    CPIndex1 = vRange[pairEnd.first] - 2;
                    CPIndex2 = vRange[pairEnd.second - 1];

                    auto pSecParam = &vSecParam[iSeg];
                    pSecParam->vecCtrlPoint.assign(itCP + CPIndex1, itCP + CPIndex2 + 1);
                    pSecParam->vecKnot.assign(itKnot + CPIndex1, itKnot + CPIndex2 + 4);

                    //search the first segment
                    auto it = nurbsParam.endPoint.begin(); // it point the first end of the first segment
                    while (U[pairEnd.first] - (*it) > -FLT_MIN && it < nurbsParam.endPoint.end())
                    {
                        it += 2; // it point to the first end of the next segment
                    }
                    --it;  // it point to the second end of the previous segment
                    if (U[pairEnd.first] - (*it) < FLT_MIN)
                    {
                        if ((*it) - U[pairEnd.second - 1] < -FLT_MIN)
                        {
                            //in this condition it point to the second end of the next segment
                            pSecParam->endPoint.emplace_back(U[pairEnd.first]);
                            pSecParam->endPoint.emplace_back(*it++);
                            pSecParam->endPoint.emplace_back(*it++);
                        }
                        else
                        {
                            pSecParam->endPoint.emplace_back(U[pairEnd.first]);
                        }
                    }
                    else
                    {
                        //in this condition it point to the second end of the next segment
                        //add a new segment with length that equal to zero
                        pSecParam->endPoint.emplace_back(U[pairEnd.first]);
                        pSecParam->endPoint.emplace_back(U[pairEnd.first]);
                        ++it;
                        pSecParam->endPoint.emplace_back(*it++);
                    }

                    //search the last segment
                    while (U[pairEnd.second - 1] - (*it) > FLT_MIN && it < nurbsParam.endPoint.end())
                    {
                        //in this condition it point to the second end of the next segment
                        pSecParam->endPoint.emplace_back(*it++);
                        pSecParam->endPoint.emplace_back(*it++);
                    }
                    --it; // it point to the first end of the current segment
                    if ((*it) < U[pairEnd.second - 1])
                    {
                        //the segment point is inside of the segment line
                        pSecParam->endPoint.emplace_back(U[pairEnd.second - 1]);
                    }
                    else
                    {
                        //the segment point is outside of the segment line
                        pSecParam->endPoint.pop_back();
                        //add a new segment with length that equal to zero
                        pSecParam->endPoint.emplace_back(U[pairEnd.second - 1]);
                        pSecParam->endPoint.emplace_back(U[pairEnd.second - 1]);
                    }

                    //compute the length of segment in parameter space
                    float32_t coeff(0);
                    auto itSecParam = pSecParam->endPoint.begin();
                    while (itSecParam < pSecParam->endPoint.end())
                    {
                        coeff -= *(itSecParam++);
                        coeff += *(itSecParam++);
                    }

                    //compute paint length of segment.
                    pSecParam->paintTotalLength = nurbsParam.paintTotalLength / paramLength * coeff;

                    //compute the length of line in parameter space
                    coeff = nurbsParam.endPoint.back() - nurbsParam.endPoint.front();
                    if (coeff < FLT_EPSILON)  // if coeff <= 0
                    {
                        pSecParam->lineLength = 0.f;
                    }
                    else
                    {
                        //length of segment = length of line * length of segment in parameter space / length of line in parameter space
                        pSecParam->lineLength = nurbsParam.lineLength * (pSecParam->endPoint.back() - pSecParam->endPoint.front()) / coeff;
                    }

                    int numPoints = static_cast<int>(vDensePoint.size());
                    if (pairEnd.first == 0 && pairEnd.second != numPoints)
                    {
                        vSegType[iSeg] = TRUNCATE_TYPE_SECOND_E;
                    }
                    else if (pairEnd.first != 0 && pairEnd.second == numPoints)
                    {
                        vSegType[iSeg] = TRUNCATE_TYPE_FIRST_E;
                    }
                    else if (pairEnd.first == 0 && pairEnd.second == numPoints)
                    {
                        vSegType[iSeg] = TRUNCATE_TYPE_NO_E;
                    }
                    else
                    {
                        vSegType[iSeg] = TRUNCATE_TYPE_BOTH_E;
                    }
                } //end for iSeg
            } //end for iSec
        }
        return bRet;
    }

private:
    template <class Point3_T>
    inline Point3_T add(const Point3_T& a, const Point3_T& b) const
    {
       Point3_T c;
       c.x = a.x + b.x;
       c.y = a.y + b.y;
       c.z = a.z + b.z;
       return c;
    }

    template <class Point3_T>
    inline float crossInXOZ(const Point3_T& a, const Point3_T& b) const
    {
       return a.x * b.z - a.z * b.x;
    }

    template <class Point3_T>
    inline Point3_T subtract(const Point3_T& a, const Point3_T& b) const
    {
       Point3_T c;
       c.x = a.x - b.x;
       c.y = a.y - b.y;
       c.z = a.z - b.z;
       return c;
    }

    template <class Point3_T>
    inline float scalarMultiply(const Point3_T& a, const Point3_T& b) const
    {
       float multiple(a.x * b.x);
       multiple += a.y * b.y;
       multiple += a.z * b.z;
       return multiple;
    }

    template <class Point3_T>
    inline float scalarMultiplyInXOZ(const Point3_T& a, const Point3_T& b) const
    {
       float multiple(a.x * b.x);
       multiple += a.z * b.z;
       return multiple;
    }

    template <class Point3_T>
    inline Point3_T numericalMultiply(const float& a, const Point3_T& point) const
    {
       Point3_T c;
       c.x = point.x * a;
       c.y = point.y * a;
       c.z = point.z * a;
       return c;
    }

    template <class Point3_T>
    inline float norm(const Point3_T& a) const
    {
       float tmp(a.x * a.x);
       tmp += a.y * a.y;
       tmp += a.z * a.z;
       return sqrt(tmp);
    }

    template <class Point3_T>
    inline float normXZ(const Point3_T& a) const
    {
        return std::sqrt(a.x * a.x + a.z * a.z);
    }

    template <class Point3_T>
    inline float normXYZ(const Point3_T& a) const
    {
        return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    }

    template <class Point3_T>

    inline void reset(Point3_T& a) const
    {
       a.x = 0.f;
       a.y = 0.f;
       a.z = 0.f;
    }

    template <typename T1, typename T2>
    bool isElement(const T1 &element, const std::vector<T2> &container) const
    {
        T2 elem = static_cast<T2>(element);
        for (auto &i : container)
        {
            if (elem == i)
            {
                return true;
            }
        }
        return false;
    }

    bool isManMadeNURBS(const roadDBCore::NURBS_t NURBSParam) const
    {
        auto c0 = NURBSParam.vecCtrlPoint.size() == 3U;
        auto c1 = NURBSParam.vecKnot.size() == 6U;
        auto c2 = NURBSParam.endPoint.size() == 2U;
        auto c3 = std::fabs(NURBSParam.lineLength - NURBSParam.paintTotalLength) < FLT_EPSILON;

        if (c0 && c1 && c2 && c3)
        {
            auto x = 0.5 * (NURBSParam.vecCtrlPoint[0].relLon + NURBSParam.vecCtrlPoint[2].relLon);
            auto y = 0.5 * (NURBSParam.vecCtrlPoint[0].relAlt + NURBSParam.vecCtrlPoint[2].relAlt);
            auto z = 0.5 * (NURBSParam.vecCtrlPoint[0].relLat + NURBSParam.vecCtrlPoint[2].relLat);

            auto c4 = std::fabs(x - NURBSParam.vecCtrlPoint[1].relLon) < FLT_EPSILON;
            auto c5 = std::fabs(y - NURBSParam.vecCtrlPoint[1].relLat) < FLT_EPSILON;
            auto c6 = std::fabs(z - NURBSParam.vecCtrlPoint[1].relAlt) < FLT_EPSILON;

            auto c7 = (NURBSParam.vecKnot[0] - 0.0) < FLT_EPSILON;
            auto c8 = (NURBSParam.vecKnot[1] - 0.0) < FLT_EPSILON;
            auto c9 = (NURBSParam.vecKnot[2] - 0.0) < FLT_EPSILON;
            auto ca = (NURBSParam.vecKnot[3] - 1.0) < FLT_EPSILON;
            auto cb = (NURBSParam.vecKnot[4] - 1.0) < FLT_EPSILON;
            auto cc = (NURBSParam.vecKnot[5] - 1.0) < FLT_EPSILON;

            auto cd = std::fabs(NURBSParam.endPoint[0] - 0.0) < FLT_EPSILON;
            auto ce = std::fabs(NURBSParam.endPoint[1] - 1.0) < FLT_EPSILON;

            if (c4 && c5 && c6 && c7 && c8 && c9 && ca && cb && cc && cd && ce)
            {
                return true;
            }
        }

        return false;
    }
};

}

#endif /* NURBS_H_ */
