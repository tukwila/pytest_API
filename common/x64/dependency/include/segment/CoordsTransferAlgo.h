/*
 *******************************************************************************
 *                       RoadDB Confidential
 *       Copyright (c) Continental AG. 2016-2018, RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   CoordsTransferAlgo.h
 * @brief  declaration of gps coordinates transfer functions
 *******************************************************************************
 */

#ifndef _COORDS_TRANSFER_ALGO_H
#define _COORDS_TRANSFER_ALGO_H

#include <Eigen/Dense>
#include "typeDef.h"
#include "segment/Segment.h"
#include "CommunicateDef/RdbV2SGeometry.h"


namespace roadDBCore
{

/**
 *******************************************************************************
 * @class BaseTransfer  "segment/CoordsTransferAlgo.h"
 *
 * @brief define some common function for coordinates transfer
 *******************************************************************************
 */
class BaseTransfer
{
protected:

    // degree <--> radian
    double deg2Rad(double degree) const;
    double rad2Deg(double radian) const;

    // geodetic <--> ecef
    void geodetic2Ecef(const double lon, const double lat, const double alt,
                       double &ecefX, double &ecefY, double &ecefZ) const;
    void ecef2Geodetic(const double ecefX, const double ecefY, const double ecefZ,
                       double &lon, double &lat, double &alt) const;

    // the transform matrix of enu <--> ecef
    void getTransMatrixOfEnu2Ecef(const double lon, const double lat,
                std::array<std::array<double, 3>, 3> &transMatrix) const;

    void getTransMatrixOfEcef2Enu(const double lon, const double lat,
                std::array<std::array<double, 3>, 3> &transMatrix) const;
};

/**
 *******************************************************************************
 * @class GpsCoordsTransfer  "segment/CoordsTransferAlgo.h"
 *
 * @brief transfer coordinates between absolute and relative coordinates
 *******************************************************************************
 */
class GpsCoordsTransfer : public BaseTransfer
{
public:
    template <typename T>
    GpsCoordsTransfer(const roadDBCore::Point3_t<T>& refPoint):
    refLon_(refPoint.relLon),
    refLat_(refPoint.relLat),
    refAlt_(refPoint.relAlt),
    refEcefX_(0), refEcefY_(0), refEcefZ_(0)
    {
    };

    void initRadius();

    template <typename U, typename T>
    void abs2Rel(const roadDBCore::Point3_t<U>& absPoint, roadDBCore::Point3_t<T>& relPoint) const
    {
        double ecefX, ecefY, ecefZ;
        double dEast, dNorth, dUp;

        geodetic2Ecef(absPoint.relLon, absPoint.relLat, absPoint.relAlt, ecefX, ecefY, ecefZ);
        ecef2Enu(ecefX, ecefY, ecefZ, dEast, dNorth, dUp);

        relPoint.relLon = dEast;
        relPoint.relLat = dNorth;
        relPoint.relAlt = dUp;
    }

    template <typename T>
    void rel2Abs(const roadDBCore::Point3_t<T>& relPoint, roadDBCore::Point3d_t& absPoint) const
    {
        double ecefX, ecefY, ecefZ;

        enu2Ecef(relPoint.relLon, relPoint.relLat, relPoint.relAlt, ecefX, ecefY, ecefZ);
        ecef2Geodetic(ecefX, ecefY, ecefZ, absPoint.relLon, absPoint.relLat, absPoint.relAlt);
    }

    template <typename T>
    void abs2RelVec(const std::vector<roadDBCore::Point3d_t> &absPoints,
                    std::vector<roadDBCore::Point3_t<T>> &outRelPoints)
    {
        outRelPoints.clear();
        roadDBCore::Point3_t<T> outPoint;

        for (auto &absPoint : absPoints)
        {
            abs2Rel(absPoint, outPoint);
            outRelPoints.push_back(outPoint);
        }
    }

    template <typename P>
    void rel2AbsVec(const std::vector<roadDBCore::Point3_t<P>> &relPoints,
                    std::vector<roadDBCore::Point3d_t> &outAbsPoints)
    {
        outAbsPoints.clear();
        roadDBCore::Point3d_t outPoint;

        for (auto &relPoint : relPoints)
        {
            rel2Abs(relPoint, outPoint);
            outAbsPoints.push_back(outPoint);
        }
    }

private:
    void ecef2Enu(const double &ecefX, const double &ecefY, const double &ecefZ,
                  double &dEast, double &dNorth, double &dUp) const;
    void enu2Ecef(const double &dEast, const double &dNorth, const double &dUp,
                  double &ecefX, double &ecefY, double &ecefZ) const;

private:
    /// geographic coordinate of the reference point (lon, lat, alt)
    double refLon_;
    double refLat_;
    double refAlt_;

    double refEcefX_;
    double refEcefY_;
    double refEcefZ_;

    std::array<std::array<double, 3>, 3> transMatrix_; // transMatrix of Ecef2Enu
};

/**
 *******************************************************************************
 * @class GpsOffsetTransfer  "segment/CoordsTransferAlgo.h"
 *
 * @brief transfer offset from one relative point to another
 *******************************************************************************
 */
class GpsOffsetTransfer : public BaseTransfer
{
public:
    GpsOffsetTransfer();

    bool initRadius(const roadDBCore::SegmentID_t dstSeg, const roadDBCore::SegmentID_t srcSeg, int32_t level = 14);

    bool initRadius(const roadDBCore::Point3d_t &dstTileAnchorPt, const roadDBCore::Point3d_t &srcTileAnchorPt,
                    int32_t level = 14, const bool bDstAnchorPt = false, const bool bSrcAnchorPt = false);

    template <typename T>
    void transOffset2Ref(roadDBCore::Point3_t<T>& curPoint) const
    {
        if (true == bSkip_)
        {
            return;
        }

        double dx = curPoint.relLon, dy = curPoint.relLat, dz = curPoint.relAlt;

        curPoint.relLon = transMatrix_[0][0] * dx + transMatrix_[0][1] * dy
                        + transMatrix_[0][2] * dz + transMatrix_[0][3];

        curPoint.relLat = transMatrix_[1][0] * dx + transMatrix_[1][1] * dy
                        + transMatrix_[1][2] * dz + transMatrix_[1][3];

        curPoint.relAlt = transMatrix_[2][0] * dx + transMatrix_[2][1] * dy
                        + transMatrix_[2][2] * dz + transMatrix_[2][3];
    }

    void transOffset2Ref(float64_t rotation[9], float64_t translation[3]) const;
    void transOffset2Ref(float32_t quaternion[4], float32_t translation[3]) const;
    void transOffset2Ref(Eigen::Matrix3d &rotation, Eigen::Vector3d &translation) const;

    bool isSkip() const { return bSkip_;};

private:
    void preCompute(const roadDBCore::Point3d_t &dstTileAnchorPt,
                    const roadDBCore::Point3d_t &srcTileAnchorPt);

private:
    bool   bSkip_;  //when two segmentids are equal, just skip the calculation
    std::array<std::array<double, 4>, 4> transMatrix_;
    std::array<std::array<double, 3>, 3> rTransMatrix_;
};

}

#endif
