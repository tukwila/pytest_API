/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   NDACache.h
 * @brief  This is for the Sqlite database based NDACache.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-07-01        Tony Xiong        Init version.
 *
 *******************************************************************************
 */
#include <list>
#include <memory>

#include "segment/Segment.h"
#include "Geo.h"
using namespace std;

// namespace geo = roaddb::geo;
namespace core = roadDBCore;

namespace RDBVehicleAPI
{
class SegmentSquare
{
    //  friend class NDACache;

 public:
    //  static std::shared_ptr<SegmentSquare> createSegmentSquare(uint32_t radius,
    //                                                            const WGS84_t& location);
    //  static std::shared_ptr<SegmentSquare> createSegmentSquare(const WGS84_t& location,
    //                                                            uint8_t numOfRings = 0);

    // Determine how many rings are needed to cover the circle given by radius and center
    // location. Return -1 if any error occurs.
    static int32_t determineNumOfRings(uint32_t radius, const WGS84_t& center);

    //  uint32_t getNumOfRings()
    //  {
    //      return numOfRings_;
    //  };

 private:
    SegmentSquare(const WGS84_t& location);
    //  SegmentSquare(const WGS84_t& location, uint8_t numOfRings);

    // const WGS84_t& getCenterLocation()
    // {
    //     return centerLocation_;
    // };

    // core::SegmentID_t getCenterSegID()
    // {
    //     return centerSegID_;
    // };

    // core::SegmentID_t getLeftButtomSegID()
    // {
    //     return centerSegID_;
    // };

    uint32_t isCircleCovered(const WGS84_t& location, uint16_t radius);

    bool getBoundaries(std::array<double, 4>& boundaries);

    //  bool expandCircle();
    bool partialExpandCircle();

 private:
    WGS84_t centerLocation_{0, 0, 0};
    core::Point3d_t coreCenterLocation_{0, 0, 0};
    uint32_t numOfRings_ = 0;
    core::Segment segment_;

    core::SegmentID_t centerSegID_ = 0;
    core::SegmentID_t leftButtomSegID_ = 0;
    core::SegmentID_t rightButtomSegID_ = 0;
    core::SegmentID_t leftTopSegID_ = 0;
    core::SegmentID_t rightTopSegID_ = 0;
    std::list<core::SegmentID_t> topSegIDs_;
    std::list<core::SegmentID_t> buttomSegIDs_;
    std::list<core::SegmentID_t> leftSegIDs_;
    std::list<core::SegmentID_t> rightSegIDs_;
};
} /* namespace dapi */

