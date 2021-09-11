/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Segment.h
 * @brief  Tile class, wap segment related functions
 *******************************************************************************
 */

#ifndef RDB_TILE_H_
#define RDB_TILE_H_

#include <array> //std::array
#include <set>   //std::set

#include "typeDef.h"
#include "segment/gpsConvert.h"
#include "LogWrapper/LogWrapper.h"
#include "CommunicateDef/RdbV2SGeometry.h"


namespace roadDBCore
{
const int32_t MAX_TILE_LEVEL = 15;
const int32_t RDB_TILE_LEVEL = 14;
const int32_t INVALID_TILE_ID = -1;

const double PI = 3.141592653589793;                    // PI
const double DEGREE_TO_RADIAN = 0.0174532925199433;     // PI/180
const double RADIAN_TO_DEGREE = 57.29577951308233;      // 1/DEGREE_TO_RADIAN
const double EARTH_RADIUS = 6378137.0;                  // semi-major axis of earth
const double EARTH_FLATTENING = 0.0033528131778969;     // flattening of the earth, 1/298.257
const double EARTH_FLATTENING2 = 0.9983235934110516;    // 1 - 0.5 * EARTH_FLATTENING
const double ECCENTRICITY = EARTH_FLATTENING * (2 - EARTH_FLATTENING); // first eccentricity ratio

enum SEG_DIRECTION_E
{
    SEG_DIRECTION_NO_E = 0,
    SEG_DIRECTION_LEFT_E = 1,
    SEG_DIRECTION_RIGHT_E = 2,
    SEG_DIRECTION_UP_E = 4,
    SEG_DIRECTION_UPLEFT_E = 5,
    SEG_DIRECTION_UPRIGHT_E = 6,
    SEG_DIRECTION_DOWN_E = 8,
    SEG_DIRECTION_DOWNLEFT_E = 9,
    SEG_DIRECTION_DOWNRIGHT_E = 10,
    SEG_DIRECTION_MAX_E
};

class Segment
{
public:
    Segment();
    Segment(int32_t level);

    /**
     *******************************************************************************
     * @brief gps2Tile - convert gps position to tile coordinate, based on tile anchor point.
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - Point3d_t &gps, unit: degree
     *
     *  @param [In]  - int32_t &tileID
     *
     *  @param [Out]  - Point3f_t &offset, unit: meter
     *
     *  @return - if success, return true, else return false
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    template<typename T>
    bool gps2Seg(const Point3d_t &gps, SegmentID_t &tileID, Point3_t<T> &offset)
    {
        if (!isGpsValid(gps))
        {
            COM_LOG_ERROR << "Invalid gps.";
            return false;
        }

        Point3d_t anchorGps;

        if (!getTileIdAndAnchorGps(gps, tileID, anchorGps))
        {
            COM_LOG_ERROR << "Failed to get segment ID.";
            return false;
        }

        calcRelativeLocation(anchorGps, gps, offset);

        return true;
    }

    bool gps2Seg(const Point3d_t &gps, SegmentID_t &segID);

    template<typename T>
    bool seg2Gps(SegmentID_t tileID, const Point3_t<T> &offset, Point3d_t &gps)
    {
        if (tileID == INVALID_TILE_ID)
        {
            COM_LOG_ERROR << "Invalid segment id.";
            return false;
        }

        Point3d_t anchorGps;

        if (!getSegAnchorGps(tileID, anchorGps))
        {
            COM_LOG_ERROR << "Failed to get Anchor GPS.";
            return false;
        }

        calcGpsFromRelLocation(anchorGps, offset, gps);

        return true;
    }

    template<typename T>
    bool seg2Gps(SegmentID_t tileID, const std::vector<Point3_t<T> > &vOffset, std::vector<Point3d_t> &vGps)
    {
        if (tileID == INVALID_TILE_ID)
        {
            COM_LOG_ERROR << "Invalid segment id.";
            return false;
        }

        Point3d_t anchorGps;

        if (!getSegAnchorGps(tileID, anchorGps))
        {
            COM_LOG_ERROR << "Failed to get Anchor GPS.";
            return false;
        }

        calcGpsFromRelLocationVec(anchorGps, vOffset, vGps);

        return true;
    }

    /**
     *******************************************************************************
     * @brief getTileIdOfGps - get tile id
     *
     *  <1> Parameter Description:
     *
     *  @param [In]  - Point3d_t &gps, unit: degree
     *
     *  @return - if success, return the tile id of gps, else failed, return -1;
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     *******************************************************************************
     */
    SegmentID_t getSegID(const Point3d_t &gps);
    SegmentID_t getSegID(float64_t lon, float64_t lat, float64_t alt);

    SegmentID_t getAroundSegID(const Point3d_t &gps);
    SegmentID_t getAroundSegID(float64_t lon, float64_t lat, float64_t alt);

    bool getSegAnchorGps(SegmentID_t tileID, Point3d_t &anchorGps);

    /********************************************************************************
     * @brief getSegBLAndTRGps - Get GPS of bottom left(anchor) and top right points
     *        of segment.
     *
     *  <1> Parameter Description:
     *
     *  @param segmentID      [IN] identity of target segment
     *  @param bottomLeftGps [OUT] GPS of bottom left point of segment
     *  @param topRightGps   [OUT] GPS of top right point of segment
     *
     *  @return true on success, false otherwise
     *
     *  <2> Detailed Description:
     *
     *  \ingroup
     ********************************************************************************/
    bool getSegBLAndTRGps(SegmentID_t segmentID, Point3d_t &bottomLeftGps, Point3d_t &topRightGps);

    bool getAroundSeg(SegmentID_t oriTileID, SEG_DIRECTION_E direct, SegmentID_t &aroundTileID);
    bool getAroundSeg(SegmentID_t oriTileID, std::vector<SegmentID_t> &vAroundSegIDs);

    /******************************************************************************************
     * @brief getSegList- Get square segment List of between two Gps Points or two segments
     *
     *  <1> Parameter Description:
     *
     *  @param  Point3d_t or SegmentID_t  [IN]   two Gps or segment
     *  @param  segList [OUT]   square segment List
     *
     *  @return true on success, false otherwise
     *
     *  NOTE: The distance between two positions should not exceed 50Km
     *  <2> Detailed Description:
     *
     *   Diagonally with two positions to get a rectangular segmentList
     *
     *****************************************************************************************/
    bool getSegList(const Point3d_t &srcGps, const Point3d_t &desGps,
                    std::vector<SegmentID_t> &segList);
    bool getSegList(SegmentID_t &src, SegmentID_t &des, std::vector<SegmentID_t> &segList);

    /******************************************************************************************
     * @brief getSegBoundary - Get boundaries of segment, in terms of longitudes and latitudes.
     *
     *  <1> Parameter Description:
     *
     *  @param segmentID   [IN] identity of target segment
     *  @param boundaries [OUT] array of boundaries of segment. The order is described below.
     *
     *  @return true on success, false otherwise
     *
     *  <2> Detailed Description:
     *
     *             top[0]
     *          ___________
     *          |          |
     *   left[3]|          |right[1]
     *          |__________|
     *            bottom[2]
     *  \ingroup
     *****************************************************************************************/
    bool getSegBoundaries(SegmentID_t segmentID, std::array<float64_t, 4> &boundaries);

    // template<typename RECT_TYPE> // RECT_TYPE can be cv::Rect_<T>
    // bool getAroundSegRect(SegmentID_t oriSegID,
    //                         std::vector<SegmentID_t> &vAroundSegIDs,
    //                         RECT_TYPE &innerRect,
    //                         RECT_TYPE &outerRect)
    // {
    //     if (oriSegID == INVALID_TILE_ID)
    //     {
    //         COM_LOG_ERROR << "Invalid original segment ID.";
    //         return false;
    //     }

    //     if (!getAroundSeg(oriSegID, vAroundSegIDs) || vAroundSegIDs.size() != 9)
    //     {
    //         COM_LOG_ERROR << "Get around segment failed.";
    //         return false;
    //     }

    //     Point3d_t sizeOfLeftDownSeg;
    //     Point3d_t sizeOfItself;
    //     Point3d_t sizeofUpRightSeg;

    //     /* Get size of left down segment, original segment and up right segment
    //        to calculate outer rectangle size*/
    //     if (!getSegSize(vAroundSegIDs[6], sizeOfLeftDownSeg) ||
    //         !getSegSize(oriSegID, sizeOfItself) ||
    //         !getSegSize(vAroundSegIDs[2], sizeofUpRightSeg))
    //     {
    //         COM_LOG_ERROR << "Get segment size failed.";
    //         return false;
    //     }

    //     innerRect = RECT_TYPE(0.0, 0.0, sizeOfItself.relLon, sizeOfItself.relLat);

    //     /*Calculate origin of outer rectangle relate to original segment*/
    //     Point3d_t outerRectOrigin;

    //     if (!offsetBetweenTwoSeg(oriSegID, vAroundSegIDs[6], outerRectOrigin))
    //     {
    //         COM_LOG_ERROR << "Calculate offset between segment "
    //                       << oriSegID
    //                       << " and "
    //                       << vAroundSegIDs[6]
    //                       << " failed.";
    //         return false;
    //     }

    //     outerRect = RECT_TYPE(outerRectOrigin.relLon,
    //                           outerRectOrigin.relLat,
    //                           sizeOfLeftDownSeg.relLon+ sizeOfItself.relLon + sizeofUpRightSeg.relLon,
    //                           sizeOfLeftDownSeg.relLat + sizeOfItself.relLat + sizeofUpRightSeg.relLat);

    //     return true;
    // }

    void getAffectSegID(const Point3d_t &gps, float32_t margin, std::set<SegmentID_t> &setSegments);
    void getAffectSegID(const std::vector<Point3d_t> &gpsList, float32_t margin, std::set<SegmentID_t> &setSegments);
    void getAffectSegID(const Point3d_t &gps, std::set<SegmentID_t> &setSegments);
    void getAffectSegID(const std::vector<Point3d_t> &gpsList, std::set<SegmentID_t> &setSegments);
    void getPassAndAffectSegID(const std::vector<Point3d_t> &gpsList, float32_t margin, std::set<SegmentID_t> &setSegments);
    void getPassAndAffectSegID(const std::vector<Point3d_t> &gpsList, std::set<SegmentID_t> &setSegments);


    template<typename T>
    bool offsetBetweenTwoSeg(SegmentID_t srcTileID, SegmentID_t destTileID, Point3_t<T> &offset)
    {
        //anchor point of source tile and dest tile
        Point3d_t srcTileAnchorPt;
        Point3d_t destTileAnchorPt;

        if ((srcTileID == INVALID_TILE_ID) || (destTileID == INVALID_TILE_ID))
        {
            COM_LOG_ERROR << "Invalid segment id.";
            return false;
        }

        if (!getSegAnchorGps(srcTileID, srcTileAnchorPt))
        {
            COM_LOG_ERROR << "Get segment " << srcTileID << " anchor point failed.";
            return false;
        }

        if (!getSegAnchorGps(destTileID, destTileAnchorPt))
        {
            COM_LOG_ERROR << "Get segment " << destTileID << " anchor point failed.";
            return false;
        }

        calcRelativeLocation(srcTileAnchorPt, destTileAnchorPt, offset);

        return true;
    }

    template<typename T>
    bool getSegSize(SegmentID_t segID, Point3_t<T> &size) //unit: meter.
    {
        Point3d_t bottomLeftGps;
        Point3d_t topRightGps;

        if (getSegBLAndTRGps(segID, bottomLeftGps, topRightGps))
        {
            calcRelativeLocation(bottomLeftGps, topRightGps, size);

            return true;

        }
        else
        {
            return false;
        }
    }

    template<typename T>
    bool getSegSize(const Point3d_t &gps, Point3_t<T> &size)  //unit: meter.
    {
        SegmentID_t segID = getSegID(gps);

        if (segID == INVALID_TILE_ID)
        {
            return false;
        }

        return getSegSize(segID, size);
    }

    template<typename T>
    bool gps2Relative(SegmentID_t refSegID, const std::vector<Point3d_t> &vGps, std::vector<Point3_t<T>> &vRelPos)
    {
        if ((refSegID == INVALID_TILE_ID) || vGps.empty())
        {
            COM_LOG_ERROR << "Invalid refSeg(" << refSegID<< ") or empty input GPS.";
            return false;
        }

        Point3d_t anchorGps;

        if (!getSegAnchorGps(refSegID, anchorGps))
        {
            COM_LOG_ERROR << "Get anchor GPS of reference segment failed.";
            return false;
        }

        calcRelativeLocationVec(anchorGps, vGps, vRelPos);

        return true;
    }

    bool combineTileID(int32_t lonNdsCoor, int32_t latNdsCoor, SegmentID_t &tileID);
    bool seperateTileID(SegmentID_t tileID, int32_t &lonNdsCoor, int32_t &latNdsCoor);

    int32_t getLevel();
    float64_t getSegSize();
    float64_t getNdsCoorUnit();
    int32_t getMaxLonCoorOfTileID();
    int32_t getMaxLatCoorOfTileID();
    int32_t getTopLatCoorOfTileID();
    int32_t getBotLatCoorOfTileID();

    bool gps2Relative(SegmentID_t refSegID, const std::vector<Point3d_t> &vGps, std::vector<Point3d_t> &vRelPos);
    bool isAdjacent(SegmentID_t srcSeg, SegmentID_t destSeg, SEG_DIRECTION_E &direction);

    void sortSegList(std::vector<SegmentID_t> &segList);
    SegmentID_t getCenterSeg(const std::vector<SegmentID_t> &segList);
    void group(std::vector<SegmentID_t> &segList, std::vector<std::vector<SegmentID_t> > &groupedSeg);

    bool isLess(SegmentID_t seg1, SegmentID_t seg2);
    void getNeighborSegments(std::vector<SegmentID_t> &vSegments,
                             SegmentID_t refSegment,
                             std::vector<SegmentID_t> &vSegGroup);


    bool isGpsValid(const Point3d_t &gps);
    //compute the passed segments by trajectory
    uint32_t calcPassedSeg(const std::vector<Point3d_t> &gpsPointVec, std::vector<SegmentID_t>& vecPassedSeg);
    uint32_t getPassSegments(SegmentID_t segID,
                                const std::vector<Point3d_t> &vecGPSTrajectory,
                                std::vector<SegmentID_t> &vecPassSegIDs);

private:
    /*
     * lontitude [-180,180] convert to [-2^31, 2^31],
     * latitude [-90,90] convert to [-2^30, 2^30], only used 31 bit
     */
    void toNdsCoor(const Point3d_t &gps, int32_t &lonCoor, int32_t &latCoor);
    void toNdsAroundCoor(const Point3d_t &gps, int32_t &lonCoor, int32_t &latCoor);

    /*x31y30x30y29...y1x1y0x0*/
    void toMortonCode(const Point3d_t &gps, int64_t &mortonCode);

    /*
     * the difference with seperateTileID is, the output of longitude and latitude coordinate
     * did not move to high bit, this function is for calculate around tile id
     */
    bool seperateTileIDEx(SegmentID_t tileID, int32_t &lonNdsCoor, int32_t &latNdsCoor);

    /* provide for improving performance */
    bool getTileIdAndAnchorGps(const Point3d_t &gps, SegmentID_t &tileID, Point3d_t &anchorGps);

    bool ndsCoor2Gps(int32_t lonNdsCoor, int32_t latNdsCoor, Point3d_t &gps);

private:
    int32_t level_;
    float64_t tileSize_;
    float64_t ndsCoorUnit_;

    int32_t maxLonCoorOfTileID_;
    int32_t maxLatCoorOfTileID_;
    int32_t TopLatCoorOfTileID_;  //Related to Longitude: 89.9789
    int32_t BottomLatCoorofTileID_; //Related to Longitude: -90

    static uint32_t s_mask_[MAX_TILE_LEVEL + 1];
};

}



#endif
