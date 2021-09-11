/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RoadModelDefs.h
 * @brief  Structure for road geometry data generalized from vehicle
 *******************************************************************************
 */

#ifndef ROAD_MODEL_DEFS_H_
#define ROAD_MODEL_DEFS_H_


#ifndef _BOOST_SERIALIZE_
#define _BOOST_SERIALIZE_
#endif

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "typeDef.h"
#include "CommunicateDef/Communicate.h"

namespace algo
{
    ///////////////////////// define event identity constant /////////////////////

    // road parameter id
    enum ROAD_PARAM
    {
        ROAD_PARAM_PLANE = 0X2000,              // road plane parameter type
        ROAD_PARAM_LINE,                        // all line parameter type
        ROAD_PARAM_LINE_STAIGHT,                // straight line
        ROAD_PARAM_LINE_STAIGHT_DASHED,         // straight dashed line
        ROAD_PARAM_LINE_CURVE,                  // curve line
        ROAD_PARAM_LINE_CURVE_DASHED,           // curve dashed line
        ROAD_PARAM_EQUATION_SOLO,               // line model equation type: one equation
        ROAD_PARAM_EQUATION_DUAL,               // line model equation type: two equations
        // to be extended...
    };




    ///////////////////////// define road data structure ///////////////////////////

    typedef std::vector<ST_ROAD_Point3D> Xh3DPointSet;
    typedef std::vector<int>             XhIntSet;

    // Item with points
    struct ST_ROAD_ItemPoint
    {
        int                 nID;              // global identity from vehicle
        ROAD_ITEM           nItemType;        // item type
        XhIntSet            vecFrameID;       // frame id
        Xh3DPointSet        vecPoint;         // supporting points
        int                 nLastFrame;       // piece modeling position

        ST_ROAD_ItemPoint() : nID(-1), nItemType(ROAD_ITEM_PAINTING), nLastFrame(-1)
        {

        }

    #ifdef _BOOST_SERIALIZE_
        template<typename Archive>
        void serialize(Archive& ar, unsigned int version){
            ar &nID  &nItemType  &nLastFrame;
            ar &vecFrameID;
            ar &vecPoint;
        }
    #endif
    };

    typedef std::vector<ST_ROAD_ItemPoint>  XhItemRawSet;

    struct Line
    {
        int64_t  dbID;
        ST_ROAD_ItemModel lineModel;
        uint32_t lineIndex; //before and back
        Line(): dbID(0), lineIndex(0) {}
    };

    struct LineSet
    {
        std::vector<Line>       lines;
        int32_t                 lineSetIndex; //left right
        float32_t               confidence;
        uint64_t                pieceId;
        uint64_t                batchId;
        LineSet(): lineSetIndex(0), confidence(0.0), pieceId(0), batchId(0){}
    };

    
    struct EdgeSet : public LineSet
    {
        uint8_t type;
        float height;
    };

    typedef std::vector<ST_ROAD_ItemModel>  XhItemModelSet;

    // all road geometry output data
    // painting and lane
    struct ST_ROAD_Geometry
    {
        XhItemModelSet  vecItem;        // painting,.. etc
        XhDriveLaneSet  vecLane;        // lane

#ifdef _BOOST_SERIALIZE_
        template<typename Archive>
        void serialize(Archive& ar, unsigned int version){
            ar &vecItem;
            ar &vecLane;
        }
#endif
    };

    // configure each frame's information for modeling
    struct ST_ROAD_FrameInfo
    {
        int              nFrameID;
        ST_ROAD_Point3D  ptTrace;
        float            fDirection;
        float            fSpeed;
        cv::Mat          campose;

        ST_ROAD_FrameInfo() : nFrameID(-1), fDirection(720.f), fSpeed(0.0f)
        {
            ptTrace.x = 0;
            ptTrace.y = 0;
            ptTrace.z = 0;
        }

        void CopyIn(const ST_ROAD_FrameInfo* pstParam)
        {
            nFrameID   = pstParam->nFrameID;
            ptTrace    = pstParam->ptTrace;
            fDirection = pstParam->fDirection;
            fSpeed     = pstParam->fSpeed;
            campose    = pstParam->campose.clone();
        }
    };

    typedef std::vector<ST_ROAD_FrameInfo>  XhFrameInfoSet;

    // configure for modeling control
    struct ST_ROAD_ModelConfig
    {
        ROAD_ITEM   nItemType;             // modeling/reconstruction algorithm will choose methods differed from different items
        float       fScale;                // the ratio between real measured value(meter) and coordinate value,  if coordinate value 3 pixel means 1.5m, then fScale = 0.5f;
        float       fResolution;           // minimum points distance in the reconstructed world(meter)
        double      dbMaxPieceLen;         // maximum length of an item piece for modeling(meter)

        ST_ROAD_ModelConfig() : nItemType(ROAD_ITEM_PAINTING), fScale(1.0f), fResolution(0.1f)
        {
            dbMaxPieceLen = fResolution * 300;
        }

        void CopyIn(const ST_ROAD_ModelConfig* pstParam)
        {
            nItemType       = pstParam->nItemType;
            fScale          = pstParam->fScale;
            fResolution     = pstParam->fResolution;
            dbMaxPieceLen   = pstParam->dbMaxPieceLen;
        }
    };
}

#endif // _ROAD_MODEL_DEFS_H_
