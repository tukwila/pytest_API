/**
 ************************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 ************************************************************************************
 * @file   RdbV2SRoadObject.h
 * @brief  Definitions of road objects that can be detected
 ************************************************************************************
 */

#ifndef RDB_V2S_ROAD_OBJECT_H
#define RDB_V2S_ROAD_OBJECT_H

#include <vector>    // std::vector
#include <memory>    // std::shared_ptr
#include "typeDef.h" // uint16 etc.
#include "RdbV2SGeometry.h"

namespace roadDBCore
{

typedef uint8_t ReliabilityType_t;

enum ROAD_OBJ_TYPE_E : uint8_t
{
    // Road plane
    ROAD_OBJ_TYPE_ROAD_PLANE_E = 0,

    // Lane boundaries
    ROAD_OBJ_TYPE_LANE_BOUNDARY_E,

    // Objects on road surface
    ROAD_OBJ_TYPE_SURFACE_OBJECT_E,

    // Logic Point
    ROAD_OBJ_TYPE_LOGIC_POINT_E,

    // Lane
    ROAD_OBJ_TYPE_LANE_E,

    // Road edges
    ROAD_OBJ_TYPE_EDGE_E,

    // Steady objects not on road
    ROAD_OBJ_TYPE_FURNITURE_OBJECT_E,

    ROAD_OBJ_TYPE_MAX_E

};

enum LANE_TYPE_E : uint8_t
{
    LANE_TYPE_SIMPLE_E = 0,
    LANE_TYPE_MAX_E
};

enum LANE_BOUNDARY_TYPE_E : uint8_t
{
    LANE_BOUNDARY_TYPE_SIMPLE_E = 0,
    LANE_BOUNDARY_TYPE_MAX_E
};

enum ROAD_SURFACE_OBJ_TYPE_E : uint8_t
{
    ROAD_SURFACE_OBJ_NON_CATEGORIZED_E = 0,
    ROAD_SURFACE_OBJ_PAINT_PED_CROSS_E,
    ROAD_SURFACE_OBJ_PAINT_HALT_LINE_E,
    ROAD_SURFACE_OBJ_PAINT_TEMP_PAINT_E,
    ROAD_SURFACE_OBJ_PAINT_EXCLUSION_E,
    ROAD_SURFACE_OBJ_PAINT_ARROW_E,
    ROAD_SURFACE_OBJ_PAINT_BYCICLE_E,
    ROAD_SURFACE_OBJ_PAINT_BUS_E,
    ROAD_SURFACE_OBJ_PAINT_HOV_E,
    ROAD_SURFACE_OBJ_SPEED_BUMP_E,
    ROAD_SURFACE_OBJ_MAN_HOLE_E,
    ROAD_SURFACE_OBJ_MAX_E
};

enum ROAD_EDGE_TYPE_E : uint8_t
{
    ROAD_EDGE_TYPE_IMPUTED = 0,
    ROAD_EDGE_TYPE_PAVEMENT_EDGE_E,
    ROAD_EDGE_TYPE_CURB_RAIL_E,
    ROAD_EDGE_TYPE_GUARD_RAIL_E,
    ROAD_EDGE_TYPE_JERSEY_WALL_E,
    ROAD_EDGE_TYPE_MAX_E
};

enum ROAD_FURNITURE_OBJ_TYPE_E : uint8_t
{
    ROAD_FURNITURE_OBJ_NON_CATEGORIZED_E = 0,
    ROAD_FURNITURE_OBJ_TRAFFIC_SIGN_E,
    ROAD_FURNITURE_OBJ_TRAFFIC_LIGHT_E,
    ROAD_FURNITURE_OBJ_MAX_E
};

enum ARROW_PAINT_TYPE_E : uint8_t
{
    ARROW_PAINT_TYPE_NON_CATEGORIZED_E = 0,

    /********** forward arrow **********/
    ARROW_PAINT_TYPE_FORWARD_STRAIGHT_E = 1,
    ARROW_PAINT_TYPE_FORWARD_LEFT_TURN_E = 2,
    ARROW_PAINT_TYPE_FORWARD_RIGHT_TURN_E = 3,
    ARROW_PAINT_TYPE_FORWARD_LEFT_MERGING_E = 4,
    ARROW_PAINT_TYPE_FORWARD_RIGHT_MERGING_E = 5,
    ARROW_PAINT_TYPE_FORWARD_STRAIGHT_LEFT_E = 6,
    ARROW_PAINT_TYPE_FORWARD_STRAIGHT_RIGHT_E = 7,
    ARROW_PAINT_TYPE_FORWARD_STRAIGHT_LEFT_RIGHT_E = 8,
    ARROW_PAINT_TYPE_FORWARD_LEFT_RIGHT_E = 9,
    ARROW_PAINT_TYPE_FORWARD_U_TURN_E = 10,
    ARROW_PAINT_TYPE_FORWARD_LEFT_U_TURN_E = 11,
    ARROW_PAINT_TYPE_FORWARD_STRAIGHT_U_TURN_E = 12,
    ARROW_PAINT_TYPE_FORWARD_STRAIGHT_LEFT_U_TURN_E = 13,

    //  Left line system, like U.K, H.K
    ARROW_PAINT_TYPE_FORWARD_U_RIGHT_TURN_E = 14,
    ARROW_PAINT_TYPE_FORWARD_RIGHT_U_RIGHT_TURN_E = 15,
    ARROW_PAINT_TYPE_FORWARD_STRAIGHT_U_RIGHT_TURN_E = 16,
    ARROW_PAINT_TYPE_FORWARD_STRAIGHT_RIGHT_U_RIGHT_TURN_E = 17,

    /********** backward arrow **********/
    ARROW_PAINT_TYPE_BACKWARD_STRAIGHT_E = 18,
    ARROW_PAINT_TYPE_BACKWARD_LEFT_TURN_E = 19,
    ARROW_PAINT_TYPE_BACKWARD_RIGHT_TURN_E = 20,
    ARROW_PAINT_TYPE_BACKWARD_LEFT_MERGING_E = 21,
    ARROW_PAINT_TYPE_BACKWARD_RIGHT_MERGING_E = 22,
    ARROW_PAINT_TYPE_BACKWARD_STRAIGHT_LEFT_E = 23,
    ARROW_PAINT_TYPE_BACKWARD_STRAIGHT_RIGHT_E = 24,
    ARROW_PAINT_TYPE_BACKWARD_STRAIGHT_LEFT_RIGHT_E = 25,
    ARROW_PAINT_TYPE_BACKWARD_LEFT_RIGHT_E = 26,
    ARROW_PAINT_TYPE_BACKWARD_U_TURN_E = 27,
    ARROW_PAINT_TYPE_BACKWARD_LEFT_U_TURN_E = 28,
    ARROW_PAINT_TYPE_BACKWARD_STRAIGHT_U_TURN_E = 29,
    ARROW_PAINT_TYPE_BACKWARD_STRAIGHT_LEFT_U_TURN_E = 30,

    //  Left line system, like U.K, H.K
    ARROW_PAINT_TYPE_BACKWARD_U_RIGHT_TURN_E = 31,
    ARROW_PAINT_TYPE_BACKWARD_RIGHT_U_RIGHT_TURN_E = 32,
    ARROW_PAINT_TYPE_BACKWARD_STRAIGHT_U_RIGHT_TURN_E = 33,
    ARROW_PAINT_TYPE_BACKWARD_STRAIGHT_RIGHT_U_RIGHT_TURN_E = 34,

    ARROW_PAINT_TYPE_MAX_E,
};

enum LOGIC_POINT_TYPE_E : uint8_t
{
    LOGIC_POINT_TYPE_UNKNOWN_E,

    //  the fork point
    LOGIC_POINT_TYPE_FORK_E,
    LOGIC_POINT_TYPE_FORK_COVERGE_E,
    LOGIC_POINT_TYPE_FORK_DIVERGE_E,
    LOGIC_POINT_TYPE_FORK_ENTRANCE_E_E,
    LOGIC_POINT_TYPE_FORK_ENTRANCE_F_E,
    LOGIC_POINT_TYPE_FORK_EIXT_E_E,
    LOGIC_POINT_TYPE_FORK_EIXT_F_E,

    // interdivision and stop line
    LOGIC_POINT_TYPE_STOPLINE_E,
    LOGIC_POINT_TYPE_INTERSECTION_E,

    //  the splitor of lane marking type
    LOGIC_POINT_TYPE_LANE_MARKING_TYPE_SPLITOR_E,
    LOGIC_POINT_TYPE_DASHED_TO_SOLID_SPLITOR_E,
    LOGIC_POINT_TYPE_SOLID_TO_DASHED_SPLITOR_E,

    //  the virtual point
    LOGIC_POINT_TYPE_VIRTUAL_TRANSITION_IN_E,
    LOGIC_POINT_TYPE_VIRTUAL_TRANSITION_OUT_E,
    LOGIC_POINT_TYPE_VIRTUAL_NORMAL,

    //  the lane-changed point
    LOGIC_POINT_TYPE_LANE_CHANGE_E,

    //  the endPoint of line
    LOGIC_POINT_TYPE_START_ENDPOINT_E,
    LOGIC_POINT_TYPE_END_ENDPOINT_E,

    LOGIC_POINT_TYPE_MAX_E
};


/*
enum UPDATE_TYPE : uint8_t
{
    UPDATE_TYPE_NEW_OBJ = 0,
    UPDATE_TYPE_UPDATE_OBJ,
    UPDATE_TYPE_DELETE_OBJ
}
*/

struct RoadObject_t
{
    ROAD_OBJ_TYPE_E eRoadObjType;

    RoadObject_t(ROAD_OBJ_TYPE_E eRoadObjTypeIn = ROAD_OBJ_TYPE_MAX_E): eRoadObjType(eRoadObjTypeIn) {}
    //UPDATE_TYPE   updateType;
};

/***********************************************
 * Lane boundaries
 ***********************************************/
// 3D road model - consists of painting item indexes
struct LaneLogicInfo_t
{
    ObserveKfInfo_t observeKfInfo;

    int16_t      leftLineIdx;     // current lane's left painting index (item's identity of ST_ROAD_ItemModel)
                                  // if < 0, means no left painting. -1 : no detection,   -2 : detected but failed to be modeled

    int16_t      rightLineIdx;    // current lane's right painting's index (item's identity of ST_ROAD_ItemModel)
                                  // if < 0, means no right painting. -1 : no detection,   -2 : detected but failed to be modeled
};

struct LaneBoundary_t : public RoadObject_t
{
    LANE_BOUNDARY_TYPE_E eLaneBoundaryType;

    LaneBoundary_t(LANE_BOUNDARY_TYPE_E eInLaneBoundaryType):
                         RoadObject_t(ROAD_OBJ_TYPE_LANE_BOUNDARY_E),
                         eLaneBoundaryType(eInLaneBoundaryType)
    {
    }
};

struct LaneLineSeg_t
{
    int32_t lineSegIndex;   //  line segment
    float32_t width;        //  line width
    LANE_MARKING_TYPE_E lineSegType;
    LANE_MARKING_COLOR_TYPE_E lineColorType;
    bool bFake;

    LaneLineSeg_t(): lineSegIndex(-1), width(-1), lineSegType(LANE_MARKING_TYPE_UNLABELED_E), lineColorType(LANE_MARKING_COLOR_TYPE_UNLABELED_E), bFake(false) {}
};

struct LaneLine_t // For SDOR version 1 serialization
{
    /*
       Code 2-byte vector size,
       supports up to 65536 line segments.
    */
    std::vector<LaneLineSeg_t> vecLineSegs;

    LaneLine_t() {}
};

struct LogicPoint_t : public RoadObject_t
{
    int32_t index;
    LOGIC_POINT_TYPE_E eLoigcPointType;
    Point3f_t logicPoint;
    std::vector<int32_t> lineIndexs;

    LogicPoint_t(LOGIC_POINT_TYPE_E eInLogicPointType = LOGIC_POINT_TYPE_MAX_E) :
                 RoadObject_t(ROAD_OBJ_TYPE_LOGIC_POINT_E),
                 index(-1),
                 eLoigcPointType(eInLogicPointType)
    {}
};

struct Lane_t : public RoadObject_t
{
    LANE_TYPE_E eLaneType;

    Lane_t(LANE_TYPE_E eInLaneType): RoadObject_t(ROAD_OBJ_TYPE_LANE_E), eLaneType(eInLaneType) {}
};

struct SimpleLane_t : public Lane_t
{
    bool        isCurLane;
    std::vector<int32_t> startLogicPointIndices;
    std::vector<int32_t> endLogicPointIndices;
    std::vector<float32_t> laneWidths;
    LaneLine_t  lLine;
    LaneLine_t  rLine;

    SimpleLane_t(): Lane_t(LANE_TYPE_SIMPLE_E),
                         isCurLane(false)
    {}
};

struct SimpleLaneBoundary_t : public LaneBoundary_t  // For SDOR version 1 serialization
{
    int32_t index;
    float32_t weight;
    std::shared_ptr<Geometry_t> curve;

    SimpleLaneBoundary_t(): LaneBoundary_t(LANE_BOUNDARY_TYPE_SIMPLE_E),
                                     index(-1),
                                     weight(0.0F)
    {}
};

/***********************************************
 * Road surface objects
 ***********************************************/
struct RoadSurfaceObject_t : public RoadObject_t
{
    ROAD_SURFACE_OBJ_TYPE_E         eObjType;
    uint32_t                        color;      //  four uchar type, format : (unused uchar) + (r) + (g) + (b)
    std::shared_ptr<Geometry_t>     geometryInfo;
    ReliabilityType_t               reliability;
    ObserveKfInfo_t                 observeKfInfo;
    std::vector<std::shared_ptr<Geometry_t>> descriptor;

    RoadSurfaceObject_t(ROAD_SURFACE_OBJ_TYPE_E eInObjType):
                                 RoadObject_t(ROAD_OBJ_TYPE_SURFACE_OBJECT_E),
                                 eObjType(eInObjType),
                                 color(0),
                                 reliability(0)
    {
    }
};

struct PedCrossingPaint_t : public RoadSurfaceObject_t
{
    PedCrossingPaint_t(): RoadSurfaceObject_t(ROAD_SURFACE_OBJ_PAINT_PED_CROSS_E) {}
};

struct HaltLinePaint_t : public RoadSurfaceObject_t
{
    HaltLinePaint_t(): RoadSurfaceObject_t(ROAD_SURFACE_OBJ_PAINT_HALT_LINE_E) {}
};

struct TempPaint_t : public RoadSurfaceObject_t
{
    TempPaint_t(): RoadSurfaceObject_t(ROAD_SURFACE_OBJ_PAINT_TEMP_PAINT_E) {}
};

struct ExclusionPaint_t : public RoadSurfaceObject_t
{
    ExclusionPaint_t(): RoadSurfaceObject_t(ROAD_SURFACE_OBJ_PAINT_EXCLUSION_E) {}
};

struct ArrowPaint_t : public RoadSurfaceObject_t
{
    ARROW_PAINT_TYPE_E eArrowType;

    ArrowPaint_t(ARROW_PAINT_TYPE_E eInArrowType):
                      RoadSurfaceObject_t(ROAD_SURFACE_OBJ_PAINT_ARROW_E),
                      eArrowType(eInArrowType)
    {
    }
};

struct ByciclePaint_t : public RoadSurfaceObject_t
{
    ByciclePaint_t(): RoadSurfaceObject_t(ROAD_SURFACE_OBJ_PAINT_BYCICLE_E) {}
};

struct BusPaint_t : public RoadSurfaceObject_t
{
    BusPaint_t(): RoadSurfaceObject_t(ROAD_SURFACE_OBJ_PAINT_BUS_E) {}
};

struct HOVPaint_t : public RoadSurfaceObject_t
{
    HOVPaint_t(): RoadSurfaceObject_t(ROAD_SURFACE_OBJ_PAINT_HOV_E) {}
};

struct SpeedBump_t : public RoadSurfaceObject_t
{
    SpeedBump_t(): RoadSurfaceObject_t(ROAD_SURFACE_OBJ_SPEED_BUMP_E) {}
};

struct Manhole_t : public RoadSurfaceObject_t
{
    Manhole_t(): RoadSurfaceObject_t(ROAD_SURFACE_OBJ_MAN_HOLE_E) {}
};


struct RoadPaintLine_t  :  public RoadObject_t
{
    LANE_MARKING_TYPE_E eRoadPaintType; //type of paint, e.g. dashed, solid etc.
    NURBS_t lineShape;                //descript the shape of the paint
    std::vector<int32_t> vLeftPaintIdx;        //index of the left side paint
    std::vector<int32_t> vRightPaintIdx;       //index of the right side paint
    std::pair<uint64_t, int32_t> prevPaintIdx; //the connected paint in previous division, first is index of division, second it index of paint.
    std::pair<uint64_t, int32_t> nextPaintIdx; //the connected paint in next division
    int32_t crossPointIdx1;               //index of connected cross point
    int32_t corssPointIdx2;
    float32_t paintWidth;                 //width of paint
};

struct RoadCrossPoint_t : public RoadObject_t
{
    ROAD_POINT_TYPE_E eRoadPointType;
    Point3f_t position;
    std::vector<int32_t> vPaintIdx;              //index fo the connected paint
};

/***********************************************
 * Road edges
 ***********************************************/
struct RoadEdge_t : public RoadObject_t
{
    ROAD_EDGE_TYPE_E                eRoadEdgeType;
    std::shared_ptr<Geometry_t>     geometryInfo;
    ReliabilityType_t               reliability;
    ObserveKfInfo_t                 observeKfInfo;
    bool                            bRightSide; //left : false   right : true

    RoadEdge_t(ROAD_EDGE_TYPE_E eInRoadEdgeType):
                     RoadObject_t(ROAD_OBJ_TYPE_EDGE_E),
                     eRoadEdgeType(eInRoadEdgeType),
                     reliability(0),
                     bRightSide(false)
    {
    }
};

struct PavementEdge_t : public RoadEdge_t
{
    PavementEdge_t(): RoadEdge_t(ROAD_EDGE_TYPE_PAVEMENT_EDGE_E) {}
};

struct Curb_t : public RoadEdge_t
{
    Curb_t(): RoadEdge_t(ROAD_EDGE_TYPE_CURB_RAIL_E) {}
};

struct JerseyWall_t : public RoadEdge_t
{
    JerseyWall_t(): RoadEdge_t(ROAD_EDGE_TYPE_JERSEY_WALL_E) {}
};

struct GuardRail_t : public RoadEdge_t
{
    GuardRail_t(): RoadEdge_t(ROAD_EDGE_TYPE_GUARD_RAIL_E) {}
};

/***********************************************
 * Road furniture objects
 ***********************************************/
struct RoadFurnitureObject_t : public RoadObject_t
{
    ROAD_FURNITURE_OBJ_TYPE_E       eObjType;
    std::shared_ptr<Geometry_t>     geometryInfo;
    ReliabilityType_t               reliability;
    ObserveKfInfo_t                 observeKfInfo;
    std::vector<std::shared_ptr<Geometry_t> > descriptor;

    RoadFurnitureObject_t(ROAD_FURNITURE_OBJ_TYPE_E  eInObjType):
                                   RoadObject_t(ROAD_OBJ_TYPE_FURNITURE_OBJECT_E),
                                   eObjType(eInObjType),
                                   reliability(0)
    {
    }
};

struct TrafficSign_t : public RoadFurnitureObject_t
{
    uint32_t  trafficSignType;

    TrafficSign_t(): RoadFurnitureObject_t(ROAD_FURNITURE_OBJ_TRAFFIC_SIGN_E),
                           trafficSignType(0)
    {
    }
};

struct TrafficLight_t : public RoadFurnitureObject_t
{
     TrafficLight_t(): RoadFurnitureObject_t(ROAD_FURNITURE_OBJ_TRAFFIC_LIGHT_E) {}
};

struct RTMatrixItemSR_t
{
    uint32_t    kfIdx;
    bool        isKeyFrame;
    float32_t   quaternion[4];
    float32_t   translation[3];

    RTMatrixItemSR_t(): kfIdx (0), isKeyFrame(true), quaternion{0}, translation{0} {}
};

struct RTItemsInDivisionSR_t
{
    uint64_t divisionID;
    uint64_t referenceID;

    /* Division version or reference version in the division. */
    uint64_t version;

    /*road plane height*/
    float32_t height;

    /*
       Code 2-byte vector size,
       supports up to 65536 RT items.
    */
    std::vector<RTMatrixItemSR_t> vecRTItems;

    RTItemsInDivisionSR_t(): divisionID (0LL), referenceID(0LL), version(0LL), height(0) {}
};

struct LineViewItem_t
{
    int16_t lineId;        //  nurbs line id, support up to 32767 line in single vehicle report

    /*
       Code 2-byte vector size,
       supports up to 65536 point.
    */
    std::vector<Point2f_t> vec2dPaint;

    LineViewItem_t() : lineId(-1) {}
};

struct ViewInfoVehicle_t
{
    /*
       Code 2-byte vector size,
       supports up to 65536 lines.
       the line ids and 2d points that can be seen in the view.
    */
    std::vector<LineViewItem_t> vecLineViewItems;
};

struct RoadGeometryPayload_t: public PayloadBase_t
{
    /* code 2 bytes vector size. */
    std::vector<std::shared_ptr<RoadObject_t> > vecRoadObjects;

    /*
       Code 2-byte vector size,
       supports up to 65536 divisions.
    */
    std::vector<RTItemsInDivisionSR_t> vecDivisionRTInfo;

    /*
       Code 2-byte vector size,
       supports up to 65536 vehicle view info.
       size is the same as RT frame num.
    */
    std::vector<ViewInfoVehicle_t> vecViewInfo;

    float32_t camK[15]; // fx, fy, cx, cy, k1, k2, k3, p1, p2, x, y, z, roll, pitch, yaw
    float32_t roadPlane[4]; // A, B, C, D : A*x + B*y +C*z + D = 0

    RoadGeometryPayload_t():
                          PayloadBase_t(SNIPPET_PAYLOAD_TYPE_ROAD_E)
    {
        memset(camK, 0, sizeof(camK));
        memset(roadPlane, 0, sizeof(roadPlane));
    }
};



}

#endif //RDB_V2S_ROAD_OBJECT_H
