/**
 ************************************************************************************
 *                         RoadDB Confidential
 *                    Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 ************************************************************************************
 * @file   RdbV2SLandmark.h
 * @brief  Definitions of Landmark objects reported from vehicle to server
 * @date   August 20th, 2019
 ************************************************************************************
 */

#ifndef RDB_V2S_LANDMARK_H
#define RDB_V2S_LANDMARK_H

#include <vector>           // std::vector
#include <memory>           // std::shared_ptr
#include "typeDef.h"        // uint32_t etc.
#include "RdbV2SGeometry.h"
#include "RdbV2SSlam.h"
#include "TVMLableDef.h"

namespace roadDBCore
{

enum LM_SHAPE_E : uint8_t
{
    LM_SHAPE_UNKNOWN_E = 0,
    LM_SHAPE_BOX_E,
    LM_SHAPE_CYLINDER_E,
    LM_SHAPE_EXTRUSION_E,
    LM_SHAPE_PRIM,
    LM_SHAPE_POLYHEDROM,
    LM_SHAPE_DRIVEIN_E,
    LM_SHAPE_FREEVOXEL_E,
    LM_SHAPE_CLUSTEREDVOXEL_E,
    LM_SHAPE_MAX_E
};

enum LM_COLOR_E : uint8_t
{
    LM_COLOR_UNKNOWN_E = 0,
    LM_COLOR_WHITE_E,
    LM_COLOR_BLACK_E,
    LM_COLOR_YELLOW_E,
    LM_COLOR_WHITE_WHITE_E,
    LM_COLOR_BLACK_BLACK_E,
    LM_COLOR_YELLOW_YELLOW_E,
    LM_COLOR_WHITE_WHITE_WHITE_E,
    LM_COLOR_WHITE_YELLOW_WHITE_E,
    LM_COLOR_MAX_E
};

enum LM_VOLUME_E : uint8_t
{
    LM_VOLUME_BOX_E = 0,
    LM_VOLUME_CYLINDER_E,
    LM_VOLUME_EXTRUSION_E,
    LM_VOLUME_IRREGULAR_E,
    LM_VOLUME_MAX_E
};

/*
   all coordinates are related to Snippet's refGps(anchor point gps)
   one volume represents a submodel of object
   only 3D point(relLon/relLat/relAlt) is used in TmpGPSData for all kinds of volume
*/
struct LmVolume_t
{
    LM_VOLUME_E         eType = LM_VOLUME_MAX_E;

    /* sub component ID for a volume. an object may contains serveral volume */
    uint32_t            id = 0;

    /* model parameters */
    Point3f_t           geoCenter;       // geometry center
    Point3f_t           gravityCenter;   // gravity center
    Point3f_t           normal;          // most components are supposed to perpendicular to road plane
    float32_t           height = 0.0F;   // volume's height, or cylinder length

    /* density is required by conti */
    float32_t           density = 0.0F;  // point cloud density

    /*
       road plane geometry:
       road plane's height, local region under volume;
       this field is for visualization or APA registration
    */
    float32_t           roadHeight = 0.0F;

    LmVolume_t(LM_VOLUME_E eTypeIn): eType(eTypeIn) {}
};

/* ordinary volume shape */
struct LmBox_t : public LmVolume_t
{
    float32_t           width = 0.0F;
    float32_t           thickness = 0.0F;

    /*
       the vector point from cube center to center of width-thickness
       surface and point against driver view.
    */
    Point3f_t           orientation;

    LmBox_t(): LmVolume_t(LM_VOLUME_BOX_E) {}
};

/* pole-like volume shape */
struct LmCylinder_t : public LmVolume_t
{
    float32_t           radius = 0.0F;

    LmCylinder_t(): LmVolume_t(LM_VOLUME_CYLINDER_E) {}
};

/* wall-like volume shape */
struct LmExtrusion_t : public LmVolume_t
{
    std::shared_ptr<Line_t> spBaseLine;
    float32_t               thickness = 0.0F;

    LmExtrusion_t(): LmVolume_t(LM_VOLUME_EXTRUSION_E) {}
};

struct LmIrregularBody_t : public LmVolume_t
{
    LM_SHAPE_E              eIrregularType = LM_SHAPE_MAX_E;  // prim or polyhedron

    /*
       1.prim:  vecVertexs_ are for the bottom polygon
       2.polyhedron: vecVertexs_ are for all the vertexs of the body.
    */
    std::vector<Point3f_t>  vecVertexs;

    LmIrregularBody_t(): LmVolume_t(LM_VOLUME_IRREGULAR_E) {}
};

struct LmVoxel_t
{
    Point3f_t               gravityCenter;          // gravity center
    uint8_t                 geoConf = 0;            // geometry reliability
    int8_t                  sdf = 0;                // TSDF value, quantilized to -100~100 [original -1~1]
    std::vector<uint8_t>    subSemanticType;        // semantic type
    std::vector<uint8_t>    subSemanticConf;        // semantic reliability

    /*
       Note:
       vecFrmIdx.size = vecObs.size for Roadscan Paint
       if the voxel is not Roadscan Paint, vecObs may be empty at this stage
    */
    std::vector<uint32_t>   vecFrmIdx;              // frame index in vector, not frame ID
    std::vector<Point2f_t>  vecObservers;           // observers in KF (u, v)

    uint32_t                roadPlaneFrmIdx = 0;    // use which pose for road plane projection
};

struct LmRoadPlane_t
{
    Point3f_t   normal; // don't need to transfer coordinates(based on camera coordinate system)
    float32_t   height = 0.0F;
};

struct LmObject_t
{
    uint32_t                    id = 0;                             // landmark ID
    std::string                 picName;                            // Picture name

    /* semantic information */
    TVM_DRIVER_VIEW_LABEL_E     semanticType = TVM_DRIVER_VIEW_LABEL_MAX_E;
    float32_t                   semanticConf = 0.0F;                // semantic reliability
                                                                    // value 0~1, paint semanticConf = 1 means in current lane
    /*
       More accurate types divided from one (driver view) semantic type,
       such as kinds of paint : arrow / crosswalk / lane boundary
    */
    uint32_t                    subSemantic = 0;

    float32_t                   quality = 0.0F;    // quality indicator for server selection
                                                   // average distance(meter) to its nearest camera center

    /* geometric information */
    LmRoadPlane_t               relRoadPlane;      // relative road plane, normal vector (3 float) and height (1 float)
    std::vector<Point3f_t>      vecRefPoints;      // reference points from lane center line
    std::vector<uint32_t>       vecRefMapPtIdx;    // index of feature 3D points in slam snippet, connect to SLAM
    std::vector<uint32_t>       vecFrmIdx;         // observation frame index in vector, not frame ID

    /*
       volume/model information : one landmark consists of several volumes
       The volume can be box, cylinder, wall-like component and irregular body, prism component
    */
    std::vector<std::shared_ptr<LmVolume_t>>  vecSpLmVolumes;   // irregular body, prism component

    /*
       voxel information : grid reporting
       No more than 2^16 voxels in a landmrak object.
       code 2 bytes vector size for serialization.
    */
    std::vector<std::shared_ptr<LmVoxel_t>>   vecSpVoxels;      // voxels for the landmark object
    float32_t                   voxelResolution = 0.25F;        // voxel's resolution, meters
    std::vector<uint16_t>       vecVertexVoxelIdx;              // vertex (voxel index) of this object, 16-bit voxel index

    /* additional description */
    std::vector<uint8_t>        colorSemanticInfo;              // reserved for color/semantic description
};

// logic type for LmSnippetSR_t::vvecObjsLogic, will be refined later
enum LM_LOGICRELATION_E
{
    LM_LOGICRELATION_UNKNOWN_E = 0,
    LM_LOGICRELATION_SEQ_E     = 1,      // one laneboudary
    LM_LOGICRELATION_ACROSS_E  = 2,      // divering / coverging
    LM_LOGICRELATION_GROUP_E   = 3,
    LM_LOGICRELATION_MAX_E
};

struct LmSnippetSR_t
{
    /* voxel landmarks */
    std::vector<std::shared_ptr<LmObject_t>> vecLmObjs;         // reported landmarks to server

    /*
       logic connection of objects
       uint32_t: ID of landmark object in vecLmObjs
       std::vector<uint32_t> : a group of small objects, eg:
           1.one lane boundary consists some dash and solid lines
           2.one complete traffic sign consists of a sign and a pole
           3.one pedestrian bridge consists of cement building and fence
       std::pair<uint32_t, std::vector<uint32_t>> :
           uint32_t is the connection type (undefined yet, default for laneboundary)
       vvecObjsLogic : all groups from one drive
    */
    std::vector<std::pair<uint32_t, std::vector<uint32_t>>> vvecObjsLogic;

    std::shared_ptr<SlamSnippetSR_t>   spSlamSnippet;
};

struct LmSnippetPayload_t: public PayloadBase_t
{
    ModelConfigVehicle_t config;

    /* code 2 bytes vector size for serialization. */
    std::vector<LmSnippetSR_t>  vecLmSnippets;

    LmSnippetPayload_t(): PayloadBase_t(SNIPPET_PAYLOAD_TYPE_LM_E) {}
};

inline void copySnippetPayload(const SlamSnippetPayload_t &slamPayload, LmSnippetPayload_t &lmPayload)
{
    copyModelConfig(slamPayload.config, lmPayload.config);
    lmPayload.vecLmSnippets.clear();

    for (auto &slamSnippet : slamPayload.vecSpSlamSnippets)
    {
        LmSnippetSR_t lmSnippet;
        lmSnippet.spSlamSnippet = std::make_shared<SlamSnippetSR_t>();
        *(lmSnippet.spSlamSnippet.get()) = *(slamSnippet.get());
        lmPayload.vecLmSnippets.emplace_back(lmSnippet);
    }

    return;
}
}


#endif //RDB_V2S_LANDMARK_H


