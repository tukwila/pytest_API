/**
 *******************************************************************************
 *                       RoadDB Confidential
 *       Copyright (c) Continental AG. 2016-2018, RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IServerTransfer.h
 * @brief  Head file of class ITransfer which define the interfaces provided by
 *            transfer in server.
 *******************************************************************************
 */
#ifndef ISERVER_TRANSFER_H_
#define ISERVER_TRANSFER_H_
#include <string>
#include <list>
#include <map>
#include <set>
#include <utility>
//#include <opencv2/imgproc.hpp>
#include <cstring>
#include <limits> // numeric_limits
#include "typeDef.h"
#include "utilityData.h"

#include "CommunicateDef/RdbV2SRoadObject.h"
#include "CommunicateDef/RdbV2SSlam.h"
#include "CommunicateDef/RdbV2SLandmark.h"
//#include "algoInterface/RoadModelDefs.h"
#include "segment/Segment.h"

//declare the two class which is defined in algo


namespace roadDBCore
{


using DivisionID_t = uint64_t;
using ReferenceID_t = uint64_t;
using NodeID_t = uint64_t;
using TaskType_t = uint8_t;

enum DB_REF_MODE_E : uint8_t
{
    DB_REF_MODE_WHOLE_E = 0,    // The whole reference (General info + frames + points + objects)
    DB_REF_MODE_GENERAL_E,      // Only General info of reference
    DB_REF_MODE_FRM_OBJ_E,      // General info, frames and objects
    DB_REF_MODE_NO_REF_E,       // There is no reference info in division
    DB_REF_MODE_MAX_E
};

enum DB_REF_ATTR_E : uint8_t
{
    DB_REF_ATTR_PRIMARY_E = 0,    // Primary reference
    DB_REF_ATTR_SECOND_E,         // Secondary reference
    DB_REF_ATTR_RESERVER_E,       // No alignment reference
    DB_REF_ATTR_MAX_E
};

enum TASK_SPAWN_MODE_E : uint8_t
{
    TASK_SPAWN_MODE_DIVIDE_E = 0,
    TASK_SPAWN_MODE_MERGE_E,
    TASK_SPAWN_MODE_MAX_E
};

enum DATA_ACCESS_MODE_E : uint8_t
{
    DATA_ACCESS_MODE_NONE_E = 0,
    DATA_ACCESS_MODE_R_E,        // Read
    DATA_ACCESS_MODE_RW_E,       // Read and Write
    DATA_ACCESS_MODE_MAX_E
};

struct TaskDependency_t
{
    TaskType_t type = 0;
    std::vector<DivisionID_t> divisions;
};

struct TaskScheduleItem_t
{
    TaskType_t type = 0;
    std::string taskName;

    std::vector<DivisionID_t> divisionPath;
    std::vector<DivisionID_t> divisions;

    uint32_t priority = 0;
    uint32_t memory = 0;     // Memory usage, measured by Megebytes.
    uint8_t  threadsNum = 0; // Potential max threads number

    DATA_ACCESS_MODE_E eBDataAccess = DATA_ACCESS_MODE_NONE_E;
    DATA_ACCESS_MODE_E eCDataAccess = DATA_ACCESS_MODE_NONE_E;
    DATA_ACCESS_MODE_E eVDataAccess = DATA_ACCESS_MODE_NONE_E;
};

struct ScheduleInfo_t
{
    TASK_SPAWN_MODE_E eSpawnMode = TASK_SPAWN_MODE_DIVIDE_E;

    std::vector<TaskScheduleItem_t> tasks;
    std::vector<TaskDependency_t> dependency; //Scheduling dependency of all the tasks above.
    std::vector<DivisionID_t> unusedDivisions;
};

struct DBFrame_t
{
    uint32_t    nID = 0;
    bool        bKF = false;
    uint64_t    nBatchID = 0LL;
    Point3f_t   coordinate; //GPS

    float64_t   rotation[9]{};
    float64_t   translation[3]{};

    float32_t   confidence = 0.0F;

    /* The original index of frame in rtv which generates this key frame */
    int32_t     oriFIdx = -1;

    /* The ratio of the coverage area of big vehicle in a frame */
    uint8_t     coverageRatio = 0;
};

struct DBObserver_t
{
    uint32_t    fnID = 0;               //indicate frame with specified nID value fnID
    Point2f_t   points;                 //uv
    uint8_t     property = 0;           //0: nature; 1:soft
    std::vector<uint8_t> descriptor;    //only not empty for descriptor observer
};

struct DBPoint_t  // with 3D descriptor
{
    uint32_t                    nID = 0;

    // 0: mappoint; 1: voxel point; 2: paint point
    DB_POINT_TYPE_E             ePointType = DB_POINT_TYPE_MAX_E;

    /* Code 2-byte vector size */
    std::vector<DBObserver_t>   observers;

    // it is world position for mappoint and is gravity center for paint point and voxel point
    Point3f_t                   pos;

    float32_t                   confidence = 0.0F;

    std::map<uint8_t, uint8_t>  mSemanticInfo;   // semantic type and confidence
};

struct DBVoxelPoint : public DBPoint_t
{
    uint8_t  geoConf = 0;   // geometry reliability(used for paint point and voxel point)
    int8_t   tsdf = 0;      // TSDF value, quantilized to -100~100 [original -1~1](only voxel point uses, not paint point)
    uint32_t hostKfNID = 0; // nID of host kf
};

struct DBPaintPoint : public DBPoint_t
{
    uint8_t  geoConf = 0;   // geometry reliability(used for paint point and voxel point)
    uint32_t hostKfNID = 0; // nID of host kf
};

//the relationship between two reference mapPoints
struct InlinerMpsInfo_t
{
    uint64_t ref1Id = 0LL;
    uint64_t ref2Id = 0LL;
    //vector<pait<id of mp1 of ref1, id of mp2 of ref2>>
    std::vector<std::pair<uint64_t, uint64_t>> mpInlinerIds;
};

struct DBFramesInfo_t
{
    std::vector<DBFrame_t>  vecFrames;
};

struct DBPointsInfo_t
{
    std::vector<DBPoint_t>  vecMps;
};

struct DBObject_t  // with 3D Object
{
    //object id same to snippet
    uint32_t                nID = 0;
    //semanticType subdividedSemantic semanticConf from snippet
    uint8_t                 semanticType = 0;
    uint32_t                subdividedSemantic = 0;
    float32_t               semanticConf = 0.0f;
    //objectSemantic used for SAM
    uint8_t                 objectSemantic = 0;

    LANE_MARKING_COLOR_TYPE_E colorType = LANE_MARKING_COLOR_TYPE_MAX_E;

    //object's cut Segment ID in DBServerReference_t::vecCutSegKfIdx, used for SAM
    int32_t                 cutSegIdx = -1;
    std::vector<std::shared_ptr<DBPoint_t>> vecSpVoxelPoint;
    float32_t               relRoadPlane[4] = {0.0f};

    //extra information from first Volume in object. See RdbV2SLandmark.h
    //0: height (from base class LmVolume_t, for all Volume)
    //LmBox_t: 1: width; 2: thickness
    //LmCylinder_t: 1: radius; 2: None
    //LmExtrusion_t: 1: None; 2: thickness
    //LmIrregularBody_t: 1: None; 2: None
    float32_t               infoFromVolume_[3] = {0.0f};
};

struct DBObjectsInfo_t
{
    std::vector<DBObject_t> vecObjects;

    /*
       logic connection of objects
       uint32_t: ID of landmark object
       std::vector<uint32_t> : a group of small objects, eg:
           1.one lane boundary consists some dash and solid lines
           2.one complete traffic sign consists of a sign and a pole
           3.one pedestrian bridge consists of cement building and fence
       std::pair<uint32_t, std::vector<uint32_t>> :
           uint32_t is the connection type (undefined yet, default for laneboundary)
       vvecObjsLogic : all groups from one drive
    */
    std::vector<std::pair<uint32_t, std::vector<uint32_t>>> vvecObjsLogic;
};

struct DBServerReference_t
{
    /* Save the following fields into DB */
    ReferenceID_t               dbID = 0LL;
    ReferenceID_t               originalID = 0LL;
    DivisionID_t                divisionID = 0LL;
    float32_t                   confidence = 0.0F;
    uint64_t                    version = 0LL;
    DB_REF_ATTR_E               attribute = DB_REF_ATTR_PRIMARY_E;
    uint32_t                    optTimes = 0;   // Optimization times
    float32_t                   oriRoadPlane[4] = {0.0f};

    ModelConfigServer_t         config;

    /* Used in T20, SAM cut reference by kfIDxs to several segments for refining object optimizing*/
    std::vector<std::pair<uint32_t, uint32_t>> vecCutSegKfIdx;

    /* The following field would be serialized as a file with a suffix ".frm" */
    std::shared_ptr<DBFramesInfo_t>      spFramesInfo;

    /* The following field would be serialized as a file with a suffix ".pnt" */
    std::shared_ptr<DBPointsInfo_t>      spPointsInfo;

    /* The following field would be serialized as a file with a suffix ".obj" */
    std::shared_ptr<DBObjectsInfo_t>     spObjectsInfo;

    /* The following fields are just used for data processing */
    uint8_t                     status = 0;     // 0:not merged, 1:merged, 2:optimized
    DB_REF_MODE_E               eMode = DB_REF_MODE_WHOLE_E;
};

struct DBDivision_t
{
    DivisionID_t                dbID;               // generate by database;
    DivisionID_t                originalID;
    float32_t                   curveLength;
    SegmentID_t                 segmentID;          // localization not need

    NodeID_t                    nodeIDA;
    NodeID_t                    nodeIDB;
    uint8_t                     status;             // 0:no operation, 1:suggest to optimize

    //for skeleton node extract api begin
    uint16_t                    trajectoryType;     // 1: trajectory in form of GPS points, 0: for no gps trajectory

    /* Code 2-byte vector size for GPS Trajectory , supports up to 65536 Trajectories*/
    std::vector<Point3d_t>      vecGPSTrajectory;

    //Passed Segments calculate according to vecGPSTrajectory
    std::set<SegmentID_t>       passSegIDs;

    std::vector<InlinerMpsInfo_t> vecInLinerMapsInfo;

    DBDivision_t():
                    dbID(0L),
                    originalID(0L),
                    curveLength(0.0F),
                    segmentID(0),
                    nodeIDA(0),
                    nodeIDB(0),
                    status(0),
                    trajectoryType(0)
    {}
};

struct DBNode_t
{
    NodeID_t                  dbID; //system
    NodeID_t                  originalID;
    SegmentID_t               segmentID;
    Point3f_t                 coordinate;
    Point3f_t                 originCoordinate;
    Point3d_t                 absCoordinate;

    /* Code 1-byte vector size for Divisions , support up to 256 Divisions*/
    std::vector<DivisionID_t> vecDivisionIDs; //localization not need, not save in DB

    //for skeleton node extract api begin
    uint32_t                  type;       //2: skeleton extracted interdivision node, 1: skeleton extracted node, 0: normal node

    DBNode_t() : dbID(0L), originalID(0L), segmentID(0), type(0) {}
};


struct DBDivisionDetail_t
{
    std::shared_ptr<DBDivision_t>     spDivision;

    /* Code 1 byte vector size */
    std::vector<std::shared_ptr<DBServerReference_t>> vecSpReferences;

    std::shared_ptr<DBNode_t>   spNodeA;
    std::shared_ptr<DBNode_t>   spNodeB;
    std::vector<uint64_t>       deleteRefID;
    std::string cacheData;

    DBDivisionDetail_t(std::shared_ptr<DBDivision_t> &spDivisionIn,
                                std::vector<std::shared_ptr<DBServerReference_t>> &vecSpReferencesIn,
                                std::shared_ptr<DBNode_t> &spNodeAIn,
                                std::shared_ptr<DBNode_t> &spNodeBIn):
                                spDivision(spDivisionIn),
                                vecSpReferences(vecSpReferencesIn),
                                spNodeA(spNodeAIn),
                                spNodeB(spNodeBIn)
    {}

    DBDivisionDetail_t() {}
};

struct DivisionChange_t
{
    std::string snippetName;
    std::vector<uint64_t> divisionID;
};

struct SamRtInfo_t
{
    std::string rtvName;

    DivisionID_t divisionID = 0LL;
    ReferenceID_t referenceID = 0LL;
    uint64_t baseTimestamp = 0LL;

    // extract slam pose from backendDB reference
    std::vector<KeyFrameSR_t>  vecSlamPose;
};

/**
 *******************************************************************************
 * @class IServerTransfer
 * @brief This class is used to defined interfaces for reading and writing
 *        division data in server side.
 *
 *******************************************************************************
 */
class IServerTransfer
{
public:
    virtual ~IServerTransfer() {};


    /**
     *******************************************************************************
     *  @brief setSegID - Set Segment ID.
     *
     *  @param [In]  - segID   segment ID
     *
     *  @ret void
     *
     *******************************************************************************
     */
    virtual void setRefSegID(SegmentID_t segID) = 0;

    /**
     *******************************************************************************
     *  @brief getSegID - Get Segment ID.
     *
     *  @param [out]  - segID   segment ID
     *
     *  @ret void
     *
     *******************************************************************************
     */
    virtual void getRefSegID(SegmentID_t &segID) = 0;

    /**
     *******************************************************************************
     *  @brief getDivisionID - Get ID set of all Divisions  need to process
     *
     *  @param [out]  - divIDs   Divisions ID
     *
     *  @ret void
     *
     *******************************************************************************
     */
    virtual uint32_t getDivisionID(std::vector<DivisionID_t>& divIDs) = 0;

    /**
     *******************************************************************************
     *  @brief getDivisions - Get all Divisions need to process
     *
     *  @param [out]  - divisions   Division data
     *
     *  @param [in]  - eRefMode   Reference mode, which means the data organization
     *                  mode of references in a division
     *
     *  @ret uint32_t
     *
     *******************************************************************************
     */
    virtual uint32_t getDivisions(std::vector<DBDivisionDetail_t> &divisions,
                                       DB_REF_MODE_E eRefMode = DB_REF_MODE_WHOLE_E) = 0;

    /**
     *******************************************************************************
     *  @brief getDivisions - Get all Divisions specified by input division IDs.
     *
     *  @param [in]  - divIDs   Divisions ID
     *
     *  @param [out]  - divisions   Division data
     *
     *  @param [in]  - eRefMode   Reference mode, which means the data organization
     *                  mode of references in a division
     *
     *  @ret uint32_t
     *
     *******************************************************************************
     */
    virtual uint32_t getDivisions(const std::set<DivisionID_t>& divIDs,
                                       std::vector<DBDivisionDetail_t> &divisions,
                                       DB_REF_MODE_E eRefMode = DB_REF_MODE_WHOLE_E) = 0;

    /**
     *******************************************************************************
     *  @brief setDivisions - Save Divisions info .
     *
     *  @param [out]  - divisions   Divisions need to be saved
     *
     *  @param [In]  - divisionChanges  DivisionChange_t info
     *
     *  @ret uint32_t
     *
     *******************************************************************************
     */
    virtual uint32_t setDivisions(std::vector<DBDivisionDetail_t> &divisions,
                                       const std::vector<DivisionChange_t> &divisionChanges) = 0;

    /**
     *******************************************************************************
     *  @brief getScheduleInput - Get schedule input param
     *
     *  @param [Out]  - workerNum   max concurrent Number
     *
     *  @param [Out]  - maxDivNum   max division number
     *
     *******************************************************************************
     */
    virtual void getScheduleInput(uint32_t &workerNum, uint32_t &maxDivNum) = 0;

    /**
     *******************************************************************************
     *  @brief setScheduleResult - Save schedule result
     *
     *  @param [In]  - scheduleInfo Schedule info of subtasks.
     *
     *  @ret uint32_t
     *
     *******************************************************************************
     */
    virtual uint32_t setScheduleResult(const ScheduleInfo_t &scheduleInfo) = 0;

    /**
     *******************************************************************************
     *  @brief setSamRtInfo - Save sam rt info(Just for RtInfoGenerator tool)
     *
     *  @param [In]  - rtInfos   rtInfos need to be saved
     *
     *  @ret uint32_t
     *
     *******************************************************************************
     */
    virtual uint32_t setSamRtInfo(const std::vector<SamRtInfo_t> &rtInfos) = 0;

    /**
     *******************************************************************************
     *  @brief getDivisionPath - Get division Path of the current task
     *
     *  @param [InOut]  - divisionPath   Division Path
     *
     *******************************************************************************
     */
    virtual void getDivisionPath(std::vector<DivisionID_t> &divisionPath) = 0;

};

}




#endif
