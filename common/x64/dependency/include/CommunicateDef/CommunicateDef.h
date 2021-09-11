/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2016-2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   CommunicateDef.h
 * @brief  Defination of commuicate struct
 *******************************************************************************
 */


#ifndef _COMMUNICATE_DEF_H_
#define _COMMUNICATE_DEF_H_

#include <opencv2/imgproc.hpp>
#include "typeDef.h"
#include <vector>
#include <iomanip>

static const int32_t communicateVersion = 1;

namespace algo
{

    using roadDBCore::int32_t;
    using roadDBCore::float32_t;

    // 3D road model - consists of painting item indexes
    struct ST_ROAD_DriveLane
    {
        // start and end position
        int32_t    nStartKF;            // start keyframe Index,  vector index of slam data's keyframe set
        int32_t    nEndKF;                // end keyframe Index,  vector index of slam data's keyframe set
        int64_t    nLeftLP;                // current lane's left painting index (item's identity of ST_ROAD_ItemModel)
                                        // if < 0, means no left painting â€”â€” -1 : no detection,   -2 : detected but failed to be modeled

        int64_t    nRightLP;            // current lane's right painting's index (item's identity of ST_ROAD_ItemModel)
                                        // if < 0, means no right painting â€”â€” -1 : no detection,   -2 : detected but failed to be modeled

        ST_ROAD_DriveLane() : nStartKF(-1), nEndKF(-1), nLeftLP(-1), nRightLP(-1)
        {

        }

#ifdef _BOOST_SERIALIZE_

template<typename Archive>
void serialize(Archive& ar, unsigned int version);

#endif

    };

    typedef std::vector<ST_ROAD_DriveLane>  XhDriveLaneSet;

    /*relative gps position with reference postion*/
    struct GPSData
    {
        int timeStamp; //the msec passed since from one special time.

        float lon;  //distance in meter from reference position, axis towards East
        float lat;  //distance in meter from reference position, axis towards North
        float alt;  //distance in meter from reference position, axis toward up

        GPSData() : timeStamp(0), lon(0.0f), lat(0.0f), alt(0.0f) {}
    };

    struct KeyFrameData
    {
        // pose
        cv::Mat r;
        cv::Mat t;

        // GPS data
        GPSData gps;

        KeyFrameData() : r(), t(), gps() {}
    };

    struct IntrinsicData
    {
        float fx, fy, cx, cy;

        IntrinsicData() : fx(0.0f), fy(0.0f), cx(0.0f), cy(0.0f) {}
    };

    struct ROI
    {
        int32_t   frmIdx; //from 0
        cv::Rect  rect;  //the location where the sign detected in the image frame
        roadDBCore::float32_t    detConf;
    };

    struct SSim3TransForm
    {
        float scale;
        cv::Mat rotation;
        cv::Mat translation;
    };

    struct SPieceAttribute
    {
        int countMissMatch; // times of miss matching
        int countGetMatch; // times of getting matched
        float confidence; // confidence score
        float feedbackByEvaluate; // score feedback from Evaluate module
        float feedbackByMatch; // score feedback from FindOverlap module
        float feedbackByJudge; // score feedback from JudgeOverlap module
        float feedbackByConnect; // score feedback from Connect Section module
        float feedbackByRMerge; // score feedback from RoadMerge module
        SPieceAttribute() :
            countMissMatch(0),
            countGetMatch(0),
            confidence(0.0f),
            feedbackByEvaluate(0.0f),
            feedbackByMatch(0.0f),
            feedbackByJudge(0.0f),
            feedbackByConnect(0.0f),
            feedbackByRMerge(0.0f)
        {}
    };

    // One 3D point
    struct ST_ROAD_Point3D
    {
        float   x;
        float   y;
        float   z;

        ST_ROAD_Point3D() : x(-1), y(-1), z(-1)
        {
        }

        ST_ROAD_Point3D(float a, float b, float c)
        {
            x = a;
            y = b;
            z = c;
        }
#ifdef _BOOST_SERIALIZE_

template<typename Archive>
void serialize(Archive& ar, unsigned int version);

#endif
    };

//#ifdef ENABLE_NURBS

    /**
     * Enumeration of line types for NURBS
     */
    enum class NURBS_LINE_TYPE_E : uint8_t
    {
        NURBS_LINE_TYPE_SOLID_E,
        NURBS_LINE_TYPE_DASHED_E
    };

    // Parameters of NURBS fitted curve
    struct NurbsCurveParam
    {
        NurbsCurveParam(const NURBS_LINE_TYPE_E lineType = NURBS_LINE_TYPE_E::NURBS_LINE_TYPE_SOLID_E,
                        const double length = 0,
                        const std::vector<ST_ROAD_Point3D> &ctrlPoints = std::vector<ST_ROAD_Point3D>(),
                        const std::vector<double> &knots = std::vector<double>(),
                        const std::vector<double> &endPoints = std::vector<double>())
         : lineType(lineType),
           length(length),
           ctrlPoints(ctrlPoints),
           knots(knots),
           endPoints(endPoints)
        {
        }

#ifdef _BOOST_SERIALIZE_
        template<typename Archive>
        void serialize(Archive& ar, unsigned int version)
        {
            ar & static_cast<uint8_t>(lineType)
               & length
               & ctrlPoints
               & knots
               & endPoints;
        }
#endif

        NURBS_LINE_TYPE_E             lineType;     // Type of the curve
        double                        length;       // The length of the curve
        std::vector<ST_ROAD_Point3D>  ctrlPoints;   // Control points
        std::vector<double>           knots;
        std::vector<double>           endPoints;    // The end points of the segments, for dashed curve only
    };

    // Parameters of NURBS fitting method
    struct ST_ROAD_NurbsParam
    {
        ST_ROAD_NurbsParam(const NurbsCurveParam &curveParam = NurbsCurveParam{},
                           const int32_t startKFID = -1,
                           const int32_t endKFID = -1)
            : curveParam(curveParam)
            , nStartKF(startKFID)
            , nEndKF(endKFID)
        {
        }

#ifdef _BOOST_SERIALIZE_
        template<typename Archive>
        void serialize(Archive& ar, unsigned int version)
        {
            ar & curveParam & nEndKF & nStartKF;
        }
#endif

        NurbsCurveParam  curveParam; // fitted NURBS curve parameter
        int32_t          nStartKF;   // id of start keyframe
        int32_t          nEndKF;     // id of end keyframe

    };
//#endif

    // Line parameter, use polynomial to represent any 2D or 3D curve
    struct ST_ROAD_LineParam
    {
        int      nInputType;          // independent variable type
                                                  // value 0 - x,  value  1 - y,  value 2 - z,
                                                  // value 3 - x and y, value 4 - x and z, value 5 - y and z,
                                                  // value 6 - y and x, value 7 - z and x, value 8 - z and y,
        int      nOutputType;         // dependent variable
                                                  // value 0 - x,  value 1 - y,  value 2 - z
                                                  // If nOutputType = 2 , nInputType = 3, then the function can be represented as  z = f(x,y)
        int      nVariable;           // independent variable number.
                                                  // eg:  a * a + a + b = c, then nVariable = 2
                                                  // a * a + 2 * a = c, then nVariable = 1
        int      nMaxPower;           // maximum power.  eg:  a * a + a + b = c, then nMaxPower = 2.
                                                  // if nVarible = 1,  polynomial item number = nMaxPower + 1
                                                  // if nVarible = 2,  polynomial item number = (nMaxPower + 1) * (nMaxPower + 2) / 2
        std::vector<double>  coef;    // coefficients for each polynomial item. vector size = polynomial item number
                                                  // sorted by descending power.
                                                  // eg: if nMaxPower = 3,  nVariable = 2,  then polynomial item number = 4 * 5 /  2 = 10
                                                  //     c =  coef[0] * a^3 + coef[1] * a^2 * b + coef[2] * a * b^2 + coef[3] * b^3
                                                  //            + coef[4] * a^2 + coef[5] * a * b + coef[6] * b^2
                                                  //            + coef[7] * a + coef[8] * b + coef[9]

        ST_ROAD_LineParam() :
          nInputType(0),
		  nOutputType(0),
          nVariable(1),
		  nMaxPower(2)

        {

        }

#ifdef _BOOST_SERIALIZE_

template<typename Archive>
void serialize(Archive& ar, unsigned int version);

#endif

        void CopyIn(const ST_ROAD_LineParam* pstIn);
    };

    // item id on road
    enum ROAD_ITEM
    {
        ROAD_ITEM_PAINTING      = 0X1000,         // all kinds of paintings
        ROAD_ITEM_PAINTING_BD,                    // all kinds of lane boundary paintings
        ROAD_ITEM_PAINTING_BD_DASHEDWHITE,        // lane boundary, dashed white line
        ROAD_ITEM_PAINTING_BD_DASHEDYELLOW,       // lane boundary, dashed yellow line
        ROAD_ITEM_PAINTING_BD_SOLIDWHITE,         // lane boundary, solid white line
        ROAD_ITEM_PAINTING_BD_SOLIDYELLOW,        // lane boundary, solid yellow line
        ROAD_ITEM_PAINTING_BD_SOLIDYELLOW_DUAL,   // lane boundary, dual solid yellow line
        ROAD_ITEM_PAINTING_BD_IMPUTED,            // lane boundary, imputed or faked line
        ROAD_ITEM_PAINTING_GUIDE,                 // all kinds of guide paintings
        ROAD_ITEM_PAINTING_GUIDE_ARROW,           // turning direction arrow
        ROAD_ITEM_PAINTING_GUIDE_LEFT,            // left-turn guide line
        ROAD_ITEM_PAINTING_GUIDE_ZEBRA,           // zebra guiding line set for vehicle moving
        ROAD_ITEM_PAINTING_GUIDE_RHOMBUS,         // rhombus painting: reminds slowing done
        ROAD_ITEM_PAINTING_CROSSROAD,             // all kinds of paintings at cross road
        ROAD_ITEM_PAINTING_CROSS_GRID,            // yellow grid line: forbids stop
        ROAD_ITEM_PAINTING_CROSS_STOP,            // horizontal stop white line
        ROAD_ITEM_PAINTING_CROSS_ZEBRA,           // zebra paintings for walking through
        ROAD_ITEM_PAINTING_VEHICLE,               // pattern painting : lane for specified vehicle
        ROAD_ITEM_PAINTING_VELOCITY,              // number painting : maximum / minimum velocity
        ROAD_ITEM_PAINTING_CHAR,                  // all character painting
        ROAD_ITEM_SIGN         = 0X1100,          // all kinds of traffic signs
        ROAD_ITEM_SURFACE      = 0X1200,          // all kinds of items on road surface except paintings
        ROAD_EDGE_PAVEMENT     = 0X1300,          // pavement edge
        ROAD_EDGE_CURB  ,                         // curb edge
        ROAD_EDGE_JERSEYWALL,                     // jerseywall
        ROAD_EDGE_GUARDRAIL                       // guard rail
        // to be extended...
    };

    // Line Structure, now most kinds of paintings can be represented by this structure set
    struct ST_ROAD_LineModel
    {
        ROAD_ITEM                        nSubItemType;        // sub item type in piece
        int                              nLineWidth;          // line width in pixel
        float                            fReliablity;         //[0,1]
        std::vector<ST_ROAD_Point3D>     vecPtControl;        // control points. when reconstruction, sampling between two adjacent control points
                                                                     // full solid line has two pt, while dashed or broken solid line has more
        std::vector<ST_ROAD_LineParam>   vecParam;            // equation parameter
                                                              // vector size: how many line equations to represent a single line
        int     nStartKF;                                        // start keyframe Index,  vector index of slam data's keyframe set
        int     nEndKF;                                            // end keyframe Index,  vector index of slam data's keyframe set

        ST_ROAD_LineModel() : nSubItemType(ROAD_ITEM_PAINTING), nLineWidth(1), fReliablity(1.0), nStartKF(0), nEndKF(0)
        {

        }

#ifdef _BOOST_SERIALIZE_

        template<typename Archive>
        void serialize(Archive& ar, unsigned int version);

#endif


        void CopyIn(const ST_ROAD_LineModel* pstIn);
    };

    // 3D item model - consists of line pieces
    struct ST_ROAD_ItemModel
    {
        int64_t     nID;                          // local identity from vehicle from one report(it will start from 0 for different reports)
        ROAD_ITEM   nItemType;                    // item type
        float       fScale;                       // the ratio between real measured value(meter) and coordinate value,  if coordinate value 3 pixel means 4.5m, then fScare = 1.5f;
        float       fResolution;                  // minimum points distance in the reconstructed world(meter)
        roadDBCore::SegmentID_t segmentID_;

#ifdef ENABLE_NURBS
        std::vector<ST_ROAD_NurbsParam>  vecPiecesNurbs;    // NURBS parameters
#else
        std::vector<ST_ROAD_LineModel>   vecPieceSet;       // small pieces of an item
#endif

        ST_ROAD_ItemModel() : nID(-1), nItemType(ROAD_ITEM_PAINTING), fScale(1.0f), fResolution(0.1f), segmentID_(-1)
        {

        }

#ifdef _BOOST_SERIALIZE_

        template<typename Archive>
        void serialize(Archive& ar, unsigned int version);

#endif


        void CopyIn(const ST_ROAD_ItemModel* pstIn);
    };
}


namespace roadDBCore
{
    typedef Point2D<int32_t> Pnt2i;
    typedef Point3D<int32_t> Pnt3i;

    struct PBMatAr
    {
        PBMatAr()
            : mat_type_(0)
            , mat_rows_(0)
            , mat_cols_(0)
            , mat_elem_size_(0) {}

        int32_t mat_type_;
        uint32_t mat_rows_;
        uint32_t mat_cols_;
        uint32_t mat_elem_size_;
        std::vector<uint8_t> mat_data_;

        bool operator==(const PBMatAr& other);
        bool operator!=(const PBMatAr& other);

    };

    struct AlgoOutputHead_t
    {

        AlgoOutputHead_t()
        : type(0)
        , majorVer(0)
        , minorVer(0)
        , revisionVer(1)
        , refTimeStamp(0)
        , refGpsLon(0.0F)
        , refGpsLat(0.0F)
        , refGpsAlt(0.0F) {}

        uint8_t type;       //ALGO_OUTPUT_TYPE_E
        uint8_t majorVer;   //major version
        uint8_t minorVer;   //minor version
        uint8_t revisionVer; //revision version
        uint64_t refTimeStamp; //reference time stamp
        float64_t refGpsLon;   //reference GPS lon
        float64_t refGpsLat;   //reference GPS lat.
        float64_t refGpsAlt;   //reference GPS alt
        std::string vehicleID;  //vehicle id

        void debugString(std::stringstream &ss);
    };

    //Key Frame Archive Format
    struct KeyFrameAr
    {
        KeyFrameAr():timeStamp_(0)
        {
            //keyPoints_.clear();
            memset(rotation_, 0, sizeof(rotation_));
            memset(translation_, 0, sizeof(translation_));
            position_.x = 0;
            position_.y = 0;
            position_.z = 0;
        }

        //std::vector<KeyPointAr> keyPoints_;
        float32_t    rotation_[9];
        float32_t    translation_[3];
        Pnt3i        position_;
        int32_t        timeStamp_;

        void debugString(std::stringstream &ss, bool bDetail = true);
        bool operator==(const KeyFrameAr& other);
        bool operator!=(const KeyFrameAr& other);
    };


    //Key Point Archive Format
    struct KeyPointAr
    {
        KeyPointAr():orientationDes_(0.0f),scaleLevel_(0)
        {
            coordinate_.x = 0;
            coordinate_.y = 0;
        }

        roadDBCore::Pnt2i coordinate_;
        float32_t orientationDes_;    //orientation descriptor
        int32_t   scaleLevel_;

        void debugString(std::stringstream &ss);
        bool operator==(const KeyPointAr& other);
        bool operator!=(const KeyPointAr& other);
    };

    struct KeyFrameIdxAr
    {
        KeyFrameIdxAr() : index_(0), kp_(), descriptor_()
        {

        }

        uint32_t index_;
        KeyPointAr kp_;
        PBMatAr descriptor_;
        //cv::Mat     descriptor_;

        void debugString(std::stringstream &ss, bool bDetail = true);
        bool operator==(const KeyFrameIdxAr& other);
        bool operator!=(const KeyFrameIdxAr& other);
    };


    struct MapPointAr
    {
        MapPointAr() : descriptor_()
        {
            memset(pos_, 0, sizeof(pos_));
            //memset(descriptor_, 0, sizeof(descriptor_));
        }

        std::vector<KeyFrameIdxAr> observers_;
        float32_t pos_[3];
        //uint8_t    descriptor_[32];
        PBMatAr descriptor_;

        void debugString(std::stringstream &ss, bool bDetail = true);
        bool operator==(const MapPointAr& other);
        bool operator!=(const MapPointAr& other);
    };

    //Map Archive Format
    struct SlamDataAr
    {
        SlamDataAr() : fx_(0.0f), fy_(0.0f), cx_(0.0f), cy_(0.0f), refGps_()
        {
            kfs_.clear();
        }

        float32_t fx_;
        float32_t fy_;
        float32_t cx_;    //optical center in x
        float32_t cy_;    //optical center in y
        cv::Point3d refGps_;
        std::vector<KeyFrameAr> kfs_;
        std::vector<MapPointAr> mps_;

        void debugString(std::stringstream &ss, bool bDetail = true);
        bool operator==(const SlamDataAr& other);
        bool operator!=(const SlamDataAr& other);
    };

    struct TrafficSignAr
    {
        TrafficSignAr() : type_(0), orientation_(0.0F), shapeWidth_(0), shapeHeight_(0), confidence_(0.0F),
                startKFIdx_(0), endKFIdx_(0) {}

        int32_t type_;
        float32_t     orientation_;
        uint32_t     shapeWidth_;
        uint32_t    shapeHeight_;
        float32_t    confidence_;
        Pnt3i         position_;
        int32_t        startKFIdx_;
        int32_t     endKFIdx_;

        void debugString(std::stringstream &ss, bool bDetail = true);
        bool operator==(const TrafficSignAr& other);
        bool operator!=(const TrafficSignAr& other);
    };

    //Equation Param Archive Format
    struct EquationParamAr
    {
        EquationParamAr() : independentVar_(0), dependentVar_(0), independentVarCnt_(0), polynomialDegree_(0)
        {
            coefs_.clear();
        }

        uint32_t    independentVar_;
        uint32_t    dependentVar_;
        int32_t        independentVarCnt_;
        int32_t        polynomialDegree_;
        std::vector<float64_t> coefs_;

        void debugString(std::stringstream &ss);
        bool operator==(const EquationParamAr &other);
        bool operator!=(const EquationParamAr &other);
    };

    //Line Mode Archive Format
    struct LineModeAr
    {
        LineModeAr() : type_(0), lineWidth_(0),startKF_(0),endKF_(0)
        {
            ctrlPoints_.clear();
            equaions_.clear();
        }

        uint32_t type_;
        int32_t lineWidth_;
        int32_t startKF_;
        int32_t endKF_;
        std::vector<Pnt3i > ctrlPoints_;
        std::vector<EquationParamAr> equaions_;

        void debugString(std::stringstream &ss);
        bool operator==(const LineModeAr& other);
        bool operator!= (const LineModeAr &other);
    };

    //Item Mode Archive Format
    struct ItemModelAr
    {
        ItemModelAr() : Id_(0), type_(0), scaleFactor_(0.0F), resolution_(0.0F)
        {
            lineModels_.clear();
        }

        int64_t Id_;
        uint32_t type_;
        float32_t scaleFactor_;
        float32_t resolution_;
        std::vector<LineModeAr> lineModels_;

        void debugString(std::stringstream &ss, bool bDetail = true);
        bool operator==(const ItemModelAr& other);
        bool operator!= (const ItemModelAr& other);
    };

    typedef std::vector<ItemModelAr> vecItemModel;

    struct RoadGeometryAr_t
    {
        vecItemModel itemModels_;
        algo::XhDriveLaneSet lanes_;
    };

}

#endif
