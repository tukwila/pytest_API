/**
 ************************************************************************************
 *                       RoadDB Confidential
 *       Copyright (c) Continental AG. 2017-2018, RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 ************************************************************************************
 * @file   RdbV2SSlam.h
 * @brief  Definitions of SLAM snippet
 ************************************************************************************
 */

#ifndef RDB_V2S_SLAM_H
#define RDB_V2S_SLAM_H

#include <vector>           // std::vector
#include <memory>           // std::shared_ptr
#include "typeDef.h"        // uint16 etc.
#include "RdbV2SGeometry.h"
#include "CommunicateDef/RdbV2SCommon.h"
#include "LogWrapper/LogWrapper.h" //log

// #include "algoInterface/LocRTMatrixImp.h"

namespace roadDBCore
{

struct LmPointBaseSR_t
{
    /* Code 2-byte vector size for vecObsFrmIdx , support up to 65536 frames*/
    std::vector<uint16_t> vecObsFrmIdx;

    /*
        Don't need to code vector size, the same vector size as vecObsFrmIdx
    */
    std::vector<Point2f_t> vec2DPoints;

    /*
        This field shares the same vector size with vecObsFrmIdx.
        Don't need to code vector size.
        Each index of descriptor size would be coded as 4 bits.
        Total bytes would be calculated as (4 * size + 7) / 8 bytes.
        Code from MSB to LSB
    */

    /*
        According to Lina's requirement, descriptor index need to
        be changed to coded as 1 byte in order to support 255 descriptors,
        instead of 4 bit mentioned above.
    */
    std::vector<uint8_t> vecDescID;

    /*
        Code 1-byte number of descriptor groups.
        Calculation: descriptor group number  = vector size / (descriptor size * descriptor number in a group)
        Currently, a group consists of 64 descriptors.
        Data coding from MSB to LSB
    */
    std::vector<uint8_t> vecDescriptors;

    /*
        The semantic context for every MP is stored as a 64 bit integer.
        Here we only store the top 5 classes together with their weights as well as the input variance.
        The weights and the input variance have a precision of 8 bit per value.
        We call this the 64bit-sparse representation of a semantic context.
        We showed that with 5 stored classes in the 64bit-sparse representation,
        the average information loss of the compression is minimal.
    */
    uint64_t semanticContext = 0LL;
};

struct Ref3DLmPointSR_t: public LmPointBaseSR_t
{
    uint32_t p3dID = 0; //the matched 3D point in database
};

struct Inc3DLmPointSR_t: public LmPointBaseSR_t
{
    Point3f_t position;
};

struct KeyFrameSR_t
{
    /* the original index of frame in rtv which generates this key frame */
    uint16_t     id = 0;

    bool         bKF = false;

    float32_t    rotation[9]{0};
    float32_t    translation[3]{0};

    Point3f_t    relativeGPS;

    uint32_t     refTimestamp = 0;

    /* The ratio of the coverage area of big vehicle in a frame */
    uint8_t      coverageRatio = 0;
};

struct SlamSnippetSR_t
{
    /* Code 2-byte vector size, supports up to 65535 key frames */
    std::vector<KeyFrameSR_t>               vecKeyFrames;

    /* Code 3-byte vector size, supports up to 65535 * 256 landmark points */
    std::vector<Inc3DLmPointSR_t>           vec3DPoints;   // new 3D points created by current vehicle option

    SlamSnippetSR_t() {}
};

struct SlamSnippetPayload_t: public PayloadBase_t
{
    ModelConfigSlam_t config;

    /* code 2 bytes vector size. */
    std::vector<std::shared_ptr<SlamSnippetSR_t>> vecSpSlamSnippets;

    SlamSnippetPayload_t(): PayloadBase_t(SNIPPET_PAYLOAD_TYPE_SLAM_E) {}
};


}


#endif //RDB_V2S_SLAM_H


