/**
 *******************************************************************************
 *                       RoadDB Confidential
 *       Copyright (c) Continental AG. 2016-2018, RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IVehicleDB.h
 * @brief  VehicleDB interface
 *******************************************************************************
 */

#ifndef IVEHICLE_DB_H_
#define IVEHICLE_DB_H_

#include <vector>
#include "typeDef.h"
#include "CommunicateDef/CommunicateDef.h"
#include "algoInterface/IVehicleTransfer.h"

//mock for the interface of vehicleDB refactoring
//When slam team starts refactoring, it should use struct VehicleReference_t in IVehicleTransfer.h
namespace roadDBCore
{

struct VehicleReference
{
    uint64_t                      dbID;

    /* Code 2-byte vector size for serialization, supports up to 65535 key frames */
    std::vector<VehicleKeyFrame_t>  vecKfs;   //serilization

    /* Code 3-byte vector size for serialization, supports up to 65535 * 256 landmark points */
    std::vector<VehicleMapPoint_t>  vecMps;   //serilization

    float32_t                     confidence;
    uint64_t                      version;
    uint32_t                      descriptorType;

    /* the detail batch info for specified batch.
       Key is batch ID.
    */
    std::map<uint64_t, ModelConfigServer_t>  mapBatchInfo;

    VehicleReference() : dbID(0),
                         confidence(0),
                         version(0),
                         descriptorType(0)
    {}
};
}
//end of mock

namespace algo
{

enum VehicleDBModeE
{
    VDB_MODE_ALIGNMENT,
    VDB_MODE_RT,
    VDB_MODE_MAX
};

struct VehicleDBSectionRange
{
    int32_t beginFrame;
    int32_t endFrame;
};

struct VehicleDBSectionDetail
{
    uint64_t sectionID;
    std::vector<VehicleDBSectionRange> sectionRange;
    std::vector<roadDBCore::VehicleReference> vecReference;// mock for the interface of vehicleDB refactoring
};

struct VehicleDBSectionSummary
{
    roadDBCore::Point3d_t refGps;
    std::list<uint64_t> sectionID;
};

class IVehicleDB
{
  public:
    virtual ~IVehicleDB(){};

    /**
     *******************************************************************************
     * @brief getMode - get mode of the algorithm,
     *
     *  <1> Parameter Description:
     *
     *  @param [Out]
     *         [In]
     *
     *  @return status - return mode of algorithm;
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    virtual VehicleDBModeE getMode() = 0;
    /**
     *******************************************************************************
     * @brief isDummySection - Judge whether the section is dummy section
     *  <1> Parameter Description:
     *
     *  @param [Out]
     *         [In] section ID
     *
     *  @return status - return true when the section is dummy section, otherwise return false;
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    virtual bool isDummySection(IN uint64_t sectionID) = 0;
    /**
     *******************************************************************************
     * @brief getSectionSummary - get the summary info of sections
     *  <1> Parameter Description:
     *
     *  @param [Out] section summary info
     *         [In]
     *
     *  @return status - return true or false;
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    virtual bool getSectionSummary(OUT VehicleDBSectionSummary &sectionSummary) = 0;
    /**
     *******************************************************************************
     * @brief getSectionDetail - get the detail info of sections
     *  <1> Parameter Description:
     *
     *  @param [Out] section detail info
     *         [In] section ID
     *
     *  @return status - return true or false;
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    virtual bool getSectionDetail(IN uint64_t sectionID, OUT VehicleDBSectionDetail &sectionDetail) = 0;
    /**
     *******************************************************************************
     * @brief getAllSectionDetail - get the detail info of all sections
     *  <1> Parameter Description:
     *
     *  @param [Out] all sections detail info
     *         [In]
     *
     *  @return status - return true or false;
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
     virtual bool getAllSectionsDetail(OUT std::vector<VehicleDBSectionDetail> &allSectionsDetail) = 0;
};

} // namespace algo

#endif
