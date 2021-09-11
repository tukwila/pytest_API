/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   PBRoadDataInfo.h
 * @brief  PBRoadDataInfo 
 *******************************************************************************
 */

#pragma once

#include "BaseRoadDataInfo.h"
#include "RDB_DB_ROADDB_SEGMENTDB.pb.h"
#include "RIVErrCode.h"

namespace RDBVehicleAPI
{
const std::string INIT_VERSION = "null";
class PBRoadDataInfo : public BaseRoadDataInfo
{
public:    
    PBRoadDataInfo(EXCEPTION_DATA_HANDLE_MODE mode = EDH_MODE_REFUSED) : BaseRoadDataInfo(mode){}
//     virtual ~PBRoadDataInfo();

public:
    bool load(segmentID_t id, const SP_CONST(DBDirectory) pDBDirectory) override;
	void destroy() override;
private:
    EDBType getDataType() const override {return E_DB_TYPE_PB;}
	virtual bool buildRoadInfo() override;
	virtual bool buildLaneInfo() override;
	virtual bool buildEvpConnect() override;
	virtual bool buildPaintRelation() override;
	virtual RIVErrCode_t getDataVersion(OUT std::string& majorVersion, OUT std::string& fixVersion) const override;
	RIVErrCode_t loadOneSegmentPB(const std::string& pbFilePath);
	RIVErrCode_t decodeSegmentData(const RDB_DB_ROADDB_SEGMENTDB::GeoAreaDatabase& dbMsg);
	RIVErrCode_t decodeVisualization(const ::google::protobuf::RepeatedPtrField< ::RDB_DB_ROADDB_SEGMENTDB::Visualization >&);
	RIVErrCode_t decodeRoad(const ::google::protobuf::RepeatedPtrField< ::RDB_DB_ROADDB_SEGMENTDB::Landscape >& roads);
	RIVErrCode_t decodeLaneInsideRoad(const RDB_DB_ROADDB_SEGMENTDB::Lane& pbLane, const SP(Landscape)& pRoad);
	RIVErrCode_t decodeLineInsideRoad(const RDB_DB_ROADDB_SEGMENTDB::Line& pbLine, 
		map<string, NurbsDesc>& nurbsInfo, map<string, int>& nurbsType);
	RIVErrCode_t decodeEVPInsideLane(const RDB_DB_ROADDB_SEGMENTDB::ExtractedVehiclePath& pbEvp, const SP(Lane)& pLane);
	RIVErrCode_t decodeEVPAttributeInsideEVP(const RDB_DB_ROADDB_SEGMENTDB::EvpAttribute& pbEvpAttribute, const SP(EVP)& pEvp);
	
	bool getRoadIDsByDivisionID(const divisionID_t divisionId,objectIDSet_t &roadIDs) const override;
	void getDivisionIDsByRoadIDs(const objectIDSet_t roadIDs, divisionIDSet_t &divisionIDs) const override;
	static RIVErrCode_t checkHeader(const RDB_DB_ROADDB_SEGMENTDB::Header& header, const std::string& segID);

private:
	WGS84_t basicPoint_;
	segmentID_t segId_ = 0;
	static std::string vehicleDataVersion_ ; //timestamp in vehicle db name generated in process
	static std::string fixVersion_;       	 // filled by delivery
};

}
