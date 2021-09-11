/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   NDACache.h
 * @brief  This is an abstract class designed for Near Driving Area Data Cache.
 *         Near Driving Area is the near area around a vehicle within a certain
 *         distance (e.g. 30 km) that is concerned by the vehicle for driving.
 *         Data Cache is a memory cache which mirrors a part (or all) of data
 *         stored in a data source. It is aimed to improve the data access
 *         performance.
 *         Data Source is a resource from which the data can be retrieved. It
 *         could be in different formats, e.g. a file (txt/xml/binary etc.), a
 *         database connection (e.g. ODBC),  a URL, or even a socket connection.
 *         Note a cache may be dynamically refreshed or swapped from the data
 *         source at runtime.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-03-18        Tony Xiong        Init version.
 *
 *******************************************************************************
 */

#pragma once

#include <memory>
#include <map>

#include "Geo.h"
#include "VehicleAPICommon.h"
#include "logHelper.h"
#include "RIVLogicMgr.h"

namespace RDBVehicleAPI
{
class EVP;
class Lane;
class LaneConnection;
class Landscape;
class Junction;
class TrafficSign;
class Line;
class Curve;
class NURBSCurve;
class Line;

/**
 * @brief  This is an abstract class designed for Near Driving Area Data Cache.
 *         Near Driving Area is the near area around a vehicle within a certain
 *         distance (e.g. 30 km) that is concerned by the vehicle for driving.
 *         Data Cache is a memory cache which mirrors a part (or all) of data
 *         stored in a data source. It is aimed to improve the data access
 *         performance.
 *         Data Source is a resource from which the data can be retrieved. It
 *         could be in different formats, e.g. a file (txt/xml/binary etc.), a
 *         database connection (e.g. ODBC),  a URL, or even a socket connection.
 *         Note a cache may be dynamically refreshed or swapped from the data
 *         source at runtime.
 */
class NDACache
{
 public:
    friend class RIVLogicMgr;
    static uint16_t const MAX_LOAD_RADIUS = 30;

    /**
     * This error code will be returned when
     * load method is invoked when the data
     * has already been loaded into the cache.
     *
     * In this situation, call unload() and
     * then call the load method will resolve
     * this problem.
     */
    static errCode_t const ERR_LOADED = 999;

    /**
     * SUCCESS code
     */
    static errCode_t const SUCCESS = 0;

 
    /**
     * This error code will be returned when
     * an invalid location is given in
     * load(location, radius).
     *
     * Note the valid location value is between:
     * lon: -180.0 ~ 180.0
     * lat: -90.0 ~ 90.0
     * alt: about -20000.0 ~ 10000.0
     */
    static errCode_t const ERR_INVALID_LOCATION = 901;

    /**
     * This error code will be returned when
     * an invalid radius is given in
     * load(location, radius).
     *
     * Note:
     * 0 < the valid radius value <= MAX_LOAD_RADIUS
     */
    static errCode_t const ERR_INVALID_RADIUS = 902;

    /**
     * This error code will be returned when
     * No data can be found within the  area
     * given in load(location, radius).
     */
    static errCode_t const ERR_NOT_IN_RANGE = 903;

    // static errCode_t const ERR_OBJ_ALREADY_EXISTS = 200;

    /**
     * @brief Construct a new NDACache object.
     *
     * @param path:dataSource the data source from which data will be loaded into this
     *        object.
     * @param logLevel:enum_DEBUG = 1, enum_INFO = 2,enum_WARN = 3,
     *                 enum_ERROR = 4, enum_LOGOFF = 5
     */
     NDACache(const char* path);

    /**
     * @brief Destroy the NDACache object.
     *
     */
     ~NDACache();

    /**
     * @brief Load data into memory.
     * @return error code
     *
     * Note:
     * This method is supposed to be implemented as a smart load (e.g. load
     * data based on memory, cache history, user preference etc.).
     * But currently this method will load all data from the database.
     */
    errCode_t load(EDBType dbType = E_DB_TYPE_PB, LOG_LEVEL logLevel = (LOG_LEVEL)enum_INFO);

    /**
     * @brief Load data within a given area into memory.
     * @param location The center of the area
     * @param radius The radius of the area (unit: kilometer)
     * @return error code
     *
     * Notes:
     * This method will only add data into the cache, but won't remove any
     * data from the cache.
     * If the radius is larger than MAX_LOAD_RADIUS, then ERR_INVALID_RADIUS
     * will be returned.
     * Because of the road network connectivity, a road element may go beyond
     * the boundary of the given circle. This method guarrantees that if any part
     * of a road element falls into the area, it will be loaded into the cache
     * as a whole.
     * Also note that the area loaded, due to implementation complexity, may not
     * be an exact circle but something bigger (e.g. a circumscribed square),
     * depending on the implementation. But in all cases the method shall
     * guarrantee all road elements within the circle area be loaded into the
     * cache.
     */
    errCode_t load(const WGS84_t& location, uint16_t radius, EDBType dbType = E_DB_TYPE_PB);

    /**
     * @brief Unload all data loaded (i.e. remove all data from the cache).
     *        This method shall be called first in the override method unload().
     * @return 0 when success, otherwise the error code.
     */
    errCode_t unload();

    //  /**
    //   * @brief Refresh the given area in this cache.
    //   *
    //   * @param center The center of the area that are to be refreshed
    //   * @param radius The radius of the area that are to be refreshed
    //   * @return Error code
    //   *
    //   * Notes:
    //   * This method may add load data into cache and may also remove data
    //   * from the cache.
    //   * This method garrantees that all data within the given area
    //   * are loaded into the cache.
    //   */
    //  virtual errCode_t refresh(const NDSPoint_t& center,
    //                            uint16_t radius) = 0;

 public:
    /**
     * @brief Get landscape object by id.
     * @param id the id of the landscape
     * @return The pointer to the landscape, or null if no such landscape.
     */
    std::shared_ptr<const Landscape> getLandscapeByID(
        objectID_t id) const;

    /**
     * @brief Get junction object by id.
     * @param id the id of the junction
     * @return The pointer to the junction, or null if no such junction.
     */
    std::shared_ptr<const Junction> getJunctionByID(
        objectID_t id) const;

    /**
     * @brief Get lane object by id.
     * @param id the id of the lane
     * @return The pointer to the lane, or null if no such lane.
     */
    std::shared_ptr<const Lane> getLaneByID(objectID_t id) const;

    /**
     * @brief Get lane boundary object by id.
     * @param id the id of the lane boundary
     * @return The pointer to the lane boundary, or null if no such lane boundary.
     */
    std::shared_ptr<const Line> getLaneBoundaryByID(
        objectID_t id) const;

    /**
     * @brief Get EVP object by id.
     * @param id the id of the EVP
     * @return The pointer to the EVP, or null if no such EVP.
     */
    std::shared_ptr<const EVP> getEVPByID(objectID_t id) const;

    /**
     * @brief Get line object by id.
     * @param id the id of the line
     * @return The pointer to the line, or null if no such line.
     */
    std::shared_ptr<const Line> getLineByID(objectID_t id) const;

    /**
     * @brief Get curve object by id.
     * @param id the id of the curve
     * @return The pointer to the curve, or null if no such curve.
     */
    std::shared_ptr<const Curve> getCurveByID(objectID_t id) const;

    /**
     * @brief Get traffic sign object by id.
     * @param id the id of the traffic sign
     * @return The pointer to the traffic sign, or null if no such traffic sign.
     */
    std::shared_ptr<const TrafficSign> getTrafficSignByID(
        objectID_t id) const;

    /**
     * @brief Get speed limit change object by id.
     * @param id the id of the speed limit change object
     * @return The pointer to the speed limit change, or null if no such speed limit change.
     */
    //virtual std::shared_ptr<const SpeedLimitChange>
    //getSpeedLimitChangeByID(objectID_t id) const;

    /**
     * @brief Get the lane in which the given position (NDS format) is located.
     * @param position the given position
     * @return The pointer to the lane in which the given position (NDS format) is located,
     *         or null if the position is not in any lane.
     */
    std::shared_ptr<const Lane> getLane( const NDSPoint_t& position, Pose_t * const pose = nullptr) const;



    /**
     * @brief Get the evp in which the given position (NDS format) and pose is located.
     * @param position the given position
     * @param pose the given pose
     * @return The pointer to the EVP in which the given position (NDS format) and pose is located,
     *         or null if the position is not in any lane.
     */
    // std::shared_ptr<const EVP> getEVP(const NDSPoint_t& location,
    //                                           const pose_t& pose) const;

private:
    NDACache() = delete;
    NDACache(const NDACache& obj) = delete;
    NDACache& operator=(const NDACache& obj) = delete;

    NDACache(const char* path, bool autoLoad);  // For ut only.

 private:
    errCode_t doLoad(const WGS84_t& location, uint16_t radius,
                     bool loadAll = false);
 private:
    std::string path_;
    conf_t conf_;
    RIVAPI::RIVLogicMgr logicMgr_;                                
    LOG_LEVEL logLvl_ = enum_INFO;
};

} // namespace com
