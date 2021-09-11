/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   DataApi.h
 * @brief  The class definition of DataApi.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-04-15      Yunyun Wei        Init version.
 *
 *******************************************************************************
 */

#pragma once

#include "Geo.h"
// #include "VehicleAPIError.h"

#include "NDACache.h"
#include "VehicleAPICommon.h"
#include "VehicleAPICommon.h"
#include <vector>

typedef enum
{
    E_ROAD_MERGE = 0,
    E_ROAD_SPLIT = 1,
    E_LANE_MERGE = 2, /*one road: mult roads*/
    E_LANE_SPLIT = 3, /*mult roads: one road*/
    E_CROSS_INT = 4   /*cross intersection*/
} JunctionType;

typedef enum
{
    HRZN_STATUS_INVALID = -1,
    HRZN_STATUS_STATIC = 0,
    HRZN_STATUS_DYNAMIC,
    HRZN_STATUS_EARLY_TERM // early termination
} HorizonStatus;

// namespace roaddb {

// namespace geo {
// class Line;
// }

namespace RDBVehicleAPI {

class EVP;
class Lane;
class Junction;
class Landscape;
class RoadPath;
class HorizonEdge;
class HorizonVertex;
class DataApi
{
 public:
    /**
     * advancedMode:
     * True: More system resources used to obtain faster response time.
     * False: Less system resources used with response time sacrificed.
     */
    static uint32_t s_priority_;
    static uint32_t s_constantCpuMask_;
    static uint32_t s_floatCpuMask_;
    static const double PI;
    static const double TWO_PI;

    /**
     * @brief Determine whether the two evps are in the same direction
     * @param preEvp the pre evp to determine with
     * @param nextEvp the next evp to determine with
     * @return true the two evps are in the same direction, otherwise false
     */
    static bool isSameDirection(const std::shared_ptr<const EVP> &preEvp,
                                const std::shared_ptr<const EVP> &nextEvp);

    /**
     * @brief get the right most lane by the reference evp
     * @param refEvp the reference evp
     * @param rightMostLane the lane to be requested
     * @return errCode_t DB_VEHICLE_SUCCESS to get the lane successfully, otherwise failed.
     */
    static errCode_t getRightMostLane(const std::shared_ptr<const EVP> &refEvp,
                                      std::shared_ptr<const Lane> &rightMostLane);
    /**
     * @brief get the left line of right most lane by the reference evp
     * @param refEvp the reference evp
     * @param leftLine the line to be requested
     * @return errCode_t DB_VEHICLE_SUCCESS to get the line successfully, otherwise failed.
     */
    static errCode_t getLeftLineOfRightMostLane(const std::shared_ptr<const EVP> &refEvp,
                                                std::shared_ptr<const Line> &leftLine);
    /**
     * @brief get landscape length
     * @param refEVP the reference evp
     * @return the length of the landscape
     */
    static errCode_t getLandscapeLength(const std::shared_ptr<const EVP> &refEVP,
                                        float64_t &length);

    /**
     * @brief get junction by evp
     * @param evp the request evp
     * @return the reference junction of the evp
     */
    static const std::shared_ptr<const Junction> getJunctionByEvp(
        const std::shared_ptr<const EVP> &evp);

    /**
     * @brief get landscape by evp
     * @param evp the request evp
     * @return the reference landscape of the evp
     */
    static const std::shared_ptr<const Landscape> getLandscapeByEvp(
        const std::shared_ptr<const EVP> &evp);

    /**
     * @brief Calculate the length of the junction,
     * the junction is the lane split or lane merge
     *
     * @param junction the junction to calculate the length
     * @return the length of the junction
     */
    static errCode_t getJunctionLength(const std::shared_ptr<const Junction> &junction,
                                       float64_t &length);

    /**
     * @brief Calculate the distance from start point of the line,
     * note:we use the left line of the right most lane to calculate the distance
     *
     * @param evp the reference evp of the location
     * @param location the request location
     * @param length the result of the distance
     *
     * @return errCode_t DB_VEHICLE_SUCCESS calculate the distance successfully, otherwise failed.
     */
    static errCode_t getDistanceFromLineStart(const std::shared_ptr<const EVP> &refEvp,
                                              const NDSPoint_t &location, float64_t &length);

    /**
     * @brief check whether evp is in horizon edges or not.
     * @param evp the evp to be checked.
     * @param edges the horizon edges whether evp is in or not.
     * @return 0 evp is in horizon edges otherwise not.
     *
     * @ToDo this function will move to RoadPath.
     */
    static errCode_t checkEVPInEdges(const std::shared_ptr<const EVP> &evp,
                                     const std::vector<const HorizonEdge *> &edges);

    /**
     * @brief check whether evp is in horizon vertices or not.
     * @param evp the evp to be checked.
     * @param vertices the horizon vertices whether evp is in or not.
     * @return 0 evp is in horizon vertices otherwise not.
     *
     * @ToDo this function will move to RoadPath.
     */
    static errCode_t checkEVPInVertices(const std::shared_ptr<const EVP> &evp,
                                        const std::vector<const HorizonVertex *> &vertices);

    static bool setCPUAffinity(uint32_t cpuMask, pid_t pid = 0);
    static bool setThreadPriority(int priorityPolicy, int priority, pid_t pid = 0);
    static void split(const std::string &src, const std::string &delim, std::vector<std::string> *ret);

    static NDACache *cache();

    static bool loadDB(const char *path, EDBType dbType = E_DB_TYPE_PB, LOG_LEVEL logLevel = (LOG_LEVEL)enum_INFO);

 private:
    DataApi() = delete;
    DataApi(const DataApi &rhs) = delete;
    DataApi &operator=(const DataApi &rhs) = delete;

    static NDACache *cache_;
};

} /* namespace RDBVehicleAPI */
