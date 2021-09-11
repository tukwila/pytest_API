/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   SpeedLimit.h
 * @brief  The class definition of SpeedLimit.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-10-08      Yunyun Wei       Init version.
 *
 *******************************************************************************
 */

#pragma once

#include <memory>
#include <vector>
#include <map>
#include "Geo.h"
#include "jsoncpp/json/json.h"

namespace RDBVehicleAPI
{
class Lane;


typedef enum
{
    UNKNOWN = 0, /* unknown */
    KM_H = 1,    /* km/h */
    K_S = 2,     /* m/s */
    MPH = 3,     /* mph */
} SpeedUnit;

class SpeedLimit : public std::enable_shared_from_this<SpeedLimit>
{
    friend class EVP;
    friend class NDACache;
    friend class GetLaneByIDResponseMsg;
 public:
    ~SpeedLimit();
    /**
     * @brief Get start point of the speed limit.
     * @return The start Point of the speed limit.
     */
    const WGS84_t &startPoint() const;

    /**
     * @brief Get start point of the speed limit.
     * @return The start Point of the speed limit.
     */
    const WGS84_t &endPoint() const;

    /**
     * @brief Get length of the speed limit.
     * @return The length of the speed limit.
     */
    float64_t getLength() const;

    /**
     * @brief Get value of the speed limit.
     * @return The value of the speed limit. -1 if no traffic sign found
     */
    int32_t getValue() const;

    /**
     * @brief Get unit of the speed limit.
     * @return The unit of the speed limit.
     */
    SpeedUnit getUnit() const;


 private:
    SpeedLimit(){};
    SpeedLimit(const SpeedLimit &obj) = delete;
    SpeedLimit &operator=(const SpeedLimit &obj) = delete;

    SpeedLimit(const WGS84_t &startPoint, const WGS84_t &endPoint,
               const std::string &value);

    void parseAttribute(const std::string &value);
    const objectID_t getDBID() const;

   void setAttributeObject(const std::shared_ptr<Object>& pObject);

 private:
    WGS84_t startPoint_;
    WGS84_t endPoint_;
    float64_t length_ = 0.0;

    int32_t value_ = -1; // objtype may can't be found

    SpeedUnit unit_ = UNKNOWN;
    objectID_t dbId_ = "";
    // int objType_ = 0;
    std::shared_ptr<TrafficSign> pTS_;

    FRIEND_2_ROADDATAINFO;
};

} 