/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   LaneChange.h
 * @brief  The class definition of LaneChange.
 *
 * Change Log:
 * Date              Author            Changes
 * 2019-10-22      Yunyun Wei       Init version.
 *
 *******************************************************************************
 */

#pragma once

#include <memory>
#include <vector>
#include <map>
#include "Geo.h"
// #include "DataApi.h"
#include "jsoncpp/json/json.h"

namespace RDBVehicleAPI
{
class Lane;

class LaneChange : public Object, public std::enable_shared_from_this<LaneChange>
{
    friend class EVP;
    friend class NDACache;

 public:
    /**
     * @brief Get start point of the lane change.
     * @return The start Point of the lane change.
     */
    const WGS84_t& getStartPoint() const;

    /**
     * @brief Get start point of the lane change.
     * @return The start Point of the lane change.
     */
    const WGS84_t& getEndPoint() const;

    /**
     * @brief Get length of the lane change.
     * @return The length of the lane change.
     */
    float64_t getLength() const;

    /**
     * @brief check whether change to left lane is allowed
     * @return true if change is allowed, otherwise false
     */
    bool isLeftChangeAllowed() const;

    /**
     * @brief check whether change to left lane is allowed
     * @return true if change is allowed, otherwise false
     */
    bool isRightChangeAllowed() const;

    /**
     * @brief get source lane of the lane change
     * @return pointer of the source lane
     */
    const std::weak_ptr<const Lane> getSourceLane() const;

    /**
     * @brief get left destination lane of the lane change
     * @return if left change is allowed return pointer
     *  of the source lane, otherwise nullptr.
     */
    const std::weak_ptr<const Lane> getLeftDestLane() const;

    /**
     * @brief get right destination lane of the lane change
     * @return if right change is allowed return pointer
     *  of the source lane, otherwise nullptr.
     */
    const std::weak_ptr<const Lane> getRightDestLane() const;

 private:
    LaneChange() = delete;
    LaneChange(const LaneChange &obj) = delete;
    LaneChange &operator=(const LaneChange &obj) = delete;

    LaneChange(const objectID_t& id, const WGS84_t &startPoint, const WGS84_t &endPoint, const std::string &value);

    void parseAttribute(const std::string &value);
    void setSourceLane(const std::shared_ptr<const Lane> &lane);

 private:
    float64_t length_;
    bool leftChange_;
    bool rightChange_;
    std::weak_ptr<const Lane> srcLane_;
    std::weak_ptr<const Lane> leftDestLane_;
    std::weak_ptr<const Lane> rightDestLane_;
    WGS84_t startPoint_;
    WGS84_t endPoint_;
};

} 