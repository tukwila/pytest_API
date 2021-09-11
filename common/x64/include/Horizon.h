/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Horizon.h
 * @brief  The class of horizon, in accordance with ADASIS v3 Spec.
 *
 * Change Log:
 * Date              Author            Changes
 * 2020-03-01        Lindun Tang       Init version.
 *
 *******************************************************************************
 */

#pragma once

#include "Path.h"
#include "Geo.h"
#include "Lane.h"
#include "Landscape.h"
#include "VehicleAPICommon.h"
#include <mutex>
#include <condition_variable>

namespace RDBVehicleAPI
{
/**
 * @brief  The class definition for objects.
 */
const float64_t ROOT_PATH_LENGTH_THRESH = 1.0;

struct HorizonPosition
{
    uint16_t pathId = 0;
    uint16_t offset = 0;
    uint16_t accuracy = 0;         // to-do
    uint16_t deviation = 0;        // to-do
    float64_t speed = 0.0;           // to-do
    float64_t relativeHeading = 0.0; // to-do
    float64_t probability = 0.0;     // to-do
    uint8_t currentLane = 0;       // to-do
};

struct autoUpdateStrategy
{
    bool autoUpdate = false;
    bool byIntersection = false; // auto update horizon every time pass a intersection
    bool byTime = false;         // update horizon periodically, in the unit of seconds
    bool byDistance = false;     // auto update horizon after a specific distance has passed, in the unit of meter
    uint32_t updatePeriod;       // horizon update period
    uint32_t updateDistance;     // horizon update distance threshold
};

class Horizon
{

public:
    /**
	 * the checking period for auto update (Value: 200 mini second).
	 */
    static const uint32_t AUTO_UPDATE_CHECK_PERIOD;

    ~Horizon();

    /**
     * @brief  Horizon Constructor.
     * @param position, the vehicle position
     * @param distance, the maximum path length in horizon
     * @param onlyMainPath, only contain main path in horizon or contain all paths
     * @param aus, the strategy for auto update
     * 
     * Notes:
     * the given position is on the main path, the start point of main path is always at the start point of some landscape
     * so, there maybe a gap between main path start point and the given vehicle point
     */
    Horizon(NDSPoint_t position, std::shared_ptr<const Lane> lane, uint16_t distance = 2000, bool onlyMainPath = false, autoUpdateStrategy aus = {});

    /**
     * @brief Given a position, find the path that contain this position.
     * @param position, the given position
     * @return a path contain this position or null ptr if given position is not in current horizon
     */
    std::shared_ptr<const Path> getPathByPos(NDSPoint_t position);

    /**
     * @brief Get all the possible horizon position according to localization output
     * @param position, the position
     * @param radisu, the accuracy of input position
     * @return a vector contain all the position horizon position of ego vehicle
     */
    void getHorizonPos(NDSPoint_t position, std::vector<HorizonPosition> &hpVec, uint16_t radius = 0);

    /**
     * @brief Get all the path in current horizon
     * @return a vector contain all the path
     */
    std::vector<std::shared_ptr<const Path>> &getAllPaths() { return allPaths_; }

    /**
     * @brief Get the main path
     * @return main path
     */
    std::shared_ptr<const Path> getMainPath() { return mainPath_; }

    /**
     * @brief Get a path by path ID
     * @return pointer to the path with the given ID
     */
    std::shared_ptr<const Path> getPathByID(uint16_t pathID);

    /**
     * @brief Update current horizon
     * @param pathID, the path which the vehicle is currently on
     * @param offset, the offset of ego vehicle
     * @return 0 if update success, 1 if fail
     */
    errCode_t update(uint16_t pathID, uint32_t offset);

    /**
     * @brief get the auto update flag of this horizon
     * @return true if autoupdate enabled, false if disabled
     */
    const autoUpdateStrategy &getAutoUpdateStratege() { return aus_; }

    /**
     * @brief disable auto update
     * @return none
     */
    void disableAutoUpdate();

    /**
     * @brief get the first intersection along the direction of ego vehicle driving direction
     * @param position, the position of ego vehicle
     * @param dis, the ditance to next intersection 
     * @return true if found the point, else return false
     */
    bool getDistanceToNextIntersection(NDSPoint_t position, double &dis);

    uint32_t getID() { return id_; }

    uint16_t getLength() { return horizonLength_; }

    uint32_t getRootPathID() { return mainPath_->getID(); }

    EHORIZON_DETAIL_LEVEL getDetailLevel() { return detailLevel_; }
    void setDetailLevel(const EHORIZON_DETAIL_LEVEL lvl) { detailLevel_ = lvl; }
private:
    uint32_t getNewPathID() { return ++maxPathID_; };
    float64_t getLandscapeLength(std::shared_ptr<const Landscape> landscapeID);
    void getAngle(std::shared_ptr<const Landscape> fromLandscape, std::shared_ptr<const Landscape> toLandscape, float64_t &crossAngle, float64_t &toAngle);
    std::shared_ptr<const Path> getPathAndLane(NDSPoint_t position, std::shared_ptr<const Lane> &lanePtr);
    bool getNextLandscape(std::shared_ptr<const Landscape> landscapePtr, std::shared_ptr<const Landscape> &nextLandscape,
                          std::vector<std::shared_ptr<const Landscape>> &otherSuccessor, bool onlyMainPath);
    bool pruneTree(uint16_t pathID, uint32_t offset, std::map<std::shared_ptr<const Landscape>, uint16_t> &pathMap,
                   std::shared_ptr<const Landscape> &startLandscape, float64_t &startRemain, uint16_t &repeatedLandscapePathID);
    void buildHorizon(std::shared_ptr<const Landscape> startLandscape, float64_t startRemain);
    static void autoUpdateTimer(Horizon *horizonPtr);
    static void autoUpdateAction(Horizon *horizonPtr);

private:
    uint32_t id_;
    std::vector<std::shared_ptr<const Path>> allPaths_;
    std::shared_ptr<const Path> mainPath_;
    uint32_t maxPathID_ = 0;
    float64_t firstLandscapeRemainLen_;
    bool onlyMainPath_;
    std::set<std::shared_ptr<const Landscape>> allLandscapes_;
    uint16_t horizonLength_;
    autoUpdateStrategy aus_;
    uint32_t timeShiftFromLastUpdate_ = 0;

    EHORIZON_DETAIL_LEVEL detailLevel_ = EHORIZON_DETAIL_LEVEL_BRIEF;

    std::mutex lckMutex_;
    std::condition_variable cv_;
};

} // namespace RDBVehicleAPI
