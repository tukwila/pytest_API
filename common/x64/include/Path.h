/*******************************************************************************
 *                       RoadDB Confidential
 *                  Copyright (c) RoadDB 2019-2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   Path.h
 * @brief  The class of horizon path, in accordance with ADASIS v3 Spec.
 *
 * Change Log:
 * Date              Author            Changes
 * 2020-03-01        Lindun Tang       Init version.
 *
 *******************************************************************************
 */

#pragma once
#include "EVP.h"
#include "Profile.h"
#include "Landscape.h"

namespace RDBVehicleAPI
{
/**
 * @brief  The class definition for objects.
 */
class Path
{
public:
    const std::vector<std::weak_ptr<const Path>> &getLayer1SubPaths() const { return layer1SubPaths_; }

    /**
     * @brief Get the nearest sub path among all 1st level sub path.
     * @return a vector contain all the nearest sub path
     */
    const std::vector<std::weak_ptr<const Path>> &getNearestSubPath() const { return nearestSubPaths_; }

    /**
     * @brief Get all the 1st level sub path with a offset smaller than the given offset and sub path of those 1st level sub path
     * @return a vector contain all the sub path with smaller offset
     */
    std::vector<std::weak_ptr<const Path>> getSubPathByOffset(uint32_t offset) const;

    /**
     * @brief Get all the sub paths on layer smaller than or equal to the given layer parameter
     * @param layer, the given layer
     * @return a vector contain all the sub path
     */
    std::vector<std::weak_ptr<const Path>> getSubPathByLayer(uint8_t layer) const;

    /**
     * @brief Get ID of this path
     * @return path ID
     */
    uint16_t getID() const { return id_; }

    /**
     * @brief Get ID of parent path
     * @return parent path ID
     */
    uint16_t getParentPathID() const { return parentID_; }

    /**
     * @brief Get path offset
     * @return offset value
     */
    uint32_t getOffset() const { return offset_; }

    /**
     * @brief Get path length
     * @return length value
     */
    uint32_t getLength() const { return length_; }

    /**
     * @brief Get all the landscape IDs inside this path
     * @return landscape IDs
     */
    const std::vector<std::shared_ptr<const Landscape>> &getLandscapeList() const { return landscapeList_; }

    /* @bref get all the profiles on a path
     * @return all the profiles on this path
     */
    const std::vector<std::shared_ptr<const Profile>> &getProfile() const { return profiles_; }

    /* @bref get all the profile of the same type on a path
     * @param type, the specific profile type
     * @return all the profiles of the given type
     */
    void getProfileByType(ProfileType type, std::vector<std::shared_ptr<const Profile>> &profiles) const;

    /* @bref get a profile at an exact offset on a path
     * @param type, the specific profile type
     * @param offset, the offset to get profile
     * @return the specific profile at the given offset or nullptr if no corresponding profile exist
     */
    std::shared_ptr<const Profile> getProfileByOffset(ProfileType type, uint32_t offset) const;

    /* @bref get the next profile with the given type and the distance to that specific type
     * @param type, the specific profile type
     * @param offset, the offset representing the position of ego vehicle
     * @param nextProfileDis, the distance to that specific profile
     * @return the specific profile, or nullptr if no coressponding profile exist
     */
    std::shared_ptr<const Profile> getNextProfile(ProfileType type, uint32_t offset, uint32_t &nextProfileDis) const;

    /* @bref get the valid distance to the end of a profile with the given type
     * @param type, the specific profile type
     * @param offset, the offset trepresenting the position of ego vehicle
     * @param validLength, the valid length of current profile with the given type
     * @return the specific profile, or nullptr if no coressponding profile exist
     */
    std::shared_ptr<const Profile> getValidProfileLength(ProfileType type, uint16_t offset, uint16_t &validLength) const;

    float64_t getOffsetFromRoot() const { return accuOffsetFromRoot_; }
    
private:
    Path(uint16_t id, uint16_t parentID, uint32_t offset) : id_(id), parentID_(parentID), offset_(offset) { LOG_TRACE << "Create path id:" << id; }
    const std::vector<std::weak_ptr<const Path>> &getAllSubPath() const;
    void setPathLength(uint32_t len) { length_ = len; }
    void profileGeneration();
    void genCurvatureProfile();
    void genNodeProfile();
    void genTrafficSignProfile();
    void genLaneModelProfile();
    void genLaneConnectivityProfile();
    void genLinearObjProfile();
    void genLaneModel();
    std::shared_ptr<const Landscape> getRootPathStartLandscape() { return rootPathStartLandscape_; }
    void setRootPathStartLandscape(std::shared_ptr<const Landscape> landscapePtr) { rootPathStartLandscape_ = landscapePtr; }
    void setHorizonFirstLegLen(float64_t len) { horizonFirstLegLen_ = len; }
    float64_t getHorizonFirstLegLen() { return horizonFirstLegLen_; }
    static float64_t getLandscapeLength(std::shared_ptr<const Landscape> landscapeID);
    static void getAngle(std::shared_ptr<const Landscape> fromLandscape, std::shared_ptr<const Landscape> toLandscape, float64_t &crossAngle, float64_t &toAngle);
    static WGS84_t getLineRefPoint(std::shared_ptr<const Line> linePtr);
    void setOffsetFromRoot(float64_t offsetValue) { accuOffsetFromRoot_ = offsetValue; }

    static LaneTransition getTransition(const std::shared_ptr<const Lane> lane);
    static bool isLaneWidthChanged(std::shared_ptr<const Lane> fromLandscape);
    static uint8_t getLaneIdx(std::shared_ptr<const Lane> lane);
    uint8_t getPathIDByLane(std::shared_ptr<const Lane> lane);
    static LineMarkingType getLineType(std::shared_ptr<const Line> linePtr);

    bool isRootPath() const { return id_ == rootPathID_; }
    void setRootPathID(uint16_t rootPathID) { rootPathID_ = rootPathID; }
    static std::shared_ptr<const Line> getLandscapeRefLine(std::shared_ptr<const Landscape> landscapeID);

private:
    std::vector<std::shared_ptr<const Landscape>> landscapeList_;
    std::vector<std::weak_ptr<const Path>> nearestSubPaths_;
    std::vector<std::weak_ptr<const Path>> allSubPaths_;
    std::vector<std::weak_ptr<const Path>> layer1SubPaths_;
    uint16_t id_;
    uint16_t parentID_;
    uint16_t rootPathID_;
    std::shared_ptr<const Landscape> rootPathStartLandscape_;
    uint32_t offset_ = 0;
    float64_t accuOffsetFromRoot_;
    float64_t horizonFirstLegLen_;
    uint32_t length_;

    std::vector<std::shared_ptr<const Profile>> profiles_;
    uint32_t profileID_ = 0;

    friend class Horizon;
};

} // namespace RDBVehicleAPI
