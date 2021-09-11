/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 2016-2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   ISnippetParser.h
 * @brief  interface of Snippet Parser
 *
 *
 *******************************************************************************
 */

#ifndef _I_SNIPPET_PARSER_
#define _I_SNIPPET_PARSER_

#include "CommunicateDef/RdbV2SSlam.h"
#include "CommunicateDef/RdbV2SGeometry.h"


namespace roadDBCore
{

struct SnippetInfo_t
{
    std::string rtvName;
    std::string snippetName;
    std::string vehicleID;
    Point3d_t refGps;
    uint64_t timeStamp = 0LL;
};

/**
 *******************************************************************************
 * @class ISnippetParser
 * @brief This class is used to define interface of snippet parser
 *
 *******************************************************************************
 */
class ISnippetParser
{
public:

    /**
     *******************************************************************************
     * @brief getInfo - get general info of the parsing snippet
     *
     * <1> Parameter Description:
     *
     * @param [Out] - vehicleID
     *
     * @param [Out] - refGps   reference gps
     *
     * @param [Out] - timeStamp   time stamp
     *
     * @return status - success return 0, otherwise return error code;
     *
     * Detailed Description:
     *
     *******************************************************************************
     **/
    virtual uint32_t getInfo(SnippetInfo_t &snippetInfo) = 0;

    /**
     *******************************************************************************
     * @brief getSnippetPayLoad - get all payload data of the parsing snippet
     *
     * Parameter Description:
     *
     * @param [Out] - snippetData   snippet payload data
     *
     * @return status - success return 0, otherwise return error code;
     *
     *  Detailed Description:
     *
     *******************************************************************************
     **/
    virtual uint32_t getSnippetPayLoad(std::vector<std::shared_ptr<PayloadBase_t>> &snippetData) = 0;
};

} // namespace roadDBCore

#endif


