/**
 ************************************************************************************
 *                         RoadDB Confidential
 *                    Copyright (c) RoadDB 2020
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 ************************************************************************************
 * @file   fwvCommon.h
 * @brief  Interface of fwvCommon
 ************************************************************************************
 */
#pragma once
#include <string>
#include<vector>
namespace FWVCommon
{

/**
 *******************************************************************************
    *  @brief shell - run a shell command,
    *
    *  <1> Parameter Description:
    *
    *  @param [Out] exitCode,
    *               [In] cmd, command,
    *
    *  @return the shell command output content;
    *
    *  <2> Detailed Description:
    *  run a shell command
********************************************************************************
**/
std::string shell(const std::string& cmd, int32_t& exitCode);
std::string shell(const std::string& cmd);
void split(const std::string &s, const std::string &delim, std::vector<std::string> &ret);


}
