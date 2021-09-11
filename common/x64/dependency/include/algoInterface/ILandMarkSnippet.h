/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   IRoadGeometrySnippet.h
 * @brief  Head file of class IRoadGeometrySnippet which define the interfaces provided by
 *            landmark snippet in server.
 *******************************************************************************
 */
 #ifndef ILANDMARK_SNIPPET_H
 #define ILANDMARK_SNIPPET_H

#include <memory>
#include <string>
#include "CommunicateDef/RdbV2SRoadObject.h"

/*************************
* 
* the interface for parser snippet file of land mark report by SDOR
*
***************************/

 namespace roadDBCore
 {

 class IRoadGeometrySnippet
 {
 public:
    virtual uint32_t getSnippetPayLoad(roadDBCore::PayloadBase_t*&  payload)=0;
    virtual void getSnippetName(std::string &fileName)=0;
 };

 }
 #endif

