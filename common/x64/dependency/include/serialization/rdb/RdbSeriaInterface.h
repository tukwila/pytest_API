/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RdbSeriaInterface.h
 * @brief  Interface of RDB serialization module.
 *******************************************************************************
 */


#include <fstream>
#include "CommunicateDef/RdbV2SCommon.h"
#include "CommunicateDef/RdbV2SGeometry.h"
#include "CommunicateDef/RdbV2SRoadObject.h"
#include "CommunicateDef/RdbV2SSlam.h"
#include "CommunicateDef/RdbV2SLandmark.h"
#include "errorCode/commonSysErrorCode.h"
#include "serialization/rdb/RdbV2SBinSerializer.h"
#include "serialization/rdb/RdbV2SBinDeserializer.h"
#include "serialization/rdb/RdbV2SSeriaImpCommon.h"
#include "serialization/rdb/RdbV2SSeriaImpGeometry.h"
#include "serialization/rdb/RdbV2SSeriaImpRoadObject.h"
#include "serialization/rdb/RdbV2SSeriaImpSlam.h"
#include "serialization/rdb/RdbV2SSeriaImpLandmark.h"
#include "serialization/rdb/RdbServerSeriaImpDB.h"
#include "serialization/rdb/RdbServerSeriaImpRoad.h"
#include "LogWrapper/LogWrapper.h" //log
#include "serialization/dataCompatibility.h"

#ifndef RDB_SERIA_INTERFACE_H
#define RDB_SERIA_INTERFACE_H

namespace roadDBCore
{
namespace rdbSerialization
{

/**
 *******************************************************************************
 * @brief rdbV2SBinFileSeria - Serialize the input header and payload data into
 *  a binary file.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - fileName   The name used to create a binary file for serialization
 *
 *  @param [In]  - header   The header data used to be serialized
 *
 *  @param [In]  - payload   The payload data used to be serialized
 *
 *  @return ROAD_DATABASE_OK if success, or return error status
 *
 *  <2> Detailed Description:
 *  This template API is used to serialize data stuctures reported from vehicle
 *  to server in binary mode.
 *  It's supposed to support any defined PayloadType.
 *
 *  \ingroup rdbSerialization
 *******************************************************************************
 */
template<typename PayloadType>
uint32_t rdbV2SBinFileSeria(const std::string &fileName,
                            const RoadDatabaseHead_t &header,
                            const PayloadType &payload)
{
    try
    {
        std::ofstream fileStream(fileName.c_str());

        if (!fileStream.good())
        {
            COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_OPEN_FILE)
                          << "Failed to open the file -- "
                          << fileName;

            return RDB_SERIA_ERR_OPEN_FILE;
        }

        RdbV2SBinSerializer rdbSerializer(fileStream);
        rdbSerializer & header & payload;
    }

    catch (RdbSeriaException &e)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_SAVE) << e.what();

        return RDB_SERIA_ERR_SAVE;
    }

    catch(std::exception &e)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_SAVE)
                      << "Failed to serialize data into the file: "
                      << fileName;
        COM_LOG_ERROR << e.what();

        return RDB_SERIA_ERR_SAVE;
    }

    catch(...)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_SAVE)
                      << "Unknow exception when serializing data into the file: "
                      << fileName;

        return RDB_SERIA_ERR_SAVE;
    }

    return ROAD_DATABASE_OK;
}

/**
  *******************************************************************************
  * @brief rdbV2SBinFileDeseria - Deserialize the header info and payload data
  *  from an input binary file.
  *
  *  <1> Parameter Description:
  *
  *  @param [In]  - fileName   The binary file used to have deserialization
  *
  *  @param [out]  - header   Deserialized header info
  *
  *  @param [out]  - payload   Deserialized payload info
  *
  *  @return ROAD_DATABASE_OK if success, or return error status
  *
  *  <2> Detailed Description:
  *  This template API is used to deserialize data stuctures reported from vehicle
  *  to server from the input binary file.
  *  It's supposed to support any defined PayloadType.
  *
  *  \ingroup rdbSerialization
  *******************************************************************************
*/
template<typename PayloadType>
uint32_t rdbV2SBinFileDeseria(const std::string &fileName,
                              RoadDatabaseHead_t &header,
                              PayloadType &payload)
{
    try
    {
        std::ifstream fileStream(fileName.c_str(), std::ios::in | std::ios::binary);

        if (!fileStream.good())
        {
            COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_OPEN_FILE)
                          << "Failed to open the file -- "
                          << fileName;

            return RDB_SERIA_ERR_OPEN_FILE;
        }

        RdbV2SBinDeserializer rdbDeserializer(fileStream);
        rdbDeserializer & header & payload;
    }

    catch (RdbSeriaException &e)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_LOAD) << e.what();

        return RDB_SERIA_ERR_LOAD;
    }

    catch (std::exception &e)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_LOAD)
                      << "Failed to deserialize data from the file: "
                      << fileName;
        COM_LOG_ERROR << e.what();

        return RDB_SERIA_ERR_LOAD;
    }

    catch(...)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_LOAD)
                      << "Unknow exception when deserializing data from the file: "
                      << fileName;

        return RDB_SERIA_ERR_LOAD;
    }

    return ROAD_DATABASE_OK;
}

/**
  *******************************************************************************
  * @brief rdbV2SBinFileSeria - Serialize the input header and payload data into
  *  a binary file.
  *
  *  <1> Parameter Description:
  *
  *  @param [In]  - fileName   The name used to create a binary file for serialization
  *
  *  @param [In]  - header   The header data used to be serialized
  *
  *  @param [In]  - vecSpPayloadBase   The vector of payload data used to be serialized
  *
  *  @return ROAD_DATABASE_OK if success, or return error status
  *
  *  <2> Detailed Description:
  *  This template API is used to serialize data stuctures reported from vehicle
  *  to server in binary mode.
  *  It's supposed to support any defined PayloadType.
  *
  *  \ingroup rdbSerialization

  *******************************************************************************
*/
uint32_t rdbV2SBinFileSeria(const std::string &fileName,
                            const RoadDatabaseHead_t &header,
                            const std::vector<std::shared_ptr<PayloadBase_t>> &vecSpPayloadBase);

/**
  *******************************************************************************
  * @brief rdbV2SBinFileDeseria - Deserialize the header info and payload data
  *  from an input binary file.
  *
  *  <1> Parameter Description:
  *
  *  @param [In]  - fileName   The binary file used to have deserialization
  *
  *  @param [out]  - header   Deserialized header info
  *
  *  @param [out]  - vecSpPayloadBase   Deserialized payload info
  *
  *  @return ROAD_DATABASE_OK if success, or return error status
  *
  *  <2> Detailed Description:
  *  This template API is used to deserialize data stuctures reported from vehicle
  *  to server from the input binary file.
  *  It's supposed to support any defined PayloadType.
  *
  *  \ingroup rdbSerialization
  *******************************************************************************
*/
uint32_t rdbV2SBinFileDeseria(const std::string &fileName,
                                       RoadDatabaseHead_t &header,
                                       std::vector<std::shared_ptr<PayloadBase_t>> &vecSpPayloadBase);

uint32_t rdbV2SBinFileDeseria(const std::string &fileName,
                              RoadDatabaseHead_t &header,
                              std::shared_ptr<PayloadBase_t> &spPayloadBase);

/**
 *******************************************************************************
 * @brief rdbV2SBinFileDeseriaHeader - Deserialize the header info
 *  from an input binary file.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - fileName   The binary file to be deserialized
 *
 *  @param [out]  - header   Deserialized header info
 *
 *  @return ROAD_DATABASE_OK if success, or return error status
 *
 *  <2> Detailed Description:
 *  This API is used to deserialize header info from a input binary file.
 *
 *  \ingroup rdbSerialization
 *******************************************************************************
 */
uint32_t rdbV2SBinFileDeseriaHeader(const std::string &fileName, RoadDatabaseHead_t &header);

/**
 *******************************************************************************
 * @brief rdbV2SBinFileDeseriaData - Deserialize a specified data by an input
 *  binary Deserializer.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - rdbDeserializer   The binary Deserializer used to have
 *  deserialization, which should have been bound to a binary serialized file.
 *
 *  @param [out]  - data   Deserialized data info
 *
 *  @return ROAD_DATABASE_OK if success, or return error status
 *
 *  <2> Detailed Description:
 *  This template API can be used to deserialize any data stucture from a binary
 *  file bound to the input Deserializer.
 *
 *  Note: Only for header (data type: RoadDatabaseHead_t), this interface can
 *  be called any times without side effects.
 *
 *  \ingroup rdbSerialization
 *******************************************************************************
 */
template<typename T>
uint32_t rdbV2SBinFileDeseriaData(RdbV2SBinDeserializer &rdbDeserializer, T &data)
{
    try
    {
        rdbDeserializer & data;
    }

    catch (RdbSeriaException &e)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_LOAD) << e.what();

        return RDB_SERIA_ERR_LOAD;
    }

    catch (std::exception &e)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_LOAD) << "Failed to deserialize data";
        COM_LOG_ERROR << e.what();

        return RDB_SERIA_ERR_LOAD;
    }

    catch(...)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_LOAD)
                      << "Unknow exception when deserializing data";

        return RDB_SERIA_ERR_LOAD;
    }

    return ROAD_DATABASE_OK;
}

/**
 *******************************************************************************
 * @brief rdbBinStreamSeria - Serialize the input data object to a stream.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - object   The data object used to be serialized
 *
 *  @param [out] - outputStream   the stream used to hold serialized data
 *
 *  @return ROAD_DATABASE_OK if success, or return error status
 *
 *  <2> Detailed Description:
 *  This template API is used to serialize a data object into a binary string
 *  format
 *
 *  \ingroup rdbSerialization
 *******************************************************************************
 */
template<typename T>
uint32_t rdbBinStreamSeria(const T &object, std::iostream &outputStream)
{
    if (!outputStream.good())
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_BAD_STREAM)
                      << "Error! Bad output stream for Serialization.";

        return RDB_SERIA_ERR_BAD_STREAM;
    }

    try
    {
        RdbV2SBinSerializer rdbSerializer(outputStream);
        rdbSerializer & object;
    }

    catch (RdbSeriaException &e)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_SAVE) << e.what();

        return RDB_SERIA_ERR_SAVE;
    }

    catch(std::exception &e)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_SAVE)
                      << "Failed to serialize data into stream.";
        COM_LOG_ERROR << e.what();

        return RDB_SERIA_ERR_SAVE;
    }

    catch(...)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_SAVE)
                      << "Unknow exception when serializing data into stream.";

        return RDB_SERIA_ERR_SAVE;
    }

    return ROAD_DATABASE_OK;
}

/**
 *******************************************************************************
 * @brief rdbBinStreamDeseria - Deserialize a data object from an input stream.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - inputStream  Input stream for deserialization
 *
 *  @param [out] - object   Deserialized data object
 *
 *  @return ROAD_DATABASE_OK if success, or return error status
 *
 *  <2> Detailed Description:
 *  This template API is used to deserialize a data object from an input binary
 *  string.
 *
 *  \ingroup rdbSerialization
 *******************************************************************************
 */
template<typename T>
uint32_t rdbBinStreamDeseria(T &object, std::iostream &inputStream)
{
    if (!inputStream.good())
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_BAD_STREAM)
                      << "Error! Bad input stream for Deserialization.";

        return RDB_SERIA_ERR_BAD_STREAM;
    }

    try
    {
        RdbV2SBinDeserializer rdbDeserializer(inputStream);
        rdbDeserializer & object;
    }

    catch (RdbSeriaException &e)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_LOAD) << e.what();

        return RDB_SERIA_ERR_LOAD;
    }

    catch (std::exception &e)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_LOAD)
                      << "Failed to deserialize.";
        COM_LOG_ERROR << e.what();

        return RDB_SERIA_ERR_LOAD;
    }

    catch(...)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_LOAD)
                      << "Unknow exception when deserializing data.";

        return RDB_SERIA_ERR_LOAD;
    }

    return ROAD_DATABASE_OK;
}


/**
 *******************************************************************************
 * @brief rdbBinFileSeria - Serialize the input data into a binary file.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - fileName   The name used to create a binary file for serialization
 *
 *  @param [In]  - object   The data object used to be serialized
 *
 *  @return ROAD_DATABASE_OK if success, or return error status
 *
 *  <2> Detailed Description:
 *  This template API is used to serialize a data object into a binary file.
 *  Currently, we can use this API to serialize rtMatrix data.
 *
 *  \ingroup rdbSerialization
 *******************************************************************************
 */
template<typename T>
uint32_t rdbBinFileSeria(const std::string &fileName, const T &object)
{
    std::fstream fileStream(fileName, std::ios::out);

    if (!fileStream.good())
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_OPEN_FILE)
                      << "Failed to open the file -- "
                      << fileName;

        return RDB_SERIA_ERR_OPEN_FILE;
    }

    return rdbBinStreamSeria(object, fileStream);
}

/**
 *******************************************************************************
 * @brief rdbBinFileDeseria - Deserialize a data object from an input binary file.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - fileName   The binary file used to have deserialization
 *
 *  @param [out]  - object   Deserialized data object
 *
 *  @return ROAD_DATABASE_OK if success, or return error status
 *
 *  <2> Detailed Description:
 *  This template API is used to deserialize a data object from an input binary file.
 *  Currently, we can use this API to deserialize rtMatrix data.
 *
 *  \ingroup rdbSerialization
 *******************************************************************************
 */
template<typename T>
uint32_t rdbBinFileDeseria(const std::string &fileName, T &object)
{
    std::fstream fileStream(fileName, std::ios::in | std::ios::binary);

    if (!fileStream.good())
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_OPEN_FILE)
                      << "Failed to open the file -- "
                      << fileName;

        return RDB_SERIA_ERR_OPEN_FILE;
    }

    return rdbBinStreamDeseria(object, fileStream);
}

/**
 *******************************************************************************
 * @brief rdbBinSStreamSeria - Serialize the input data object to a string.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - object   The data object used to be serialized
 *
 *  @param [out]  - str   the string converted from the input data object
 *
 *  @return ROAD_DATABASE_OK if success, or return error status
 *
 *  <2> Detailed Description:
 *  This template API is used to serialize a data object into a binary string
 *  format
 *
 *  \ingroup rdbSerialization
 *******************************************************************************
 */
template<typename T>
uint32_t rdbBinSStreamSeria(const T &object, std::string &str)
{
    uint32_t status = ROAD_DATABASE_OK;
    std::stringstream sstream;

    if (!sstream.good())
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_OPEN_FILE)
                      << "Failed to open string stream.";

        return RDB_SERIA_ERR_OPEN_FILE;
    }

    status = rdbBinStreamSeria(object, sstream);

    if (ROAD_DATABASE_OK == status)
    {
        str = sstream.str();
    }

    return status;
}

/**
 *******************************************************************************
 * @brief rdbBinSStreamDeseria - Deserialize a data object from an input string.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - str   The string to be deserialized
 *
 *  @param [out]  - object   Deserialized data object
 *
 *  @return ROAD_DATABASE_OK if success, or return error status
 *
 *  <2> Detailed Description:
 *  This template API is used to deserialize a data object from an input binary
 *  string.
 *
 *  \ingroup rdbSerialization
 *******************************************************************************
 */
template<typename T>
uint32_t rdbBinSStreamDeseria(T &object, const std::string &str)
{
    std::stringstream sstream;

    sstream.str(str);

    if (!sstream.good())
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_OPEN_FILE)
                      << "Failed to open string stream.";

        return RDB_SERIA_ERR_OPEN_FILE;
    }

    return rdbBinStreamDeseria(object, sstream);
}

/**
 *******************************************************************************
 * @brief rdbBinVecSStreamSeria - Serialize the input vector object to a string.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - object   The data object used to be serialized
 *
 *  @param [out]  - str   the string converted from the input data object
 *
 *  @return ROAD_DATABASE_OK if success, or return error status
 *
 *  <2> Detailed Description:
 *  This template API is used to serialize a data object into a binary string
 *  format
 *
 *  \ingroup rdbSerialization
 *******************************************************************************
 */
template <VECTOR_CODE_TYPE_E codeSize, typename T>
uint32_t rdbBinVecSStreamSeria(const std::vector<T> &object, std::string &str)
{
    CodingVector<codeSize, T> cvector(const_cast<std::vector<T> &>(object));

    return rdbBinSStreamSeria(cvector, str);
}

/**
 *******************************************************************************
 * @brief rdbBinVecSStreamDeseria - Deserialize a vector object from an input string.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - str   The string to be deserialized
 *
 *  @param [out]  - object   Deserialized data object
 *
 *  @return ROAD_DATABASE_OK if success, or return error status
 *
 *  <2> Detailed Description:
 *  This template API is used to deserialize a data object from an input binary
 *  string.
 *
 *  \ingroup rdbSerialization
 *******************************************************************************
 */
template <VECTOR_CODE_TYPE_E codeSize, typename T>
uint32_t rdbBinVecSStreamDeseria(std::vector<T> &object, const std::string &str)
{
    CodingVector<codeSize, T> cvector(object);

    return rdbBinSStreamDeseria(cvector, str);
}

/**
 *******************************************************************************
 * @brief rdbBinFileSeria - Serialize VehicleSectionDetail data to file.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - fileName   The name used to create a binary file for serialization
 *
 *  @param [In]  - vehicleDetail   The vehicleDetail data used to be serialized
 *
 *  @return void
 *******************************************************************************
 */
template<uint32_t version = DB_VERSION_VEHICLE_FILE>
uint32_t rdbBinFileSeria(const std::string &fileName,
                    const typename VehicleDBData<version>::DATA_TYPE &vehicleDetail)
{
    static_assert(VehicleDBData<version>::bSupport, "The vehicle data version is not supported!");

    std::fstream fileStream(fileName, std::ios::out);

    if (!fileStream.good())
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_OPEN_FILE) << "Failed to open the file -- " << fileName;
        return RDB_SERIA_ERR_OPEN_FILE;
    }

    try
    {
        RdbV2SBinSerializer rdbSerializer(fileStream);

        rdbSerializer & version & vehicleDetail;
    }

    catch (RdbSeriaException &e)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_SAVE) << e.what();
        return RDB_SERIA_ERR_SAVE;
    }

    catch(std::exception &e)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_SAVE)
                      << "Failed to serialize data into the file: "
                      << fileName;
        COM_LOG_ERROR << e.what();
        return RDB_SERIA_ERR_SAVE;
    }

    catch(...)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_SAVE)
                      << "Unknow exception when serializing data into the file: " << fileName;
        return RDB_SERIA_ERR_SAVE;
    }

    return ROAD_DATABASE_OK;
}

/**
 *******************************************************************************
 * @brief rdbBinFileDeseria - Deserialize VehicleSectionDetail data from an input binary file.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - fileName   The binary file used to have deserialization
 *
 *  @param [In]  - vehicleDetail   Deserialized vehicleDetail info
 *
 *  @return void
 *******************************************************************************
 */
template<uint32_t version = DB_VERSION_VEHICLE_FILE>
uint32_t rdbBinFileDeseria(const std::string &fileName,
                           typename VehicleDBData<version>::DATA_TYPE &vehicleDetail)
{
    static_assert(VehicleDBData<version>::bSupport, "The vehicle data version is not supported!");

    uint32_t status = ROAD_DATABASE_OK;
    std::fstream fileStream(fileName, std::ios::in | std::ios::binary);

    if (!fileStream.good())
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_OPEN_FILE) << "Failed to open the file -- " << fileName;
        return RDB_SERIA_ERR_OPEN_FILE;
    }

    try
    {
        RdbV2SBinDeserializer rdbDeserializer(fileStream);
        uint32_t codedVersion = DB_VERSION_VEHICLE_FILE;

        rdbDeserializer & codedVersion;

        status = deseriaCompatibleData(rdbDeserializer, codedVersion, vehicleDetail);

        if (ROAD_DATABASE_OK != status)
        {
            COM_LOG_ERROR << "The coded vehicle data version "
                          << codedVersion
                          << " is not supported, current version is "
                          << DB_VERSION_VEHICLE_FILE;
        }
    }

    catch (RdbSeriaException &e)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_LOAD) << e.what();
        return RDB_SERIA_ERR_LOAD;
    }

    catch (std::exception &e)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_LOAD)
                      << "Failed to deserialize data from the file: " << fileName;
        COM_LOG_ERROR << e.what();
        return RDB_SERIA_ERR_LOAD;
    }

    catch(...)
    {
        COM_LOG_ERROR << errorCode(RDB_SERIA_ERR_LOAD)
                      << "Unknow exception when deserializing data from the file: " << fileName;
        return RDB_SERIA_ERR_LOAD;
    }

    return ROAD_DATABASE_OK;
}

/**
 *******************************************************************************
 * @brief rdbBinFileSeria - Serialize VehicleSectionDetail data to file.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - fileName   The name used to create a binary file for serialization
 *
 *  @param [In]  - spVehicleData   The vehicleDetail data used to be serialized
 *
 *  @return void
 *******************************************************************************
 */
template<uint32_t version = DB_VERSION_VEHICLE_FILE>
uint32_t rdbBinFileSeria(const std::string &fileName,
                        const std::shared_ptr<typename VehicleDBData<version>::DATA_TYPE> &spVehicleData)
{
    static_assert(VehicleDBData<version>::bSupport, "The vehicle data version is not supported!");

    uint32_t status = ROAD_DATABASE_OK;

    if (spVehicleData)
    {
        status = rdbBinFileSeria<version>(fileName, *spVehicleData);
    }

    return status;
}

/**
 *******************************************************************************
 * @brief rdbBinFileDeseria - Deserialize VehicleSectionDetail data from an input binary file.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - fileName   The binary file used to have deserialization
 *
 *  @param [In]  - spVehicleData   Deserialized vehicleDetail info
 *
 *  @return void
 *******************************************************************************
 */
template<uint32_t version = DB_VERSION_VEHICLE_FILE>
uint32_t rdbBinFileDeseria(const std::string &fileName,
                           std::shared_ptr<typename VehicleDBData<version>::DATA_TYPE> &spVehicleData,
                           bool isReference = true)
{
    static_assert(VehicleDBData<version>::bSupport, "The vehicle data version is not supported!");

    uint32_t status = ROAD_DATABASE_FAIL;

    if (!spVehicleData)
    {
        spVehicleData = std::make_shared<typename VehicleDBData<version>::DATA_TYPE>();
    }

    if (spVehicleData)
    {
        if (!isReference)
        {
            spVehicleData->eMode = VEHICLE_DIV_MODE_NO_REF_E;
        }

        status = rdbBinFileDeseria<version>(fileName, *spVehicleData);
    }
    else
    {
        COM_LOG_WARN << "Failed to allocate memory for deserializaiton of vehicle data from file "
                     << fileName;
    }

    return status;
}

/**
 *******************************************************************************
 * @brief rdbBinFileSeria - Serialize anchorGps roads and intersections data to file.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - fileName   The name used to create a binary file for serialization
 *               - anchorGps
 *               - intersections
 *               - roads
 *
 *  @return ROAD_DATABASE_OK if success, or return error status
 *******************************************************************************
 */
uint32_t rdbBinFileSeria(const std::string &fileName,
                         const Point3d_t &anchorGps,
                         const std::vector<std::shared_ptr<SIntersection_t>> &intersections,
                         const std::vector<std::shared_ptr<SRoad_t>> &roads);

/**
 *******************************************************************************
 * @brief rdbBinFileDeseria - Deserialize anchorGps roads and intersections data from file.
 *
 *  <1> Parameter Description:
 *
 *  @param [In]  - fileName   The binary file used to have deserialization
 *
 *  @param [Out] - anchorGps
 *               - intersections
 *               - roads
 *
 *  @return ROAD_DATABASE_OK if success, or return error status
 *******************************************************************************
 */
uint32_t rdbBinFileDeseria(const std::string &fileName,
                           Point3d_t &anchorGps,
                           std::vector<std::shared_ptr<SIntersection_t>> &intersections,
                           std::vector<std::shared_ptr<SRoad_t>> &roads);

} // namespace rdbSerialization

}// namespace roadDBCore




#endif //RDB_SERIA_INTERFACE_H


