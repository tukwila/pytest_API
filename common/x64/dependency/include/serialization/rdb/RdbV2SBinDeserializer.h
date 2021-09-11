/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RdbV2SBinDeserializer.h
 * @brief  Interface of deserializing data reported by vehicle from a binary file
 *******************************************************************************
 */

#include <iosfwd>
#include <streambuf> // basic_streambuf
#include <string>
#include <cstddef>   // size_t
#include "serialization/rdb/RdbBinDeseriaBase.h"
#include "CommunicateDef/RdbV2SCommon.h"

#ifndef RDB_V2S_BIN_DESERIALIZER_H
#define RDB_V2S_BIN_DESERIALIZER_H

namespace roadDBCore
{
namespace rdbSerialization
{

/**
 *******************************************************************************
 * @class RdbV2SBinDeserializer
 *
 * @brief Implementation of data deserialization from binary stream.
 *******************************************************************************
 */
class  RdbV2SBinDeserializer: public RdbBinDeseriaBase<RdbV2SBinDeserializer>
{
public:
    /**
     *******************************************************************************
     * @brief RdbV2SBinDeserializer - constructor
     *
     *  <1> Parameter Description:
     *
     *  @param [In] is    Input stream.
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    RdbV2SBinDeserializer(std::istream &is): RdbBinDeseriaBase<RdbV2SBinDeserializer>(is)
    {
        init();
    }

    /**
     *******************************************************************************
     * @brief ~RdbV2SBinDeserializer - destructor
     *
     *  <1> Parameter Description:
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    ~RdbV2SBinDeserializer()
    {
    }

    /**
     *******************************************************************************
     * @brief init - Init internal status
     *
     *  <1> Parameter Description:
     *
     *  @return void
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void init()
    {
        // header info
        headerLen_ = 0;
        payloadType_ = SNIPPET_PAYLOAD_TYPE_MAX_E;
        version_ = 0;
        refTimeStamp_ = 0;
        refGpsLon_ = 0;
        refGpsLat_ = 0;
        refGpsAlt_ = 0;
        sensorDataSpecVersion_ = 0;
        //payloadLen_ = 0;
        next_ = SNIPPET_PAYLOAD_TYPE_MAX_E;

        //Context variables
        groupSize_ = 0;
#if 0
        // Load header info
        loadData(static_cast<void *>(&headerLen_), 2);
        loadData(static_cast<void *>(&payloadType_), 1);
        loadData(static_cast<void *>(&version_), 4);
        loadData(static_cast<void *>(&refTimeStamp_), 8);
        loadData(static_cast<void *>(&refGpsLon_), 4);
        loadData(static_cast<void *>(&refGpsLat_), 4);
        loadData(static_cast<void *>(&refGpsAlt_), 4);
        loadData(static_cast<void *>(&containerSize_), static_cast<std::size_t>(STRING_CODE_SIZE_1BYTE_E));

        if (0 < containerSize_)
        {
            char *pData = new char[containerSize_ + 1];
            loadData(pData, containerSize_);
            pData[containerSize_] = 0;
            strVehicleID_ = pData;
        }

        //loadData(static_cast<void *>(&payloadLen_), 4);
#endif
    }

    using RdbBinDeseriaBase::operator&;

    /**
     *******************************************************************************
     * @brief operator& - Deserialize data from binary stream
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut] data
     *
     *  @return RdbV2SBinDeserializer &   Reference of RdbV2SBinDeserializer object
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbV2SBinDeserializer & operator&(CodingVector<VECTOR_CODE_TYPE_0BYTE_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbV2SBinDeserializer & operator&(CodingVector<VECTOR_CODE_TYPE_1BYTE_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbV2SBinDeserializer & operator&(CodingVector<VECTOR_CODE_TYPE_2BYTE_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbV2SBinDeserializer & operator&(CodingVector<VECTOR_CODE_TYPE_3BYTE_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbV2SBinDeserializer & operator&(CodingVector<VECTOR_CODE_TYPE_4BYTE_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbV2SBinDeserializer & operator&(CodingVector<VECTOR_CODE_TYPE_0BYTE_2BIT_ELEM_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbV2SBinDeserializer & operator&(CodingVector<VECTOR_CODE_TYPE_0BYTE_4BIT_ELEM_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbV2SBinDeserializer & operator&(CodingVector<VECTOR_CODE_TYPE_1BYTE_ELEM_NUMBER_E, T, Allocator> &data);

    /**
     *******************************************************************************
     * @brief getHeaderLen - Get internal deserialized header length
     *
     *  <1> Parameter Description:
     *
     *  @return uint16_t   Header length
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    uint16_t getHeaderLen() {return headerLen_;}
    SNIPPET_PAYLOAD_TYPE_E  getPayloadType() {return payloadType_;}
    uint32_t getVersion() {return version_;}
    uint64_t getRefTimeStamp() {return refTimeStamp_;}
    uint32_t getRefGpsLon() {return refGpsLon_;}
    uint32_t getRefGpsLat() {return refGpsLat_;}
    uint32_t getRefGpsAlt() {return refGpsAlt_;}
    std::string getVehicleID() {return strVehicleID_;}
    float32_t getSensorDataSpecVersion() {return sensorDataSpecVersion_;}
    //uint32_t getPayloadLen() {return payloadLen_;}
    uint32_t getGroupSize() {return groupSize_;}
    SNIPPET_PAYLOAD_TYPE_E  getNextPayloadType() {return next_;}

    /**
     *******************************************************************************
     * @brief setHeaderLen - Set header length
     *
     *  <1> Parameter Description:
     *
     *  @param [In] headerLen    Header length deserialized from header structure.
     *
     *  @return void
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void setHeaderLen(uint16_t headerLen) {headerLen_ = headerLen;}
    void setPayloadType(SNIPPET_PAYLOAD_TYPE_E payloadType) {payloadType_ = payloadType;}
    void setVersion(uint32_t version) {version_ = version;}
    void setRefTimeStamp(uint64_t refTimeStamp) {refTimeStamp_ = refTimeStamp;}
    void setRefGpsLon(uint32_t refGpsLon) {refGpsLon_ = refGpsLon;}
    void setRefGpsLat(uint32_t refGpsLat) {refGpsLat_ = refGpsLat;}
    void setRefGpsAlt(uint32_t refGpsAlt) {refGpsAlt_ = refGpsAlt;}
    void setVehicleID(std::string strVehicleID) {strVehicleID_ = strVehicleID;}

    void setSensorDataSpecVersion(float32_t sensorDataSpecVersion)
    {
        sensorDataSpecVersion_ = sensorDataSpecVersion;
    }

    void setGroupSize(uint32_t groupSize) {groupSize_ = groupSize;}
    void setNextPayloadType(SNIPPET_PAYLOAD_TYPE_E next) {next_ = next;}

private:
    // header info
    uint16_t headerLen_;
    SNIPPET_PAYLOAD_TYPE_E payloadType_;
    uint32_t version_;
    uint64_t refTimeStamp_;
    uint32_t refGpsLon_;
    uint32_t refGpsLat_;
    uint32_t refGpsAlt_;
    std::string strVehicleID_;
    float32_t sensorDataSpecVersion_;
    //uint32_t payloadLen_;

    //Context variables
    uint32_t groupSize_; //depending on descriptorType;
    SNIPPET_PAYLOAD_TYPE_E next_;

};

/**
 *******************************************************************************
 * @brief operator& - Deserialize data from binary stream
 *
 *  <1> Parameter Description:
 *
 *  @param [InOut] data
 *
 *  @return RdbV2SBinDeserializer &   Reference of RdbV2SBinDeserializer object
 *
 *  <2> Detailed Description:
 *******************************************************************************
 */
template <typename T, typename Allocator>
inline RdbV2SBinDeserializer & RdbV2SBinDeserializer::operator&
                              (CodingVector<VECTOR_CODE_TYPE_0BYTE_E, T, Allocator> &data)
{
    for (auto &element : data.getObject())
    {
        *this & element;
    }

    return *this;
}

template <typename T, typename Allocator>
inline RdbV2SBinDeserializer & RdbV2SBinDeserializer::operator&
                              (CodingVector<VECTOR_CODE_TYPE_1BYTE_E, T, Allocator> &data)
{
    uint8_t size = 0;

    *this & size;
    data.getObject().resize(static_cast<uint32_t>(size));

    for (auto &element : data.getObject())
    {
        *this & element;
    }

    return *this;
}

template <>
inline RdbV2SBinDeserializer & RdbV2SBinDeserializer::operator&<uint8_t>
                              (CodingVector<VECTOR_CODE_TYPE_1BYTE_E, uint8_t> &data)
{
    uint8_t size = 0;

    *this & size;
    data.getObject().resize(static_cast<uint32_t>(size));
    loadData(static_cast<void *>(data.getObject().data()), static_cast<uint32_t>(size));

    return *this;
}

template <typename T, typename Allocator>
inline RdbV2SBinDeserializer & RdbV2SBinDeserializer::operator&
                              (CodingVector<VECTOR_CODE_TYPE_2BYTE_E, T, Allocator> &data)
{
    uint16_t size = 0;

    *this & size;
    data.getObject().resize(static_cast<uint32_t>(size));

    for (auto &element : data.getObject())
    {
        *this & element;
    }

    return *this;
}

template <typename T, typename Allocator>
inline RdbV2SBinDeserializer & RdbV2SBinDeserializer::operator&
                              (CodingVector<VECTOR_CODE_TYPE_3BYTE_E, T, Allocator> &data)
{
    uint32_t size = 0;

    //*this & size;
    loadData(static_cast<void *>(&size), VECTOR_CODE_TYPE_3BYTE_E);
    data.getObject().resize(size);

    for (auto &element : data.getObject())
    {
        *this & element;
    }

    return *this;
}

template <typename T, typename Allocator>
inline RdbV2SBinDeserializer & RdbV2SBinDeserializer::operator&
                              (CodingVector<VECTOR_CODE_TYPE_4BYTE_E, T, Allocator> &data)
{
    uint32_t size = 0;

    *this & size;
    data.getObject().resize(size);

    for (auto &element : data.getObject())
    {
        *this & element;
    }

    return *this;
}

template <>
inline RdbV2SBinDeserializer & RdbV2SBinDeserializer::operator&<uint8_t>
                              (CodingVector<VECTOR_CODE_TYPE_0BYTE_2BIT_ELEM_E, uint8_t> &data)
{
    uint32_t size = data.getObject().size();
    uint32_t loadSize  = (2 * size + 7) / 8;
    auto &vec = data.getObject();

    for (uint32_t i = 0; i < loadSize; ++i)
    {
        uint8_t value = 0;

        *this & value; // get a byte

        if ((size % 4) && (i == (loadSize - 1)))
        {
            for (uint32_t j = 0; j < (size % 4); ++j)
            {
                vec[i * 4 + j] = (value & (0x3 << 2 * j)) >> 2 * j;
            }
        }
        else
        {
            vec[i * 4] = value & 0x3;                     // 1st 2 bits
            vec[i * 4 + 1] = (value & (0x3 << 2)) >> 2;   // 2nd
            vec[i * 4 + 2] = (value & (0x3 << 4)) >> 4;   // 3rd
            vec[i * 4 + 3] = (value & (0x3 << 6)) >> 6;   // 4th
        }

#ifdef DEBUG_RDB_SERIALIZATION
        std::cout << "index " << i << std::hex << ": 0x" << static_cast<uint32_t>(value);
#endif
    }

    return *this;
}

template <>
inline RdbV2SBinDeserializer & RdbV2SBinDeserializer::operator&<uint8_t>
                              (CodingVector<VECTOR_CODE_TYPE_0BYTE_4BIT_ELEM_E, uint8_t> &data)
{
    uint32_t size = data.getObject().size();
    uint32_t loadSize  = (4 * size + 7) / 8;
    auto &vec = data.getObject();

    for (uint32_t i = 0; i < loadSize; ++i)
    {
        uint8_t value = 0;

        *this & value; // get a byte

        if ((size % 2) && (i == (loadSize - 1)))
        {
            for (uint32_t j = 0; j < (size % 2); ++j)
            {
                vec[i * 2 + j] = (value & (0xf << 4 * j)) >> 4 * j;
            }
        }
        else
        {
            vec[i * 2] = value & 0xf;
            vec[i * 2 + 1] = (value & 0xf0) >> 4;
        }

#ifdef DEBUG_RDB_SERIALIZATION
        std::cout << "index " << i << std::hex << ": 0x" << static_cast<uint32_t>(value);
#endif
    }

    return *this;
}

template <>
inline RdbV2SBinDeserializer & RdbV2SBinDeserializer::operator&<uint8_t>
                              (CodingVector<VECTOR_CODE_TYPE_1BYTE_ELEM_NUMBER_E, uint8_t> &data)
{
    uint8_t size = 0;

    *this & size;

    uint32_t elemtentNum = static_cast<uint32_t>(size);

    elemtentNum *= groupSize_;
    data.getObject().resize(elemtentNum);
    loadData(static_cast<void *>(data.getObject().data()), elemtentNum);

    return *this;
}


} // namespace rdbSerialization

}// namespace roadDBCore




#endif //RDB_V2S_BIN_DESERIALIZER_H





