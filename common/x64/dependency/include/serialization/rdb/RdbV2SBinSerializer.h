/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RdbV2SBinSerializer.h
 * @brief  Implementation of serializing data reported by vehicle into a binary file
 *******************************************************************************
 */

#include <iosfwd>
#include <streambuf> // basic_streambuf
#include <string>
#include <cstddef>   // size_t
#include "serialization/rdb/RdbBinSeriaBase.h"
#include "serialization/rdb/RdbSeriaSize.h"

#ifndef RDB_V2S_BIN_SERIALIZER_H
#define RDB_V2S_BIN_SERIALIZER_H

namespace roadDBCore
{

namespace rdbSerialization
{

/**
 *******************************************************************************
 * @class RdbV2SBinSerializer
 *
 * @brief Implementation of data serialization into binary stream.
 *******************************************************************************
 */
class  RdbV2SBinSerializer: public RdbBinSeriaBase<RdbV2SBinSerializer>
{
public:
    /**
     *******************************************************************************
     * @brief RdbV2SBinSerializer - constructor
     *
     *  <1> Parameter Description:
     *
     *  @param [In] os    Output stream.
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    RdbV2SBinSerializer(std::ostream &os): RdbBinSeriaBase<RdbV2SBinSerializer>(os)
    {
        version_ = 0;
        groupSize_ = 0;
        next_ = SNIPPET_PAYLOAD_TYPE_MAX_E;
    }

    /**
     *******************************************************************************
     * @brief ~RdbV2SBinSerializer - destructor
     *
     *  <1> Parameter Description:
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    ~RdbV2SBinSerializer()
    {
    }

    using RdbBinSeriaBase::operator&;

    /**
     *******************************************************************************
     * @brief operator& - Serialize data into binary stream
     *
     *  <1> Parameter Description:
     *
     *  @param [In] data
     *
     *  @return RdbV2SBinSerializer &   Reference of RdbV2SBinSerializer object
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbV2SBinSerializer & operator&(const CodingVector<VECTOR_CODE_TYPE_0BYTE_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbV2SBinSerializer & operator&(const CodingVector<VECTOR_CODE_TYPE_1BYTE_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbV2SBinSerializer & operator&(const CodingVector<VECTOR_CODE_TYPE_2BYTE_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbV2SBinSerializer & operator&(const CodingVector<VECTOR_CODE_TYPE_3BYTE_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbV2SBinSerializer & operator&(const CodingVector<VECTOR_CODE_TYPE_4BYTE_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbV2SBinSerializer & operator&(const CodingVector<VECTOR_CODE_TYPE_0BYTE_2BIT_ELEM_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbV2SBinSerializer & operator&(const CodingVector<VECTOR_CODE_TYPE_0BYTE_4BIT_ELEM_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbV2SBinSerializer & operator&(const CodingVector<VECTOR_CODE_TYPE_1BYTE_ELEM_NUMBER_E, T, Allocator> &data);

    uint32_t getVersion() {return version_;}
    void setVersion(uint32_t version) {version_ = version;}
    uint32_t getGroupSize() {return groupSize_;}
    void setGroupSize(uint32_t groupSize) {groupSize_ = groupSize;}
    SNIPPET_PAYLOAD_TYPE_E  getNextPayloadType() {return next_;}
    void setNextPayloadType(SNIPPET_PAYLOAD_TYPE_E next) {next_ = next;}

private:

    uint32_t version_;
    uint32_t groupSize_; //depending on descriptorType;
    SNIPPET_PAYLOAD_TYPE_E next_;
};

/**
 *******************************************************************************
 * @brief operator& - Serialize data into binary stream
 *
 *  <1> Parameter Description:
 *
 *  @param [Out] data
 *
 *  @return RdbV2SBinSerializer &   Reference of RdbV2SBinSerializer object
 *
 *  <2> Detailed Description:
 *******************************************************************************
 */
template <typename T, typename Allocator>
inline RdbV2SBinSerializer & RdbV2SBinSerializer::operator&
                            (const CodingVector<VECTOR_CODE_TYPE_0BYTE_E, T, Allocator> &data)
{
    for (auto &element : data.getObject())
    {
        *this & element;
    }

    return *this;
}

template <typename T, typename Allocator>
inline RdbV2SBinSerializer & RdbV2SBinSerializer::operator&
                            (const CodingVector<VECTOR_CODE_TYPE_1BYTE_E, T, Allocator> &data)
{
    uint8_t size = static_cast<uint8_t>(data.getObject().size());

    if (data.getObject().size() > (1 << 8 * VECTOR_CODE_TYPE_1BYTE_E) - 1)
    {
        std::stringstream ss;

        ss << "CodingVector<VECTOR_CODE_TYPE_1BYTE_E, T, Allocator>: " << data.getObject().size();
        throw RdbSeriaException(RdbSeriaException::SERIA_EXCEPTION_SIZE_OVERFLOW_E, ss.str().c_str());
    }

    *this & size;

    for (auto &element : data.getObject())
    {
        *this & element;
    }

    return *this;
}

template <>
inline RdbV2SBinSerializer & RdbV2SBinSerializer::operator&<uint8_t >
                            (const CodingVector<VECTOR_CODE_TYPE_1BYTE_E, uint8_t> &data)
{
    uint8_t size = static_cast<uint8_t>(data.getObject().size());

    if (data.getObject().size() > (1 << 8 * VECTOR_CODE_TYPE_1BYTE_E) - 1)
    {
        std::stringstream ss;

        ss << "CodingVector<VECTOR_CODE_TYPE_1BYTE_E, uint8_t>: " << data.getObject().size();
        throw RdbSeriaException(RdbSeriaException::SERIA_EXCEPTION_SIZE_OVERFLOW_E, ss.str().c_str());
    }

    *this & size;
    saveData(static_cast<const void *>(data.getObject().data()), size);

    return *this;
}

template <typename T, typename Allocator>
inline RdbV2SBinSerializer & RdbV2SBinSerializer::operator&
                            (const CodingVector<VECTOR_CODE_TYPE_2BYTE_E, T, Allocator> &data)
{
    uint16_t size = static_cast<uint16_t>(data.getObject().size());

    if (data.getObject().size() > (1 << 8 * VECTOR_CODE_TYPE_2BYTE_E) - 1)
    {
        std::stringstream ss;

        ss << "CodingVector<VECTOR_CODE_TYPE_2BYTE_E, T, Allocator>: " << data.getObject().size();
        throw RdbSeriaException(RdbSeriaException::SERIA_EXCEPTION_SIZE_OVERFLOW_E, ss.str().c_str());
    }

    *this & size;

    for (auto &element : data.getObject())
    {
        *this & element;
    }

    return *this;
}

template <typename T, typename Allocator>
inline RdbV2SBinSerializer & RdbV2SBinSerializer::operator&
                            (const CodingVector<VECTOR_CODE_TYPE_3BYTE_E, T, Allocator> &data)
{
    uint32_t size = static_cast<uint32_t>(data.getObject().size());

    if (size >= 65536 * 256)
    {
        std::stringstream ss;

        ss << "CodingVector<VECTOR_CODE_TYPE_3BYTE_E, T, Allocator>: " << size;
        throw RdbSeriaException(RdbSeriaException::SERIA_EXCEPTION_SIZE_OVERFLOW_E, ss.str().c_str());
    }

    //*this & size;
    saveData(static_cast<const void *>(&size), VECTOR_CODE_TYPE_3BYTE_E);

    for (auto &element : data.getObject())
    {
        *this & element;
    }

    return *this;
}

template <typename T, typename Allocator>
inline RdbV2SBinSerializer & RdbV2SBinSerializer::operator&
                            (const CodingVector<VECTOR_CODE_TYPE_4BYTE_E, T, Allocator> &data)
{
    uint32_t size = static_cast<uint32_t>(data.getObject().size());

    *this & size;

    for (auto &element : data.getObject())
    {
        *this & element;
    }

    return *this;
}

template <>
inline RdbV2SBinSerializer & RdbV2SBinSerializer::operator&<uint8_t>
                            (const CodingVector<VECTOR_CODE_TYPE_0BYTE_2BIT_ELEM_E, uint8_t> &data)
{
    uint32_t size = data.getObject().size();
    const uint32_t saveSize  = (2 * size + 7) / 8;
    auto &vec = data.getObject();

    for (uint32_t i = 0; i < saveSize; ++i)
    {
        uint8_t value = 0;

        if ((size % 4) && (i == (saveSize - 1)))
        {
            for (uint32_t j = 0; j < (size % 4); ++j)
            {
                value |= (vec[i * 4 + j] & 0x3) << 2 * j;
            }
        }
        else
        {
            value |=  vec[i * 4] & 0x3;
            value |= (vec[i * 4 + 1] & 0x3) << 2;
            value |= (vec[i * 4 + 2] & 0x3) << 4;
            value |= (vec[i * 4 + 3] & 0x3) << 6;
        }

#ifdef DEBUG_RDB_SERIALIZATION
        std::cout << "index " << i << std::hex << ": 0x" << static_cast<uint32_t>(value);
#endif
        *this & value; // save a byte
   }

   return *this;
}

template <>
inline RdbV2SBinSerializer & RdbV2SBinSerializer::operator&<uint8_t>
                            (const CodingVector<VECTOR_CODE_TYPE_0BYTE_4BIT_ELEM_E, uint8_t> &data)
{
    uint32_t size = data.getObject().size();
    const uint32_t saveSize  = (4 * size + 7) / 8;
    auto &vec = data.getObject();

    for (uint32_t i = 0; i < saveSize; ++i)
    {
        uint8_t value = 0;

        if ((size % 2) && (i == (saveSize - 1)))
        {
            for (uint32_t j = 0; j < (size % 2); ++j)
            {
                value |= (vec[i * 2 + j] & 0xf) << 4 * j;
            }
        }
        else
        {
            value |=  vec[i * 2] & 0xf;
            value |= (vec[i * 2 + 1] & 0xf) << 4;
        }

#ifdef DEBUG_RDB_SERIALIZATION
        std::cout << "index " << i << std::hex << ": 0x" << static_cast<uint32_t>(value);
#endif
        *this & value; // save a byte
   }

   return *this;
}

template <>
inline RdbV2SBinSerializer & RdbV2SBinSerializer::operator&<uint8_t>
                            (const CodingVector<VECTOR_CODE_TYPE_1BYTE_ELEM_NUMBER_E, uint8_t> &data)
{
    uint32_t size = data.getObject().size();
    uint8_t number = static_cast<uint8_t>(size / groupSize_);

    if (size / groupSize_ > (1 << 8 * 1) - 1)
    {
        std::stringstream ss;

        ss << size / groupSize_;
        throw RdbSeriaException(RdbSeriaException::SERIA_EXCEPTION_SIZE_OVERFLOW_E, ss.str().c_str());
    }

    *this & number;
    saveData(data.getObject().data(), size);

#ifdef DEBUG_RDB_SERIALIZATION
    COM_LOG_INFO << "size: " << size << ", groupSize: " << groupSize_ << ", number: " << number;
#endif

    return *this;
}


}// namespace rdbSerialization

}// namespace roadDBCore




#endif //RDB_V2S_BIN_SERIALIZER_H




