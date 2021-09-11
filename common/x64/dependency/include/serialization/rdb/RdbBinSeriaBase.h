/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RdbBinSeriaBase.h
 * @brief  Implementation of serializing data into a binary file
 *******************************************************************************
 */

#include <iosfwd>
#include <streambuf> // basic_streambuf
#include <string>
#include <cstddef>   // size_t
#include <memory>      // shared_ptr
#include <vector>
#include <set>
#include <map>
#include <list>
#include "serialization/rdb/RdbSeriaCommon.h"

#ifndef RDB_BIN_SERIA_BASE_H
#define RDB_BIN_SERIA_BASE_H

namespace roadDBCore
{

namespace rdbSerialization
{
//#define DEBUG_RDB_SERIALIZATION 1

/**
 *******************************************************************************
 * @class RdbBinSeriaBase
 *
 * @brief base class of data serialization into binary stream.
 *******************************************************************************
 */
template<typename  Seria>
class RdbBinSeriaBase
{
public:
    /**
     *******************************************************************************
     * @brief RdbBinSeriaBase - constructor
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
    RdbBinSeriaBase(std::ostream &os): charStreambuf_(*os.rdbuf())
    {
    }

    /**
     *******************************************************************************
     * @brief ~RdbBinSeriaBase - destructor
     *
     *  <1> Parameter Description:
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    ~RdbBinSeriaBase()
    {
        //static_cast<RdbStreamBufAccess<char> &>(charStreambuf_).sync();
    }

    /**
     *******************************************************************************
     * @brief This - Get the pointer of the derived Serializer object
     *
     *  <1> Parameter Description:
     *
     *  @return Seria *   Pointer of derived Serializer object
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    Seria * This()
    {
        return static_cast<Seria *>(this);
    }

    /**
     *******************************************************************************
     * @brief saveData - Save data into binary stream
     *
     *  <1> Parameter Description:
     *
     *  @param [In] pData   Data buffer.
     *
     *  @param [In] size    Data size.
     *
     *  @return void
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void saveData(const void *pData, std::size_t size)
    {
        std::streamsize count = charStreambuf_.sputn(static_cast<const char *>(pData), static_cast<std::streamsize>(size));

        if (size != static_cast<std::size_t>(count))
        {
            throw RdbSeriaException(RdbSeriaException::SERIA_EXCEPTION_WRITE_SB_E);
        }
    }

    /**
     *******************************************************************************
     * @brief operator& - Serialize data into binary stream
     *
     *  <1> Parameter Description:
     *
     *  @param [In] data
     *
     *  @return Seria   Reference of derived Serializer object
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    template <typename T>
    Seria & operator&(const T &data)
    {
#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "Start to serialize type: " << typeid(data).name();
#endif
        serialize(*this->This(), const_cast<T &>(data));

        return *this->This();
    }

    inline Seria & operator&(const uint8_t &data)
    {
        saveData((const char *)(&data), 1);

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "uint8_t data: " << data;
#endif

        return *this->This();
    }

    inline Seria & operator&(const int8_t &data)
    {
        saveData((const char *)(&data), 1);

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "int8_t data: " << data;
#endif

        return *this->This();
    }

    inline Seria & operator&(const uint16_t &data)
    {
#ifdef RDB_SYSTEM_BIG_ENDIAN
        uint16_t temp  = data;
        RdbReverse2Bytes(static_cast<const void *>(&temp));
        saveData((const char *)(&temp), 2);
#else
        saveData((const char *)(&data), 2);
#endif

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "uint16_t data: " << data;
#endif

        return *this->This();
    }

    inline Seria & operator&(const int16_t &data)
    {
#ifdef RDB_SYSTEM_BIG_ENDIAN
        uint16_t temp  = data;
        RdbReverse2Bytes(static_cast<const void *>(&temp));
        saveData((const char *)(&temp), 2);
#else
        saveData((const char *)(&data), 2);
#endif

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "int16_t data: " << data;
#endif

        return *this->This();
    }

    inline Seria & operator&(const uint32_t &data)
    {
#ifdef RDB_SYSTEM_BIG_ENDIAN
        uint32_t temp  = data;
        RdbReverse4Bytes(static_cast<const void *>(&temp));
        saveData((const char *)(&temp), 4);
#else
        saveData((const char *)(&data), 4);
#endif

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "uint32_t data: " << data;
#endif

        return *this->This();
    }

    inline Seria & operator&(const int32_t &data)
    {
#ifdef RDB_SYSTEM_BIG_ENDIAN
        uint32_t temp  = data;
        RdbReverse4Bytes(static_cast<const void *>(&temp));
        saveData((const char *)(&temp), 4);
#else
        saveData((const char *)(&data), 4);
#endif

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "int32_t data: " << data;
#endif

        return *this->This();
    }

    inline Seria & operator&(const uint64_t &data)
    {
#ifdef RDB_SYSTEM_BIG_ENDIAN
        uint64_t temp  = data;
        RdbReverse8Bytes(static_cast<const void *>(&temp));
        saveData((const char *)(&temp), 8);
#else
        saveData((const char *)(&data), 8);
#endif

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "uint64_t data: " << data;
#endif

        return *this->This();
    }

    inline Seria & operator&(const int64_t &data)
    {
#ifdef RDB_SYSTEM_BIG_ENDIAN
        uint64_t temp  = data;
        RdbReverse8Bytes(static_cast<const void *>(&temp));
        saveData((const char *)(&temp), 8);
#else
        saveData((const char *)(&data), 8);
#endif

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "int64_t data: " << data;
#endif

        return *this->This();
    }

    inline Seria & operator&(const float32_t &data)
    {
#ifdef RDB_SYSTEM_BIG_ENDIAN
        float32_t temp  = data;
        RdbReverse4Bytes(static_cast<const void *>(&temp));
        saveData((const char *)(&temp), 4);
#else
        saveData((const char *)(&data), 4);
#endif

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "float32_t data: " << data;
#endif

        return *this->This();
    }

    inline Seria & operator&(const float64_t &data)
    {
#ifdef RDB_SYSTEM_BIG_ENDIAN
        float64_t temp  = data;
        RdbReverse8Bytes(static_cast<const void *>(&temp));
        saveData((const char *)(&temp), 8);
#else
        saveData((const char *)(&data), 8);
#endif

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "float64_t data: " << data;
#endif

        return *this->This();
    }

    template <typename T, typename Allocator>
    inline Seria & operator&(const std::vector<T, Allocator> &data)
    {
        uint32_t size = static_cast<uint32_t>(data.size());

        *this->This() & size;

        for (auto &element : data)
        {
            *this->This() & element;
        }

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "vector size: " << size;
#endif

        return *this->This();
    }

    template <typename T, typename Allocator>
    inline Seria & operator&(const std::list<T, Allocator> &data)
    {
        uint32_t size = static_cast<uint32_t>(data.size());

        *this->This() & size;

        for (auto &element : data)
        {
            *this->This() & element;
        }

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "vector size: " << size;
#endif

        return *this->This();
    }

    template <class T, class Compare, class Alloc>
    inline Seria & operator&(const std::set<T, Compare, Alloc> &data)
    {
        uint32_t size = static_cast<uint32_t>(data.size());

        *this->This() & size;

        for (auto &element : data)
        {
            *this->This() & element;
        }

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "set size: " << size;
#endif

        return *this->This();
    }

    template <class Key, class T, class Compare, class Alloc>
    inline Seria & operator&(const std::map<Key, T, Compare, Alloc> &data)
    {
        uint32_t size = static_cast<uint32_t>(data.size());

        *this->This() & size;

        for (auto &element : data)
        {
            *this->This() & element.first & element.second;
        }

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "map size: " << size;
#endif

        return *this->This();
    }

    inline Seria & operator&(const std::string &strData)
    {
        uint32_t size = static_cast<uint32_t>(strData.size());

        *this->This() & size;
        saveData(static_cast<const void *>(strData.data()), size);

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "string size: " << size;
#endif

        return *this->This();
    }

    inline Seria & operator&(const CodingString &cstrData)
    {
        uint32_t size = cstrData.getObject().size();

        if ((cstrData.getCodeSize() < sizeof(size)) &&
            (size > static_cast<uint32_t>((1 << 8 * cstrData.getCodeSize()) - 1)))
        {
            std::stringstream ss;

            ss << "CodingString code size: " << cstrData.getCodeSize()
               << ", string length: " << size;
            throw RdbSeriaException(RdbSeriaException::SERIA_EXCEPTION_SIZE_OVERFLOW_E, ss.str().c_str());
        }

        saveData(static_cast<const void *>(&size), cstrData.getCodeSize());
        saveData(static_cast<const void *>(cstrData.getObject().data()), size);

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "CodingString size: " << size;
#endif

        return *this->This();
    }




private:

    std::basic_streambuf<char> &charStreambuf_;
};









}// namespace rdbSerialization

}// namespace roadDBCore




#endif // RDB_BIN_SERIA_BASE_H




