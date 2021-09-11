/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RdbBinDeseriaBase.h
 * @brief  Interface of deserializing data from a binary file
 *******************************************************************************
 */

#include <iosfwd>
#include <streambuf>    // basic_streambuf
#include <string>
#include <cstddef>      // size_t
#include <memory>       // shared_ptr
#include <vector>
#include <set>
#include <map>
#include <list>
#include "serialization/rdb/RdbSeriaCommon.h"

#ifndef RDB_BIN_DESERIA_BASE_H
#define RDB_BIN_DESERIA_BASE_H

namespace roadDBCore
{
namespace rdbSerialization
{
//#define DEBUG_RDB_SERIALIZATION 1

/**
 *******************************************************************************
 * @class RdbBinDeseriaBase
 *
 * @brief Base class of data deserialization from binary stream.
 *******************************************************************************
 */
template<typename  Seria>
class RdbBinDeseriaBase
{
public:
    /**
     *******************************************************************************
     * @brief RdbBinDeseriaBase - constructor
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
    RdbBinDeseriaBase(std::istream &is): charStreambuf_(*is.rdbuf())
    {
        count_ = 0;
    }

    /**
     *******************************************************************************
     * @brief ~RdbBinDeseriaBase - destructor
     *
     *  <1> Parameter Description:
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */

    ~RdbBinDeseriaBase()
    {
    }

    /**
     *******************************************************************************
     * @brief This - Get the pointer of the derived deserializer object
     *
     *  <1> Parameter Description:
     *
     *  @return Seria *   Pointer of derived deserializer object
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    Seria * This()
    {
        return static_cast<Seria *>(this);
    }

    uint32_t getCount() {return count_;}

    /**
     *******************************************************************************
     * @brief loadData - Load data from binary stream
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut] pData   Data buffer.
     *
     *  @param [In] size    Data size.
     *
     *  @return void
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    void loadData(void *pData, std::size_t size)
    {
        std::streamsize sSize = static_cast<std::streamsize>(size);
        std::streamsize count = charStreambuf_.sgetn(static_cast<char *>(pData), sSize);

        if (sSize != count)
        {
            throw RdbSeriaException(RdbSeriaException::SERIA_EXCEPTION_READ_SB_E);
        }

        count_ += static_cast<uint32_t>(size);
    }

    /**
     *******************************************************************************
     * @brief operator& - Deserialize data from binary stream
     *
     *  <1> Parameter Description:
     *
     *  @param [InOut] data
     *
     *  @return Seria &   Reference of derived deserializer object
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    template <typename T>
    inline Seria & operator&(T &data)
    {
#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "Start to deserialize type: " << typeid(data).name();
#endif
        serialize(*this->This(), data);

        return *this->This();
    }


    inline Seria & operator&(uint8_t &data)
    {
        loadData(static_cast<void *>(&data), 1);

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "uint8_t data: " << data;
#endif

        return *this->This();
    }

    inline Seria & operator&(int8_t &data)
    {
        loadData(static_cast<void *>(&data), 1);

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "int8_t data: " << data;
#endif

        return *this->This();
    }

    inline Seria & operator&(uint16_t &data)
    {
#ifdef RDB_SYSTEM_BIG_ENDIAN
        loadData(static_cast<void *>(&data), 2);
        RdbReverse2Bytes(static_cast<void *>(&data));
#else
        loadData(static_cast<void *>(&data), 2);
#endif

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "uint16_t data: " << data;
#endif

        return *this->This();
    }

    inline Seria & operator&(int16_t &data)
    {
#ifdef RDB_SYSTEM_BIG_ENDIAN
        loadData(static_cast<void *>(&data), 2);
        RdbReverse2Bytes(static_cast<void *>(&data));
#else
        loadData(static_cast<void *>(&data), 2);
#endif

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "int16_t data: " << data;
#endif

        return *this->This();
    }

    inline Seria & operator&(uint32_t &data)
    {
#ifdef RDB_SYSTEM_BIG_ENDIAN
        loadData(static_cast<void *>(&data), 4);
        RdbReverse2Bytes(static_cast<void *>(&data));
#else
        loadData(static_cast<void *>(&data), 4);
#endif

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "uint32_t data: " << data;
#endif

        return *this->This();
    }

    inline Seria & operator&(int32_t &data)
    {
#ifdef RDB_SYSTEM_BIG_ENDIAN
        loadData(static_cast<void *>(&data), 4);
        RdbReverse2Bytes(static_cast<void *>(&data));
#else
        loadData(static_cast<void *>(&data), 4);
#endif

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "int32_t data: " << data;
#endif

        return *this->This();
    }

    inline Seria & operator&(uint64_t &data)
    {
#ifdef RDB_SYSTEM_BIG_ENDIAN
        loadData(static_cast<void *>(&data), 8);
        RdbReverse2Bytes(static_cast<void *>(&data));
#else
        loadData(static_cast<void *>(&data), 8);
#endif

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "uint64_t data: " << data;
#endif

        return *this->This();
    }

    inline Seria & operator&(int64_t &data)
    {
#ifdef RDB_SYSTEM_BIG_ENDIAN
        loadData(static_cast<void *>(&data), 8);
        RdbReverse2Bytes(static_cast<void *>(&data));
#else
        loadData(static_cast<void *>(&data), 8);
#endif

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "int64_t data: " << data;
#endif

        return *this->This();
    }

    inline Seria & operator&(float32_t &data)
    {
#ifdef RDB_SYSTEM_BIG_ENDIAN
        loadData(static_cast<void *>(&data), 4);
        RdbReverse2Bytes(static_cast<void *>(&data));
#else
        loadData(static_cast<void *>(&data), 4);
#endif

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "float32_t data: " << data;
#endif

        return *this->This();
    }

    inline Seria & operator&(float64_t &data)
    {
#ifdef RDB_SYSTEM_BIG_ENDIAN
        loadData(static_cast<void *>(&data), 8);
        RdbReverse2Bytes(static_cast<void *>(&data));
#else
        loadData(static_cast<void *>(&data), 8);
#endif

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "float64_t data: " << data;
#endif

        return *this->This();
    }

    template <typename T, typename Allocator>
    inline Seria & operator&(std::vector<T, Allocator> &data)
    {
        uint32_t size = 0;

        *this->This() & size;
        data.resize(size);

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
    inline Seria & operator&(std::list<T, Allocator> &data)
    {
        uint32_t size = 0;

        *this->This() & size;
        data.resize(size);

        for (auto &element : data)
        {
            *this->This() & element;
        }

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "list size: " << size;
#endif

        return *this->This();
    }

    template <class T, class Compare, class Alloc>
    inline Seria & operator&(std::set<T, Compare, Alloc> &data)
    {
        T element;
        uint32_t size = 0;

        *this->This() & size;

        for (uint32_t i = 0; i < size; ++i)
        {
            *this->This() & element;
            data.insert(element);
        }

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "set size: " << size;
#endif

        return *this->This();
    }

    template <class Key, class T, class Compare, class Alloc>
    inline Seria & operator&(std::map<Key, T, Compare, Alloc> &data)
    {
        Key key;
        uint32_t size = 0;


        *this->This() & size;

        for (uint32_t i = 0; i < size; ++i)
        {
            *this->This() & key;
            *this->This() & data[key];
        }

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "map size: " << size;
#endif

        return *this->This();
    }

    inline Seria & operator&(std::string &strData)
    {
        uint32_t size = 0;

        *this->This() & size;
        strData.resize(size);

        //Note: the following code can not work in some string implementation
        if (0 < size)
        {
            loadData(static_cast<void *>(&(*strData.begin())), size);
        }

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "string: " << strData
                     << ", size: " << size;
#endif

        return *this->This();
    }

    inline Seria & operator&(CodingString &cstrData)
    {
        uint32_t size = 0;

        loadData(static_cast<void *>(&size), cstrData.getCodeSize());
        cstrData.getObject().resize(size);

        //Note: the following code can not work in some string implementation
        if (0 < size)
        {
            loadData(static_cast<void *>(&(*cstrData.getObject().begin())), size);
        }

#ifdef DEBUG_RDB_SERIALIZATION
        COM_LOG_INFO << "CodingString size: " << size;
#endif

        return *this->This();
    }


protected:
    //Context variables
    uint32_t count_;

private:
    std::basic_streambuf<char> &charStreambuf_;

};






} // namespace rdbSerialization

}// namespace roadDBCore




#endif // RDB_BIN_DESERIA_BASE_H





