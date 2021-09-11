/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RdbSeriaSize.h
 * @brief  Interface of getting serializing size of a data type
 *******************************************************************************
 */

#include <string>
#include <vector>
#include "RdbSeriaCommon.h"

#ifndef RDB_SERIA_SIZE_H
#define RDB_SERIA_SIZE_H

namespace roadDBCore
{

namespace rdbSerialization
{

/**
 *******************************************************************************
 * @class RdbSeriaSize
 *
 * @brief Used to caculate serialized data size.
 *******************************************************************************
 */
class RdbSeriaSize
{
public:
    /**
     *******************************************************************************
     * @brief RdbSeriaSize - constructor
     *
     *  <1> Parameter Description:
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    RdbSeriaSize() {size_ = 0;}

    /**
     *******************************************************************************
     * @brief ~RdbSeriaSize - destructor
     *
     *  <1> Parameter Description:
     *
     *  @return
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    ~RdbSeriaSize() {}

    /**
     *******************************************************************************
     * @brief getSize - Get serialized data size.
     *
     *  <1> Parameter Description:
     *
     *  @param [In] data
     *
     *  @return uint32_t serialized data size
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    template <typename T>
    inline uint32_t getSize(const T &data);

    /**
     *******************************************************************************
     * @brief operator& - Caculate serialized data size.
     *
     *  <1> Parameter Description:
     *
     *  @param [In] data
     *
     *  @return RdbSeriaSize &   Reference of RdbSeriaSize object
     *
     *  <2> Detailed Description:
     *******************************************************************************
     */
    template <typename T>
    inline RdbSeriaSize & operator&(const T &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbSeriaSize & operator&(std::vector<T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbSeriaSize & operator&(const CodingVector<VECTOR_CODE_TYPE_0BYTE_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbSeriaSize & operator&(const CodingVector<VECTOR_CODE_TYPE_1BYTE_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbSeriaSize & operator&(const CodingVector<VECTOR_CODE_TYPE_2BYTE_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbSeriaSize & operator&(const CodingVector<VECTOR_CODE_TYPE_3BYTE_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbSeriaSize & operator&(const CodingVector<VECTOR_CODE_TYPE_4BYTE_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbSeriaSize & operator&(const CodingVector<VECTOR_CODE_TYPE_0BYTE_2BIT_ELEM_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbSeriaSize & operator&(const CodingVector<VECTOR_CODE_TYPE_0BYTE_4BIT_ELEM_E, T, Allocator> &data);

    template <typename T, typename Allocator = std::allocator<T> >
    inline RdbSeriaSize & operator&(const CodingVector<VECTOR_CODE_TYPE_1BYTE_ELEM_NUMBER_E, T, Allocator> &data);


private:

    uint32_t size_;

    template<typename Seria, typename T>
    friend void serialize(Seria &seria, T &data);
};

/**
 *******************************************************************************
 * @brief getSize - Caculate serialized data size.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] data
 *
 *  @return RdbSeriaSize    RdbSeriaSize object
 *
 *  <2> Detailed Description:
 *******************************************************************************
 */
template <typename T>
inline RdbSeriaSize & RdbSeriaSize::operator&(const T &data)
{
    *this & data;

    return *this;
}

template <>
inline RdbSeriaSize & RdbSeriaSize::operator&<uint8_t>(const uint8_t &data)
{
    size_ += 1;

    return *this;
}

template <>
inline RdbSeriaSize & RdbSeriaSize::operator&<int8_t>(const int8_t &data)
{
    size_ += 1;

    return *this;
}

template <>
inline RdbSeriaSize & RdbSeriaSize::operator&<uint16_t>(const uint16_t &data)
{
    size_ += 2;

    return *this;
}

template <>
inline RdbSeriaSize & RdbSeriaSize::operator&<int16_t>(const int16_t &data)
{
    size_ += 2;

    return *this;
}

template <>
inline RdbSeriaSize & RdbSeriaSize::operator&<uint32_t>(const uint32_t &data)
{
    size_ += 4;

    return *this;
}

template <>
inline RdbSeriaSize & RdbSeriaSize::operator&<int32_t>(const int32_t &data)
{
    size_ += 4;

    return *this;
}

template <>
inline RdbSeriaSize & RdbSeriaSize::operator&<uint64_t>(const uint64_t &data)
{
    size_ += 8;

    return *this;
}

template <>
inline RdbSeriaSize & RdbSeriaSize::operator&<int64_t>(const int64_t &data)
{
    size_ += 8;

    return *this;
}

template <>
inline RdbSeriaSize & RdbSeriaSize::operator&<float32_t>(const float32_t &data)
{
    size_ += 4;

    return *this;
}

template <>
inline RdbSeriaSize & RdbSeriaSize::operator&<float64_t>(const float64_t &data)
{
    size_ += 8;

    return *this;
}

template <typename T, typename Allocator>
inline RdbSeriaSize & RdbSeriaSize::operator&(const CodingVector<VECTOR_CODE_TYPE_0BYTE_E, T, Allocator> &data)
{
    uint32_t number = data.getObject().size();

    if (number > 0)
    {
        uint32_t size = size_;
        size_ = size + number * getSize(data.getObject()[0]);
    }

    return *this;
}

template <typename T, typename Allocator>
inline RdbSeriaSize & RdbSeriaSize::operator&(const CodingVector<VECTOR_CODE_TYPE_1BYTE_E, T, Allocator> &data)
{
    uint32_t number = data.getObject().size();

    size_ += 1;

    if (number > 0)
    {
        uint32_t size = size_;
        size_ = size + number * getSize(data.getObject()[0]);
    }

    return *this;
}

template <typename T, typename Allocator>
inline RdbSeriaSize & RdbSeriaSize::operator&(const CodingVector<VECTOR_CODE_TYPE_2BYTE_E, T, Allocator> &data)
{
    uint32_t number = data.getObject().size();

    size_ += 2;

    if (number > 0)
    {
        uint32_t size = size_;
        size_ = size + number * getSize(data.getObject()[0]);
    }

    return *this;
}

template <typename T, typename Allocator>
inline RdbSeriaSize & RdbSeriaSize::operator&(const CodingVector<VECTOR_CODE_TYPE_3BYTE_E, T, Allocator> &data)
{
    uint32_t number = data.getObject().size();

    size_ += 3;

    if (number > 0)
    {
        uint32_t size = size_;
        size_ = size + number * getSize(data.getObject()[0]);
    }

    return *this;
}

template <typename T, typename Allocator>
inline RdbSeriaSize & RdbSeriaSize::operator&(const CodingVector<VECTOR_CODE_TYPE_4BYTE_E, T, Allocator> &data)
{
    uint32_t number = data.getObject().size();

    size_ += 4;

    if (number > 0)
    {
        uint32_t size = size_;
        size_ = size + number * getSize(data.getObject()[0]);
    }

    return *this;
}

template <>
inline RdbSeriaSize & RdbSeriaSize::operator&<uint8_t>
                        (const CodingVector<VECTOR_CODE_TYPE_0BYTE_2BIT_ELEM_E, uint8_t> &data)
{
    uint32_t number = data.getObject().size();

    size_ += (number * 2 + 7) / 8;

    return *this;
}

template <>
inline RdbSeriaSize & RdbSeriaSize::operator&<uint8_t>
                        (const CodingVector<VECTOR_CODE_TYPE_0BYTE_4BIT_ELEM_E, uint8_t> &data)
{
    uint32_t number = data.getObject().size();

    size_ += (number * 4 + 7) / 8;

    return *this;
}

template <>
inline RdbSeriaSize & RdbSeriaSize::operator&<uint8_t>
                        (const CodingVector<VECTOR_CODE_TYPE_1BYTE_ELEM_NUMBER_E, uint8_t> &data)
{
    size_ += 1 + data.getObject().size();

    return *this;
}

template <typename T, typename Allocator>
inline RdbSeriaSize & RdbSeriaSize::operator&(std::vector<T, Allocator> &data)
{
    uint32_t number = data.size();

    size_ += 4;

    if (number > 0)
    {
        uint32_t size = size_;
        size_ = size + number * getSize(data[0]);
    }

    return *this;
}

template <>
inline RdbSeriaSize & RdbSeriaSize::operator&<std::string>(const std::string &data)
{
    size_ += 4 + data.size();

    return *this;
}

/**
 *******************************************************************************
 * @brief getSize - Get serialized data size.
 *
 *  <1> Parameter Description:
 *
 *  @param [In] data
 *
 *  @return uint32_t serialized data size
 *
 *  <2> Detailed Description:
 *******************************************************************************
 */
template <typename T>
inline uint32_t RdbSeriaSize::getSize(const T &data)
{
    size_ = 0;
    *this & data;
    return size_;
}

template <>
inline uint32_t RdbSeriaSize::getSize<uint8_t>(const uint8_t &data)
{
    return 1;
}

template <>
inline uint32_t RdbSeriaSize::getSize<int8_t>(const int8_t &data)
{
    return 1;
}

template <>
inline uint32_t RdbSeriaSize::getSize<uint16_t>(const uint16_t &data)
{
    return 2;
}

template <>
inline uint32_t RdbSeriaSize::getSize<int16_t>(const int16_t &data)
{
    return 2;
}

template <>
inline uint32_t RdbSeriaSize::getSize<uint32_t>(const uint32_t &data)
{
    return 4;
}

template <>
inline uint32_t RdbSeriaSize::getSize<int32_t>(const int32_t &data)
{
    return 4;
}

template <>
inline uint32_t RdbSeriaSize::getSize<uint64_t>(const uint64_t &data)
{
    return 8;
}

template <>
inline uint32_t RdbSeriaSize::getSize<int64_t>(const int64_t &data)
{
    return 8;
}

template <>
inline uint32_t RdbSeriaSize::getSize<float32_t>(const float32_t &data)
{
    return 4;
}

template <>
inline uint32_t RdbSeriaSize::getSize<float64_t>(const float64_t &data)
{
    return 8;
}


}// namespace rdbSerialization

}// namespace roadDBCore




#endif // RDB_SERIA_SIZE_H




