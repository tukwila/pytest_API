/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017-2019
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   RdbSeriaCommon.h
 * @brief  Common interface and type definition for Road Database serialization
 *******************************************************************************
 */

#include <vector>
#include "typeDef.h"
#include "LogWrapper/LogWrapper.h" //log

#ifndef RDB_SERIA_COMMON_H
#define RDB_SERIA_COMMON_H

namespace roadDBCore
{

namespace rdbSerialization
{

//#define DEBUG_RDB_SERIALIZATION

const uint32_t SERIALIZE_SYNC_CODE_FIN = 0xABADFACE;
const uint32_t SERIALIZE_SYNC_CODE_NEXT = 0x00BEFACE;

enum VECTOR_CODE_TYPE_E: uint8_t
{
    /* Code vector size with 0 byte,
       which can be deduced by previous vector size */
    VECTOR_CODE_TYPE_0BYTE_E = 0,

    /* Code vector size with 1 byte,
       thus vector size should be less than 256 */
    VECTOR_CODE_TYPE_1BYTE_E = 1,

    /* Code vector size with 2 bytes,
       thus vector size should be less than 65536 */
    VECTOR_CODE_TYPE_2BYTE_E = 2,

    /* Code vector size with 3 bytes,
       thus vector size should be less than 65536 * 256 */
    VECTOR_CODE_TYPE_3BYTE_E = 3,

    /* Code vector size with 4 bytes,
       thus vector size should be less than 65536 * 65536 */
    VECTOR_CODE_TYPE_4BYTE_E = 4,

     /* Code vector size with 0 byte,
        and each vector element just holds 2 bits */
    VECTOR_CODE_TYPE_0BYTE_2BIT_ELEM_E,

     /* Code vector size with 0 byte,
        and each vector element just holds 4 bits */
    VECTOR_CODE_TYPE_0BYTE_4BIT_ELEM_E,

    /* Code 1-byte elements number, not vector size.
       Each element size can be computed by other predfined parameters.
       Element number can be calculated by "total size" / "element size" */
    VECTOR_CODE_TYPE_1BYTE_ELEM_NUMBER_E,

    VECTOR_CODE_TYPE_MAX_E
};

enum STRING_CODE_SIZE_E: uint8_t
{
    /* Code string size with 0 byte,
       which can be deduced by previous string size */
    //STRING_CODE_SIZE_0BYTE_E = 0,

    /* Code string size with 1 byte,
       thus string size should be less than 256 */
    STRING_CODE_SIZE_1BYTE_E = 1,

    /* Code string size with 2 bytes,
       thus string size should be less than 65536 */
    STRING_CODE_SIZE_2BYTE_E = 2,

    /* Code string size with 4 bytes,
       thus string size should be less than 65536 * 65536 */
    STRING_CODE_SIZE_4BYTE_E = 4,

    STRING_CODE_SIZE_MAX_E
};

enum ENDIAN_TYPE_E: uint8_t
{
    ENDIAN_TYPE_LITTLE_E = 0,
    ENDIAN_TYPE_BIG_E,
    ENDIAN_TYPE_MAX_E
};

template <VECTOR_CODE_TYPE_E eCodeSize, typename T, typename Allocator = std::allocator<T> >
class CodingVector
{
public:
    explicit CodingVector(std::vector<T, Allocator> &vecObject, uint32_t size = 0)
                              : vecObject_(vecObject)
    {
        if (size > 0)
        {
            vecObject_.resize(size);
        }
    }

    ~CodingVector() {}

    //operator std::vector<T, Allocator> &()
    std::vector<T, Allocator> & getObject()
    {
        return vecObject_;
    }

    const std::vector<T, Allocator> & getObject() const
    {
        return vecObject_;
    }

private:
    std::vector<T, Allocator> &vecObject_;
};

class CodingString
{
public:
    explicit CodingString(std::string &strObject, STRING_CODE_SIZE_E eCodeSize): strObject_(strObject),
                                                                                       eCodeSize_(eCodeSize)
    {
    }

    ~CodingString() {}

    uint32_t getCodeSize() const
    {
        return static_cast<uint32_t>(eCodeSize_);
    }

    std::string & getObject() const
    {
        return strObject_;
    }

private:
    std::string &strObject_;
    STRING_CODE_SIZE_E eCodeSize_;
};

class  RdbSeriaException: public std::exception
{
public:
    enum SERIA_EXCEPTION_E: uint8_t
    {
        SERIA_NO_EXCEPTION = 0,                 // unspecified exception code
        SERIA_EXCEPTION_WRITE_SB_E,             // Exception when writing stream buffer
        SERIA_EXCEPTION_READ_SB_E,              // Exception when reading stream buffer
        SERIA_EXCEPTION_NO_SERIALIZE_E,         // Exception when no implementation for serialization
        SERIA_EXCEPTION_SYNC_CODE_E,            // Failed to check sync code
        SERIA_EXCEPTION_SLAM_DESCRIPTOR_E,      // Invalid SLAM descriptor
        SERIA_EXCEPTION_NORM_CAMERA_TYPE_E,     // Invalid nomalized camera type
        SERIA_EXCEPTION_TRUNCATE_TYPE_E,        // Invalid truncate type
        SERIA_EXCEPTION_SCALE_FACTOR_E,         // Invalid scale factor
        SERIA_EXCEPTION_MAT_ELEM_TYPE_E,        // Unsupported element type in matrix
        SERIA_EXCEPTION_LANE_MARKING_TYPE_E,    // Invalid lane mark type
        SERIA_EXCEPTION_ROAD_OBJ_TYPE_E,        // Invalid road object type
        SERIA_EXCEPTION_LANE_BOUNDARY_TYPE_E,   // Invalid lane boundary type
        SERIA_EXCEPTION_LANE_TYPE_E,            // Invalid lane type
        SERIA_EXCEPTION_ROAD_SURFACE_OBJ_TYPE_E,    // Invalid road surface object type
        SERIA_EXCEPTION_ARROW_PAINT_TYPE_E,         // Invalid arrow paint type
        SERIA_EXCEPTION_ROAD_FURNITURE_OBJ_TYPE_E,  // Invalid road funiture object type
        SERIA_EXCEPTION_ROAD_EDGE_TYPE_E,           // Invalid road edge type
        SERIA_EXCEPTION_GEOMETRY_TYPE_E,            // Invalid geometry type
        SERIA_EXCEPTION_POINT_TYPE_E,               // Invalid point type
        SERIA_EXCEPTION_LINE_TYPE_E,                // Invalid line type
        SERIA_EXCEPTION_SURFACE_TYPE_E,             // Invalid surface type
        SERIA_EXCEPTION_VOLUME_TYPE_E,              // Invalid volume type
        SERIA_EXCEPTION_EQUATION_TYPE_E,            // Invalid equation type
        SERIA_EXCEPTION_VERSION_E,                  // Invalid version number
        SERIA_EXCEPTION_SIZE_OVERFLOW_E,            // Container size overflow
        SERIA_EXCEPTION_INVALID_PAYLOAD_TYPE_E,     // Invalid snippet PAYLOAD type
        SERIA_EXCEPTION_LOGIC_POINT_TYPE_E,         // Invalid logic point type
        SERIA_EXCEPTION_SCHEMA_TYPE_E,              // Invalid schema type
        SERIA_EXCEPTION_LINE_COLOR_TYPE_E,          // Invalid line color type
        SERIA_EXCEPTION_LM_VOLUEM_TYPE_E,           // Invalid landmark volume type
        SERIA_EXCEPTION_LM_SHAPE_TYPE_E,            // Invalid landmark shape type
        SERIA_EXCEPTION_TVM_DRIVER_VIEW_LABEL_E,    // Invalid landmark semantic
        SERIA_EXCEPTION_REF_ATTRIBUTE_E,            // Invalid reference attribute
        SERIA_EXCEPTION_DB_POINT_TYPE_E,            // Invalid lane mark type
        SERIA_EXCEPTION_EVP_ATTRIBUTE_TYPE_E,       // Invalid evp attribute type
        SERIA_EXCEPTION_MAX_E
    } ;

    RdbSeriaException(SERIA_EXCEPTION_E c, const char * pInfo1 = NULL,const char * pInfo2 = NULL) noexcept;

    RdbSeriaException(const RdbSeriaException &) noexcept;

    virtual  ~RdbSeriaException(){};

    virtual  const char * what() const noexcept
    {
        return buffer_;
    }

protected:
    uint32_t append(uint32_t pos, const char *pInfo);

    RdbSeriaException() noexcept: eCode_(SERIA_NO_EXCEPTION){}

private:
    char buffer_[128];
    SERIA_EXCEPTION_E eCode_;
};

template<typename CharT, typename Traits = std::char_traits<CharT> >
class RdbStreamBufAccess : public std::basic_streambuf<CharT, Traits>
{
public:
    virtual int sync()
    {
        return this->std::basic_streambuf<CharT, Traits>::sync();
    }
};

template<typename Seria, typename T>
void serialize(Seria &seria, T &data);

inline void RdbReverse2Bytes(char *pAddress)
{
    char temp = *pAddress;

    *pAddress = *(pAddress + 1);
    *(pAddress + 1) = temp;
}

inline void RdbReverse4Bytes(char *pAddress)
{
    char temp = *pAddress;

    *pAddress = *(pAddress + 3);
    *(pAddress + 3) = temp;

    temp = *(pAddress + 1);
    *(pAddress + 1) = *(pAddress + 2);
    *(pAddress + 2) = temp;
}

inline void RdbReverse8Bytes(char *pAddress)
{
    char temp = *pAddress;

    *pAddress = *(pAddress + 7);
    *(pAddress + 7) = temp;

    temp = *(pAddress + 1);
    *(pAddress + 1) = *(pAddress + 6);
    *(pAddress + 6) = temp;

    temp = *(pAddress + 2);
    *(pAddress + 2) = *(pAddress + 5);
    *(pAddress + 5) = temp;

    temp = *(pAddress + 3);
    *(pAddress + 3) = *(pAddress + 4);
    *(pAddress + 4) = temp;
}

}// namespace rdbSerialization
} //namespace roadDBCore



#endif // RDB_SERIA_COMMON_H

