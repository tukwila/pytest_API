/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental AG. 2017
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   File typeDef.h
 * @brief  type define file
 *******************************************************************************
 */

#ifndef TYPEDEF_H
#define TYPEDEF_H

#include <stdint.h>
#include <iostream>

#if defined WIN32 || defined _WIN32
    #include <windows.h>
#elif defined __MACH__ && defined __APPLE__
    #include <time.h>
    #include <mach/mach_time.h>
    #include <mach/clock.h>
    #include <mach/mach.h>
#else
    #include <ctime>
    #include <sys/time.h>
    #include <cmath>
#endif


namespace roadDBCore
{
/****************************************************************************************\
*                         Platform And Language Related Macro                           *
\****************************************************************************************/

#if defined WIN32 || defined _WIN32
#  define RDB_CDECL __cdecl
#  define RDB_STDCALL __stdcall
#else
#  define RDB_CDECL
#  define RDB_STDCALL
#endif

#ifndef RDB_EXTERN_C
#  ifdef __cplusplus
#    define RDB_EXTERN_C extern "C"
#    define RDB_DEFAULT(val) = val
#  else
#    define RDB_EXTERN_C
#    define RDB_DEFAULT(val)
#  endif
#endif

#ifndef RDB_EXTERN_C_FUNCPTR
#  ifdef __cplusplus
#    define RDB_EXTERN_C_FUNCPTR(x) extern "C" { typedef x; }
#  else
#    define RDB_EXTERN_C_FUNCPTR(x) typedef x
#  endif
#endif

#ifndef RDB_INLINE
#  if defined __cplusplus
#    define RDB_INLINE inline
#  elif defined _MSC_VER
#    define RDB_INLINE __inline
#  else
#    define RDB_INLINE static
#  endif
#endif /* RDB_INLINE */

#if (defined WIN32 || defined _WIN32 || defined WINCE) && defined CVAPI_EXPORTS
#  define RDB_EXPORTS __declspec(dllexport)
#else
#  define RDB_EXPORTS
#endif

#ifndef RDB_API
#  define RDB_API(rettype) RDB_EXTERN_C RDB_EXPORTS rettype RDB_CDECL
#endif


/****************************************************************************************\
*                                      Types Definition                                 *
\****************************************************************************************/
typedef char                char_t;
typedef signed char         int8_t;
typedef short                int16_t;
typedef int                 int32_t;

typedef unsigned char        uint8_t;
typedef unsigned short        uint16_t;
typedef unsigned int        uint32_t;

typedef ::int64_t            int64_t;
typedef ::uint64_t            uint64_t;

typedef float               float32_t;
typedef double              float64_t;
typedef long double         float128_t;

typedef int32_t                SegmentID_t;

#ifdef USING_DOUBLE_TYPE

using algo_float_t = roadDBCore::float64_t;

#else

using algo_float_t = roadDBCore::float32_t;

#endif

/************************************ Point2D, Point3D ****************************************/

const char VERSION_INFO[] = "2021.0105.3798\n";

template<typename T>
class Point2D
{
public:
    Point2D();
    Point2D(T _x, T _y);
    Point2D(const Point2D & point);

public:
    T x;
    T y;
};

template<typename T>
class Point3D
{
public:
    Point3D();
    Point3D(T _x, T _y, T _z);
    Point3D(const Point3D & point);

public:
    T x;
    T y;
    T z;
};

template <typename T>
RDB_INLINE Point2D<T>::Point2D() : x(0),y(0) {}

template <typename T>
RDB_INLINE Point2D<T>::Point2D(T _x, T _y) : x(_x),y(_y) {}

template <typename T>
RDB_INLINE Point2D<T>::Point2D(const Point2D & point) : x(point.x),y(point.y) {}


template <typename T>
RDB_INLINE Point3D<T>::Point3D() : x(0),y(0),z(0) {}

template <typename T>
RDB_INLINE Point3D<T>::Point3D(T _x, T _y, T _z) : x(_x),y(_y),z(_z) {}

template <typename T>
RDB_INLINE Point3D<T>::Point3D(const Point3D & point) : x(point.x),y(point.y),z(point.z) {}

typedef Point2D<float32_t> Pnt2f;
typedef Point2D<float64_t> Pnt2d;

typedef Point3D<float32_t> Pnt3f;
typedef Point3D<float64_t> Pnt3d;


/****************************************************************************************\
*                                 Macros and Variables                           *
\****************************************************************************************/
#ifndef NULL
#   define NULL  ((void*)0)
#endif

#ifndef FALSE
#   define FALSE (false)
#endif

#ifndef TRUE
#   define TRUE  (true)
#endif

#define IN
#define OUT
#define INOUT

const uint8_t NUM_BITS_IN_BYTE = 8;
const uint8_t NUM_BYTES_IN_INT8 = sizeof(int8_t);
const uint8_t NUM_BYTES_IN_INT16 = sizeof(int16_t);
const uint8_t NUM_BYTES_IN_INT32 = sizeof(int32_t);
const uint8_t NUM_BYTES_IN_INT64 = sizeof(int64_t);

const uint8_t ON = 1;
const uint8_t OFF = 0;

enum IMAGE_MODEL_E
{
    IMAGE_MODEL_YUV420_E = 0,
    IMAGE_MODEL_RGB_E,
    IMAGE_MODEL_BGR_E,
    IMAGE_MODEL_MAX_E
};

enum SCALE_E: uint8_t
{
    SCALE_NO_E = 0,
    SCALE_7_8_E,
    SCALE_3_4_E,
    SCALE_5_8_E,
    SCALE_1_2_E,
    SCALE_3_8_E,
    SCALE_1_4_E,
    SCALE_1_8_E,
    SCALE_MAX_E
};

enum NORM_CAMERA_TYPE_E: uint8_t
{
    NORM_CAMERA_TYPE_65 = 0,        // camera 65
    NORM_CAMERA_TYPE_127_E,         // camera 127
    NORM_CAMERA_TYPE_IPHONE6_E,     // camera iphone6
    NORM_CAMERA_TYPE_65_GM_E,       // camera 65 for GM
    NORM_CAMERA_TYPE_65_CES_E,      // camera 65 for CES
    NORM_CAMERA_TYPE_MAX_E
};

/****************************************************************************************\
*                                      Inline functions                                 *
\****************************************************************************************/
#ifndef MIN
template <typename T> RDB_INLINE
T MIN(T a, T b)
{
    return (a > b ? b : a);
}
#endif

#ifndef MAX
template <typename T> RDB_INLINE
T MAX(T a, T b)
{
    return (a < b ? b : a);
}
#endif


/**************************************** Tick *****************************************/

/*!
  The function returns the number of ticks since the certain event (e.g. when the machine was turned on).
  It can be used to initialize random or to measure a function execution time by reading the tick count
  before and after the function call. The granularity of ticks depends on the hardware and OS used. Use
  rdbGetTickFrequency() to convert ticks to seconds.
*/
RDB_INLINE int64_t rdbGetTickCount(void)
{
#if defined WIN32 || defined _WIN32 || defined WINCE
    LARGE_INTEGER counter;
    QueryPerformanceCounter( &counter );
    return (int64)counter.QuadPart;
#elif defined __linux || defined __linux__
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    return (int64_t)tp.tv_sec*1000000000 + tp.tv_nsec;
#elif defined __MACH__ && defined __APPLE__
    return (int64_t)mach_absolute_time();
#else
    struct timeval tv;
    struct timezone tz;
    gettimeofday( &tv, &tz );
    return (int64_t)tv.tv_sec*1000000 + tv.tv_usec;
#endif
}

/*!
  Returns the number of ticks per seconds.

  The function returns the number of ticks (as returned by cv::rdbGetTickCount()) per second.
  The following code computes the execution time in milliseconds:

  \code
  float64_t exec_time = (float64_t)rdbGetTickCount();
  // do something ...
  exec_time = ((float64_t)rdbGetTickCount() - exec_time)*1000./rdbGetTickFrequency();
  \endcode
*/
RDB_INLINE float64_t rdbGetTickFrequency(void)
{
#if defined WIN32 || defined _WIN32 || defined WINCE
    LARGE_INTEGER freq;
    QueryPerformanceFrequency(&freq);
    return (double)freq.QuadPart;
#elif defined __linux || defined __linux__
    return 1e9;
#elif defined __MACH__ && defined __APPLE__
    static double freq = 0;
    if( freq == 0 )
    {
        mach_timebase_info_data_t sTimebaseInfo;
        mach_timebase_info(&sTimebaseInfo);
        freq = sTimebaseInfo.denom*1e9/sTimebaseInfo.numer;
    }
    return freq;
#else
    return 1e6;
#endif
}


/****************************************************************************************\
*                                  Wrap System Related API                             *
\****************************************************************************************/

/************************************ System Time ***************************************/
typedef struct tagRDBSysTime_t
{
    uint32_t    year;
    uint32_t    month;
    uint32_t    day;
    uint32_t    hour;
    uint32_t    minute;
    uint32_t    second;

    // Added by Caixing Song: support millisecond
    uint32_t    millisecond;
}RDBSysTime_t;

RDB_INLINE void RDBGetLocalTime(RDBSysTime_t &localTime)
{
#if defined WIN32 || defined _WIN32
    SYSTEMTIME sys;
    GetLocalTime( &sys );

    localTime.year = sys.wYear;
    localTime.month = sys.wMonth;
    localTime.day = sys.wDay;
    localTime.hour = sys.wHour;
    localTime.minute = sys.wMinute;
    localTime.second = sys.wSecond;

#else
    time_t sysnow;
    struct tm *sys;

#if defined __MACH__ && __APPLE__ // OS X does not have clock_gettime, use clock_get_time
    clock_serv_t cclock;
    mach_timespec_t mts;
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);

    localTime.millisecond = static_cast<uint32_t>(mts.tv_nsec / 1.6e6);
#else
    struct timespec timeSpec;

    // Try to use clock_gettime() to obtain better time accuracy.
    // On failure, use time() instead. In this case, no millisecond will be obtained.
    if ( 0 == clock_gettime(CLOCK_REALTIME, &timeSpec) )
    {
        sysnow = timeSpec.tv_sec;
        localTime.millisecond = static_cast<uint32_t>(round((double)(timeSpec.tv_nsec) / 1.0e6));
    }
    else
    {
        time(&sysnow);

        localTime.millisecond = 0;
    }
#endif

    sys = localtime(&sysnow);

    localTime.year = sys->tm_year + 1900;
    localTime.month = sys->tm_mon + 1;
    localTime.day = sys->tm_mday;
    localTime.hour = sys->tm_hour;
    localTime.minute = sys->tm_min;
    localTime.second = sys->tm_sec;

#endif
}

enum TSErrorCode : int32_t {
    e_NOERROR                  = 0,  /**< No error >*/
    e_ERROR_FAILED             = -1,  /**< common failed >*/
    e_ERROR_INVALID            = -2,  /**< invalid parameters >*/
    e_ERROR_NULL_POINTER       = -3,   /**< The input has NULL pointers >*/
    e_ERROR_MEMORY             = -4,  /**< memory allocing error >*/
    e_ERROR_SETTING_FAILED     = -5,  /**< setting failed >*/
    e_ERROR_INIT_FAILED        = -6,  /**< init failed >*/
    e_ERROR_DETECT_FAILED      = -7,  /**< detect failed >*/
    e_ERROR_TRACK_FAILED       = -8,  /**< track failed >*/
    e_ERROR_CLASSIFY_FAILED    = -9,  /**< classify failed >*/
    e_ERROR_RECONSTRUCT_FAILED = -10,  /**< reconstruct failed >*/
    e_ERROR_LIB_FAILED         = -11,  /**< load or call library funtions failed >*/
};

enum DB_POINT_TYPE_E : uint8_t
{
    DB_POINT_TYPE_MP_E = 0,    // map point
    DB_POINT_TYPE_VOXEL_E,     // voxel point
    DB_POINT_TYPE_PAINT_E,     // paint point
    DB_POINT_TYPE_MAX_E
};

const uint32_t RDB_VERSION_MAIN = 0X00040000;


} /* end of namespace roadDBCore */

#endif /* TYPEDEF_H */

