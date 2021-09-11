/**
*******************************************************************************
*                       Continental Confidential
*                  Copyright (c) Continental AG. 2016-2017
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
******************************************************************************
* @file   SensorTagger.h
* @brief  Declaration of SensorTagger
*******************************************************************************
*/
#pragma once

#include "sensorDataType.h"
#include "RoadDBVideoFileReader.h"
#include "sensorFormatStreamReader.h"
#include "GpsTagger.h"
#include "ImuTagger.h"
#include "ObdTagger.h"
#include <stdint.h>
#include <unistd.h>
#include <vector>
#include <memory>
#include <string>
#include <fstream>
#include <tuple>
#include "log.h"

using namespace libSensor;

enum SENSOR_TAGGER_GET_TYPE_E
{
	SENSOR_TAGGER_GET_TYPE_GPS_E = 0,
	SENSOR_TAGGER_GET_TYPE_IMU_E,
	SENSOR_TAGGER_GET_TYPE_OBD_E
};

class SensorTagger
{
public:
    /**
     *******************************************************************************
     * @brief constructor
     *
     *  <1> Parameter Description:
     *
     *  @param
     *  videoFileReader   [IN]    a std::share_ptr of video file(img/rtv)
     *  gpsFile           [IN]    input gps file, if no imu file, let it empty
     *  imuFile           [IN]    input imu file, if no imu file, let it empty
     *  obdFile           [IN]    input obd file, if no imu file, let it empty
     *
     *  @return
     *
     *
     *  <2> Detailed Description:
     *  constructor of SensorTagger
     *******************************************************************************
     */
    SensorTagger(std::shared_ptr<RoadDBVideoFileReader> videoFileReader,
            const std::string & gpsFile,
            const std::string & imuFile,
			const std::string & obdFile);
    /**
     *******************************************************************************
     * @brief constructor
     *
     *  <1> Parameter Description:
     *
     *  @param
     *  timeTxtFile       [IN]    input txt file which contains time info(timefile)
     *  gpsFile           [IN]    input gps file, if no imu file, let it empty
     *  imuFile           [IN]    input imu file, if no imu file, let it empty
     *  obdFile           [IN]    input obd file, if no imu file, let it empty
     *
     *  @return
     *
     *
     *  <2> Detailed Description:
     *  constructor of SensorTagger
     *******************************************************************************
     */
    SensorTagger(const std::string & timeTxtFile,
                const std::string & gpsFile,
				const std::string & imuFile,
				const std::string & obdFile);

    /**
     *******************************************************************************
     * @brief init function
     *
     *  <1> Parameter Description:
     *
     *  @param
     *
     *  @return
     *  uint32_t, to show every type of sensor tagged success or fail
     *  details at below:
     * 
     *  #define SENSOR_TAGGER_TAG_SUCCESS_E  (0x0)
     *  #define SENSOR_TAGGER_TAG_GPS_ERR_E  (0x1)
     *  #define SENSOR_TAGGER_TAG_IMU_ERR_E  (0x2)
     *  #define SENSOR_TAGGER_TAG_OBD_ERR_E  (0x4)
     *  #define SENSOR_TAGGER_TAG_ALL_ERR_E  (0x1 | 0x2 | 0x4)
     *  
     *  if the return value is 0, all sensors tagged successed
     *  if the return value is 1, just gps tagged failed
     *  if the return value is 5, gps and obd tagged failed, only imu successed
     *  if the return value is 7, all sensors tagged failed
     * 
     *  <2> Detailed Description:
     *  init
     *******************************************************************************
     */
    uint32_t init();
    /**
     *******************************************************************************
     * @brief uninit
     *
     *  <1> Parameter Description:
     *
     *  @param
     *
     *  @return
     *  void
     *
     *  <2> Detailed Description:
     *  uninit
     *******************************************************************************
     */
    void uninit();

    /**
     *******************************************************************************
     * @brief getData
     *
     *  <1> Parameter Description:
     *
     *  @param
     *  fStart  [IN]  start frame index of the request
     *  fEnd    [IN]  end frame index of the request
     *  tup     [IN]  a std::tuple which contains : I.  vector of sensor data
     *                                              II. one type of data
     *
     *  @return
     *  true, if get all data you want success
     *  false, if get one error occur
     *
     *  <2> Detailed Description:
     *  getData, THE type Tup must be std::tuple
     *******************************************************************************
     */
    template<typename Tup>
    bool getData(uint32_t fStart, uint32_t fEnd, Tup & tup)
    {
        return TupHelper<Tup, std::tuple_size<Tup>::value - 1>::traverseHelper(this, fStart, fEnd, tup);
    }

    /**
     *******************************************************************************
     * @brief getOneTypeData
     *
     *  <1> Parameter Description:
     *
     *  @param
     *  fStart  [IN]  start frame index of the request
     *  fEnd    [IN]  end frame index of the request
     *  tup     [IN]  a std::tuple which contains : I.  vector of sensor data
     *                                              II. one type of data
     *
     *  @return
     *  true, if get all data you want success
     *  false, if get one error occur
     *
     *  <2> Detailed Description:
     *  getOneTypeData, the T can be vector of the sensor you want, 
     *  or just one GpsData(OBDData) if you want to get one frame data, 
     *  on such situation, start must be equal to end
     *******************************************************************************
     */
    template<typename T>
    bool getOneTypeData(uint32_t fStart, uint32_t fEnd, T & data)
    {
        return getDataInner(fStart, fEnd, data);
    }

    /**
     *******************************************************************************
     * @brief getValidTagIndex
     *
     *  <1> Parameter Description:
     *
     *  @param
     *  sensortype  [IN]  sensor type of request, it is gps, imu, or obd
     *	start       [OUT] start index of tagged frame
     *	end         [OUT] end index of tagged frame
     *
     *  @return
     *	true, if the type of sensor is tagged
     *	false, if the type of sensor is not tagged
     *
     *  <2> Detailed Description:
     *  getValidTagIndex [start, end]
     *******************************************************************************
     */
    bool getValidTagIndex(SENSOR_TAGGER_GET_TYPE_E sensortype, uint64_t & start, uint64_t & end);

    /**
     *******************************************************************************
     * @brief destructor
     *
     *  <1> Parameter Description:
     *
     *  @param
     *
     *
     *  @return
     *
     *
     *  <2> Detailed Description:
     *  destructor
     *******************************************************************************
     */
    virtual ~SensorTagger();

private:
    enum SENEOR_TAGGER_TIMEFROM_E
	{
    	SENEOR_TAGGER_TIMEFROM_VIDEO_E = 0,
		SENEOR_TAGGER_TIMEFROM_TXT_E
	};

    template<typename Tup, size_t index>
    class TupHelper
    {
    public:
        static bool traverseHelper(SensorTagger * t, uint32_t start, uint32_t end, Tup & tup)
        {
            bool r = TupHelper<Tup, index-1>::traverseHelper(t, start, end, tup);
            return r && t->getDataInner(start, end, std::get<index>(tup));
        }
    };

    template<typename Tup> class TupHelper<Tup, 0>
    {
    public:
        static bool traverseHelper(SensorTagger * t, uint32_t start, uint32_t end, Tup & tup)
        {
            return t->getDataInner(start, end, std::get<0>(tup));
        }
    };

    bool doTag();
    bool setTimeInfo();
    template<typename T>
    bool getDataInner(uint32_t start, uint32_t end, T & ref);

    std::string gpsFileStr_;
    std::string imuFileStr_;
    std::string obdFileStr_;
    std::string timeTxtFileStr_;

    std::shared_ptr<RoadDBVideoFileReader> videoFileReader_;
    GpsTagger gpsTagger_;
    ImuTagger imuTagger_;
    OBDTagger obdTagger_;

    std::ifstream timeTxtFile_;
    SENEOR_TAGGER_TIMEFROM_E taggerType_;
    std::vector<uint64_t> timeInfo_;
    uint32_t oriVideoFramesCount_;
    uint64_t oriVideoStartTime_;

 };
