#!/usr/bin/python3
# encoding=utf-8

import pytest
import sys
import time
sys.path.append('./')
sys.path.append('..')
from config import Configs
from common.PyVehicleDataLoader import *
conf = Configs.config()
DBPath = conf.get('DB', 'dockerDBPath')

vehicleDB = {
    'correct': [DBPath + 'USi696_vehicleDB',
                DBPath + 'fake1_USi696_vehicleDB'
    ],
    'lackvehicldDB': [DBPath + 'fake2_USi696_vehicleDB'],
    'timestamp': [
        DBPath + 'fake3_USi696_vehicleDB',
        DBPath + 'fake4_USi696_vehicleDB'
    ]
}
def test_vehicleDBload():
    for i in vehicleDB['correct']:
        print('---------------TestDB correct Path in WHOLE: ', i)
        conf = vehicle_conf_t()
        conf.dbPath = i
        conf.cacheMode = CACHE_MODE_E.CACHE_MODE_WHOLE_E
        loader = PyVehicleDataLoader(conf)
        loader.initLog("vehiclDBload", "loader", LOG_LEVEL.enum_ERROR)
        return_code = loader.start()
        assert return_code is True, ("start: fail; return code: ", return_code)
        loader.clearBuffer()
    for j in vehicleDB['lackvehicldDB']:
        print('---------------TestDB notcorrect Path in WHOLE: ', j)
        conf = vehicle_conf_t()
        conf.dbPath = j
        conf.cacheMode = CACHE_MODE_E.CACHE_MODE_WHOLE_E
        loader = PyVehicleDataLoader(conf)
        loader.initLog("vehiclDBload", "loader", LOG_LEVEL.enum_WARN)
        return_code = loader.start()
        assert return_code is True, ("start: fail; return code: ", return_code)
        loader.clearBuffer()
    for k in vehicleDB['timestamp']:
        print('---------------TestDB notcorrect Path in WHOLE: ', k)
        conf = vehicle_conf_t()
        conf.dbPath = k
        conf.cacheMode = CACHE_MODE_E.CACHE_MODE_WHOLE_E
        loader = PyVehicleDataLoader(conf)
        loader.initLog("vehiclDBload", "loader", LOG_LEVEL.enum_WARN)
        return_code = loader.start()
        assert return_code is False, ("start: fail; return code: ", return_code)
        loader.clearBuffer()

    time.sleep(5)
    print('\n' * 5)

    for i in vehicleDB['correct']:
        print('---------------TestDB correct Path in REALTIME: ', i)
        conf = vehicle_conf_t()
        conf.dbPath = i
        conf.cacheMode = CACHE_MODE_E.CACHE_MODE_REALTIME_E
        conf.initialPosition.lon = -83.140616
        conf.initialPosition.lat = 42.475209
        conf.initialPosition.alt = 0
        loader = PyVehicleDataLoader(conf)
        loader.initLog("vehiclDBload", "loader", LOG_LEVEL.enum_ERROR)
        return_code = loader.start()
        assert return_code is True, ("start: fail; return code: ", return_code)
        loader.clearBuffer()
    for j in vehicleDB['lackvehicldDB']:
        print('---------------TestDB notcorrect Path in REALTIME: ', j)
        conf = vehicle_conf_t()
        conf.dbPath = j
        conf.cacheMode = CACHE_MODE_E.CACHE_MODE_REALTIME_E
        conf.initialPosition.lon = -83.140616
        conf.initialPosition.lat = 42.475209
        conf.initialPosition.alt = 0
        loader = PyVehicleDataLoader(conf)
        loader.initLog("vehiclDBload", "loader", LOG_LEVEL.enum_WARN)
        return_code = loader.start()
        assert return_code is True, ("start: fail; return code: ", return_code)
        loader.clearBuffer()
    for k in vehicleDB['timestamp']:
        print('---------------TestDB notcorrect Path in REALTIME: ', k)
        conf = vehicle_conf_t()
        conf.dbPath = k
        conf.cacheMode = CACHE_MODE_E.CACHE_MODE_REALTIME_E
        conf.initialPosition.lon = -83.140616
        conf.initialPosition.lat = 42.475209
        conf.initialPosition.alt = 0
        loader = PyVehicleDataLoader(conf)
        loader.initLog("vehiclDBload", "loader", LOG_LEVEL.enum_WARN)
        return_code = loader.start()
        assert return_code is False, ("start: fail; return code: ", return_code)
        loader.clearBuffer()

# if __name__ == '__main__':
#     print('---------------Test vehicleDB -------------------')
#     test_vehicleDBload()
