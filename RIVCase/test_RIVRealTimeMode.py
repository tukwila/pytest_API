#!/usr/bin/python3
# encoding=utf-8
import sys, os
import json
import time
#import pytest
sys.path.append('./')
sys.path.append('./RIVCase')
from test_RIVWholeMode import getDivAndSeg
from test_RIVWholeMode import riv_mngr_whole_mode_getAllDivisionIDs
from test_RIVWholeMode import riv_mngr_whole_mode_getPassRoadIDs
import random

sys.path.append('..')
from config import Configs
from common import PyRIVLogicMgr
conf = Configs.config()
DBPath = conf.get('DB', 'dockerDBPath')
testDB = {
    'mmg': {
        'db': DBPath + 'fw-721/',
        'GPSFile': os.getcwd() + '/Params/GPSFileRT_mmg.json',
        'proto_ver': 0.14
    }
}
testDB11 = {
    'fuji': {
        'db': DBPath + 'Fujiv14/',
        'GPSFile': os.getcwd() + '/Params/GPSFileRT_fuji.json',
        'proto_ver': 0.14
    },
    'mmg': {
        'db': DBPath + 'mmgv14_roaddbPB/',
        'GPSFile': os.getcwd() + '/Params/GPSFileRT_mmg.json',
        'proto_ver': 0.14
    }
}


def riv_mngr_realtime_mode_updatePosition_getWholeHorizon(conf_path, GPSFile, segmentList, divisionList):
    def getAllObjects(obj):
        objList = []
        for o in obj:
            objList.append(o.getID())
        objList = sorted(objList)
        return objList

    gpsFile = GPSFile
    mngr = PyRIVLogicMgr.RIVLogicMgr()
    conf = PyRIVLogicMgr.conf_t()
    conf.dbPath = conf_path
    conf.dbType_ = PyRIVLogicMgr.EDBType.E_DB_TYPE_PB
    conf.cacheMode = PyRIVLogicMgr.CACHE_MODE_E.CACHE_MODE_REALTIME_E
    conf.minGetLayer = 1
    conf.dataCheckMode = PyRIVLogicMgr.EXCEPTION_DATA_HANDLE_MODE.EDH_MODE_ACCEPTED
    conf.refreshConf.cacheLayerCount = 1
    conf.refreshConf.minRefreshLayerIndex = 1
    conf.refreshConf.gpsCheckTimeInterval = 2000
    mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_ERROR)
    return_code = mngr.initWithConf(conf)
    assert return_code == 0, ("initWithConf: fail; return code: ", return_code)
    with open(gpsFile, 'r') as f:
        gpsFile = json.load(f)
    lastTimeRoad = []
    lastTimeJunction = []
    lastTimegetDataBySegment = []
    lastTimegetDataByDivision = []
    iterationCount = 0
    for i in gpsFile:
        iterationCount = iterationCount + 1
        pos = PyRIVLogicMgr.WGS84_t()
        pos.lon = i[0]
        pos.lat = i[1]
        pos.alt = i[2]
        ret = mngr.updatePosition(pos)
        # print('---------------------iterationCount: ', iterationCount)
        # print('position: ', pos.lon, 'updatePosition ret: ', ret)
        time.sleep(20)
        horizon_result = mngr.getWholeHorizon()
        roadList = []
        juncionList = []
        roadList = getAllObjects(horizon_result.spHorizon.getLandscapes())
        #juncionList = getAllObjects(horizon_result.spHorizon.getJunctions())
        # for r in horizon_result.spHorizon.getLandscapes():
        #     roadList.append(r.getID())
        # roadList = sorted(roadList)
        for j in horizon_result.spHorizon.getJunctions():
            juncionList.append(j.getID())
        juncionList = sorted(juncionList)
        # compare content of mem this iteration with the last iteration
        #print('junction list: ', juncionList)
        compareRoadResult = list(set(roadList).difference(set(lastTimeRoad)))
        compareRoadResult = sorted(compareRoadResult)
        compareJunctionResult = list(set(juncionList).difference(set(lastTimeJunction)))
        compareJunctionResult = sorted(compareJunctionResult)

        # if iterationCount != 1:
        #     print('roads this iteration compare with the last: ', compareRoadResult)
        #     print('junction this iteration compare with the last: ', compareJunctionResult)
        lastTimeRoad = roadList
        lastTimeJunction = juncionList

        # test getDataBySegment() in realtime mode
        horizonRes = mngr.getDataBySegment(segmentList)
        assert horizonRes.errorCode == 0, ("getDataBySegment API returned error code:", horizonRes.errorCode)
        assert horizonRes.spHorizon != None, "getDataBySegment spHorizon is null"
        roadListInSeg = getAllObjects(horizonRes.spHorizon.getLandscapes())
        # if iterationCount != 1:
        #     print('getDataBySegment this iteration compare with the last: ', \
        #       set(roadListInSeg).difference(set(lastTimegetDataBySegment)))
        # lastTimegetDataBySegment = roadListInSeg
        # check roads in getDataBySegment()
        assert sorted(roadListInSeg) == roadList, ('getDataBySegment result is different with roadList')

        # test getDataByDivision() in realtime mode
        horizonResu = mngr.getDataByDivision(divisionList)
        assert horizonResu.errorCode == 0, ("getDataByDivision API returned error code:", horizonResu.errorCode)
        assert horizonResu.spHorizon != None, "getDataByDivision spHorizon is null"
        roadListInDiv = getAllObjects(horizonResu.spHorizon.getLandscapes())
        # if iterationCount != 1:
        #     print('getDataByDivision this iteration compare with the last: ', \
        #       set(roadListInDiv).difference(set(lastTimegetDataByDivision)))
        # lastTimegetDataByDivision = roadListInDiv
        assert sorted(roadListInDiv) == roadList, ('getDataByDivision result is different with roadList')

    mngr.destroy()


def minRefreshLayerIndex():
    print('>>>>>>>>>>>>>>>>>>Test minRefreshLayerIndex>>>>>>>>>>>>>>>>>')
    mngr = PyRIVLogicMgr.RIVLogicMgr()
    mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_ERROR)
    conf = PyRIVLogicMgr.conf_t()
    conf.dbPath = DBPath + 'mmgv14_roaddbPB/'
    conf.dbType_ = PyRIVLogicMgr.EDBType.E_DB_TYPE_PB
    conf.cacheMode = PyRIVLogicMgr.CACHE_MODE_E.CACHE_MODE_REALTIME_E
    conf.refreshConf.cacheLayerCount = 3
    conf.refreshConf.minRefreshLayerIndex = 2
    conf.refreshConf.gpsCheckTimeInterval = 0
    mngr.initWithConf(conf)
    pos = PyRIVLogicMgr.WGS84_t()
    pos.lon = 10.157267
    pos.lat = 47.997759
    pos.alt = 0
    mngr.updatePosition(pos)
    time.sleep(20)
    horizon_result = mngr.getWholeHorizon()
    roadList = []
    if horizon_result.errorCode == 0 and horizon_result.spHorizon != None:
        for r in horizon_result.spHorizon.getLandscapes():
            roadList.append(r.getID())
        roadList = sorted(roadList)

    pos = PyRIVLogicMgr.WGS84_t()
    pos.lon = 10.131563
    pos.lat = 48.026579
    pos.alt = 0
    mngr.updatePosition(pos)
    time.sleep(20)
    horizon_result = mngr.getWholeHorizon()
    roadList1 = []
    if horizon_result.errorCode == 0 and horizon_result.spHorizon != None:
        for r in horizon_result.spHorizon.getLandscapes():
            roadList1.append(r.getID())
        roadList1 = sorted(roadList1)
    assert roadList1 != roadList, 'pos.lon: 10.157267 roadlist == pos.lon: 10.131563 roadList1'

    pos = PyRIVLogicMgr.WGS84_t()
    pos.lon = 10.131478
    pos.lat = 48.033636
    pos.alt = 0
    mngr.updatePosition(pos)
    time.sleep(20)
    horizon_result = mngr.getWholeHorizon()
    roadList2 = []
    if horizon_result.errorCode == 0 and horizon_result.spHorizon != None:
        for r in horizon_result.spHorizon.getLandscapes():
            roadList2.append(r.getID())
        roadList2 = sorted(roadList2)
        assert roadList2 == roadList1, 'pos.lon: 10.131478 roadlist2 != pos.lon: 10.131563 roadlist1'

def riv_mngr_realtime_mode_test():
    print('>>>>>>>>>>>>>>>>>>Start riv_mngr_realtime_mode_test >>>>>>>>>>>>>>>>>')
    for i in testDB.keys():
        print('>>>>>>>>>>>>>>>>>>db under test: ', i, '>>>>>>>>>>>>>>>>>')
        divisionList, segmentList = getDivAndSeg(testDB[i]['db'], testDB[i]['proto_ver'])
        riv_mngr_realtime_mode_updatePosition_getWholeHorizon(testDB[i]['db'], testDB[i]['GPSFile'], segmentList, set(
            divisionList))
        # getAllDivisionIDs/getPassRoadIDs/getDataVersion return result are same in both realtime/whole mode
        #riv_mngr_whole_mode_getAllDivisionIDs(testDB[i]['db'], testDB[i]['proto_ver'])
        #riv_mngr_whole_mode_getPassRoadIDs(testDB[i]['db'], segmentList, testDB[i]['proto_ver'])
        print('>>>>>>>>>>>>>>>>>>End riv_mngr_realtime_mode_test >>>>>>>>>>>>>>>>>')

    minRefreshLayerIndex()

# if __name__ == '__main__':
#     riv_mngr_realtime_mode_test()


def test_RIVRealTimeMode():
    riv_mngr_realtime_mode_test()

if __name__=='__main__':
    test_RIVRealTimeMode()

