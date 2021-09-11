#!/usr/bin/python3
# encoding=utf-8
import pytest
import sys, os
import time
import json
sys.path.append('./')
sys.path.append('..')
from config import Configs
from common.PyVehicleDataLoader import *
conf = Configs.config()
DBPath = conf.get('DB', 'dockerDBPath')

testDB = {
    'i696': {
        'db': DBPath + 'USi696_vehicleDB',
        'ArgsFile': os.getcwd() + '/Params/Args_i696.json'
    },
    'mmg': {
        'db': DBPath + 'mmg_vehicleDB',
        'ArgsFile': os.getcwd() + '/Params/Args_mmg.json'
    },
    'detroit': {
        'db': DBPath + 'Detroit_vehicleDB',
        'ArgsFile': os.getcwd() + '/Params/Args_detroit.json'
    }
}


def vehicleDB_loader_realmode(constructor, updatePosition, nextDivisionIDs, segments):
    dictPars = {}

    def updatePostionCallback(dictPars) -> None:
        temp = []
        for i in dictPars.keys():
            temp.append(i)
        if set(divisionIdList) <= set(temp):
            assert set(divisionIdList) <= set(temp), \
                ('----------------updatePosition with callback, divisionIds are not include in dictPars, point: ',
                 postion.relLon)

    conf = vehicle_conf_t()
    conf.dbPath = constructor['dbPath']
    conf.cacheMode = CACHE_MODE_E.CACHE_MODE_REALTIME_E
    conf.initialPosition.lon = constructor['initPosition'][0]
    conf.initialPosition.lat = constructor['initPosition'][1]
    conf.initialPosition.alt = constructor['initPosition'][2]
    # conf.tolerantTime = constructor['tolerantTime']
    print('>>>>>>>>>>>>>>>>>>Test start >>>>>>>>>>>>>>>>>')
    loader = PyVehicleDataLoader(conf)
    loader.initLog("vehiclDB", "loader", LOG_LEVEL.enum_ERROR)
    return_code = loader.start()
    assert return_code is True, ("start fail; return code: ", return_code)
    time.sleep(1)
    cachedSegIDs = loader.getCachedSegmentIDs()
    print("----------------start cachedSegIDs: ", cachedSegIDs)
    print('>>>>>>>>>>>>>>>>>>Test updatePosition >>>>>>>>>>>>>>>>>')
    postion = RPoint3D_t()
    for i in updatePosition:
        postion.relLon = i['position'][0]
        postion.relLat = i['position'][1]
        postion.relAlt = i['position'][2]
        divisionIdList = i['divisionId']
        layer = i['layer']
        return_code = loader.updatePosition(divisionIdList, layer, postion)
        assert return_code == 0, ("updatePosition: fail; return code: ", return_code)

        time.sleep(10)
        cachedSegIDs = loader.getCachedSegmentIDs()
        # print('point ', n, ': ', cachedSegIDs, 'in position: ',  postion.relLon)
        SDKSegments = []
        for j in cachedSegIDs:
            if j in segments:
                SDKSegments.append(j)
        SDKSegments = sorted(SDKSegments)
        # print('>>>>>valid Segments: ', SDKSegments)
        getCachedSegmentIDs = sorted(i['expectedResult']['getCachedSegmentIDs'])
        assert SDKSegments == getCachedSegmentIDs, \
            ("----------------updatePosition getCachedSegmentIDs fail, SDK return: ", SDKSegments, \
             "expect: ", getCachedSegmentIDs, 'position: ', postion.relLon)
        divisionIDsRlt = loader.getPassedDivisionIDs(cachedSegIDs)
        assert divisionIDsRlt.errCode == 0, \
            ("RealTime updatePosition getPassedDivisionIDs: fail; return code: ", divisionIDsRlt.errCode)
        assert divisionIDsRlt.divisionIds == set(i['expectedResult']['getPassedDivisionIDs']), \
            ("----------------updatePosition getPassedDivisionIDs fail, SDK return: ", divisionIDsRlt.divisionIds, \
             "expect: ", i['expectedResult']['getPassedDivisionIDs'], 'position: ', postion.relLon)
        divisionDetailResult = loader.getDivisionDetails(list(divisionIDsRlt.divisionIds))
        assert divisionDetailResult.errCode == 0, \
            ("RealTime updatePosition getDivisionDetails: fail; return code: ", divisionDetailResult.errCode, \
             'position: ', postion.relLon)
        DivisionTrajectoriesResult = loader.getDivisionTrajectories(list(divisionIDsRlt.divisionIds))
        # getDivisionTrajectories return error code in realtime mode
        assert DivisionTrajectoriesResult.errCode != 0, \
            ("RealTime updatePosition getDivisionTrajectories: fail; return code: ", DivisionTrajectoriesResult.errCode, \
             'position: ', postion.relLon)
    loader.clearBuffer()
    cachedSegIDs = loader.getCachedSegmentIDs()
    assert cachedSegIDs == set(), ('----------------clearBuffer cachedSegIDs is not empty')
    print('>>>>>>>>>>>>>>>>>>Test updatePosition with callback >>>>>>>>>>>>>>>>>')
    return_code = loader.start()
    assert return_code is True, ("start fail; return code: ", return_code)
    postion = RPoint3D_t()
    for i in updatePosition:
        postion.relLon = i['position'][0]
        postion.relLat = i['position'][1]
        postion.relAlt = i['position'][2]
        divisionIdList = i['divisionId']
        # divisionIdList = []
        layer = i['layer']
        # print('---divisionId: ', divisionIdList)
        return_code = loader.updatePosition(divisionIdList, layer, postion, updatePostionCallback)
        assert return_code == 0, ("updatePosition with callback: fail; return code: ", return_code)
        time.sleep(5)
    loader.clearBuffer()
    print('>>>>>>>>>>>>>>>>>>Test getNextDivisionIDs >>>>>>>>>>>>>>>>>')
    for i in nextDivisionIDs:
        conf.initialPosition.lon = i['initPosition'][0]
        conf.initialPosition.lat = i['initPosition'][1]
        conf.initialPosition.alt = i['initPosition'][2]
        return_code = loader.start()
        assert return_code is True, ("start: fail; return code: ", return_code)
        nodeID = i['nodeId']
        nextDivisionIDsRlt = loader.getNextDivisionIDs(nodeID)
        if i['ExpectedResult'] != 541077506:
            assert nextDivisionIDsRlt.errCode == 0, \
                ("nodeID: ", nodeID, "RealTime getNextDivisionIDs: fail; return code: ", nextDivisionIDsRlt.errCode)
            assert i['ExpectedResult'] in nextDivisionIDsRlt.divisionIds and len(nextDivisionIDsRlt.divisionIds) == 1, \
                ("nodeID: ", nodeID, "RealTime getNextDivisionIDs: fail; SDK return divisionID: ",
                 nextDivisionIDsRlt.divisionIds)
        else:
            assert nextDivisionIDsRlt.errCode == i['ExpectedResult'], \
                ("nodeID: ", nodeID, "RealTime getNextDivisionIDs: fail; return code: ", nextDivisionIDsRlt.errCode)
        loader.clearBuffer()

def test_vehicleLoaderRealTimeMode():
    for i in testDB.keys():
        print('.....................Test DB: ', i, '.................')
        with open(testDB[i]['ArgsFile'], 'r') as f:
            requestDict = json.load(f)
        constructor = {}
        constructor['dbPath'] = requestDict['constructor']['dbPath']
        constructor['tolerantTime'] = requestDict['constructor']['tolerantTime']
        constructor['initPosition'] = requestDict['constructor']['initPosition']
        segments = requestDict['segments']
        # divisions = requestDict['divisions']
        updatePosition = requestDict['updatePosition']
        nextDivisionIDs = requestDict['nextDivisionIDs']
        vehicleDB_loader_realmode(constructor, updatePosition, nextDivisionIDs, segments)