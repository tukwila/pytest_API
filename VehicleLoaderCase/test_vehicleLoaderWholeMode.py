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
    }
}

def vehicleDB_loader_wholemode(constructor, updatePosition, nextDivisionIDs):
    conf = vehicle_conf_t()
    conf.dbPath = constructor['dbPath']
    conf.cacheMode = CACHE_MODE_E.CACHE_MODE_WHOLE_E
    # initialPosition: only used in realtime mode
    conf.initialPosition.lon = 0
    conf.initialPosition.lat = 0
    conf.initialPosition.alt = 0
    # conf.tolerantTime = constructor['tolerantTime']
    print('>>>>>>>>>>>>>>>>>>Test start >>>>>>>>>>>>>>>>>')
    loader = PyVehicleDataLoader(conf)
    loader.initLog("vehiclDB", "loader", LOG_LEVEL.enum_ERROR)
    return_code = loader.start()
    assert return_code is True, ("start: fail; return code: ", return_code)
    time.sleep(1)
    cachedSegIDs = loader.getCachedSegmentIDs()
    #print("----------------start cachedSegIDs: ", cachedSegIDs)
    divisionIDsRlt = loader.getPassedDivisionIDs(cachedSegIDs)
    assert divisionIDsRlt.errCode == 0, \
            ("Wholemode getPassedDivisionIDs: fail; return code: ", divisionIDsRlt.errCode)
    #print("----------------getPassedDivisionIDs: ", divisionIDsRlt.divisionIds)
    # in wholemode, getDivisionDetails is not supported
    divisionDetailResult = loader.getDivisionDetails(list(divisionIDsRlt.divisionIds))
    assert divisionDetailResult.errCode != 0, \
            ("Wholemode getDivisionDetails: fail; return code: ", divisionDetailResult.errCode)
    DivisionTrajectoriesResult = loader.getDivisionTrajectories(list(divisionIDsRlt.divisionIds))
    assert DivisionTrajectoriesResult.errCode == 0, \
            ("Wholemode getDivisionTrajectories(divisionIds): fail; return code: ", DivisionTrajectoriesResult.errCode)
    divTraList = []
    for division in DivisionTrajectoriesResult.divisions:
        divTraList.append(division.dbID)
        #print("----------------getDivisionTrajectories: ", division.dbID)
    assert set(divTraList) == divisionIDsRlt.divisionIds, \
        ('Wholemode getDivisionTrajectories return: ', divTraList, 'getPassedDivisionIDs return: ', divisionIDsRlt.divisionIds)
    divTraResult = loader.getDivisionTrajectories()
    assert divTraResult.errCode == 0, \
        ("Wholemode getDivisionTrajectories(): fail; return code: ", divTraResult.errCode)

    # in wholemode, updatePosition is not supported so it return error code
    postion = RPoint3D_t()
    postion.relLon = updatePosition[0]['position'][0]
    postion.relLat = updatePosition[0]['position'][1]
    postion.relAlt = updatePosition[0]['position'][2]
    divisionIdList = updatePosition[0]['divisionId']
    layer = updatePosition[0]['layer']
    return_code = loader.updatePosition(divisionIdList, layer, postion)
    assert return_code != 0, ("updatePosition in wholemode: fail; return code: ", return_code)
    loader.clearBuffer()
    print('>>>>>>>>>>>>>>>>>>Test getNextDivisionIDs >>>>>>>>>>>>>>>>>')
    loader = PyVehicleDataLoader(conf)
    loader.initLog("vehiclDB", "loader", LOG_LEVEL.enum_ERROR)
    return_code = loader.start()
    assert return_code is True, ("start: fail; return code: ", return_code)
    for i in nextDivisionIDs:
        conf.initialPosition.lon = i['initPosition'][0]
        conf.initialPosition.lat = i['initPosition'][1]
        conf.initialPosition.alt = i['initPosition'][2]
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

def test_vehicleLoaderWholeMode():
    for i in testDB.keys():
        print('.....................Test DB: ', i, '.................')
        with open(testDB[i]['ArgsFile'], 'r') as f:
            requestDict = json.load(f)
        constructor = {}
        constructor['dbPath'] = requestDict['constructor']['dbPath']
        constructor['tolerantTime'] = requestDict['constructor']['tolerantTime']
        constructor['initPosition'] = requestDict['constructor']['initPosition']
        updatePosition = requestDict['updatePosition']
        nextDivisionIDs = requestDict['nextDivisionIDs']
        vehicleDB_loader_wholemode(constructor, updatePosition, nextDivisionIDs)