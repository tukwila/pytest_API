#!/usr/bin/python3
# encoding=utf-8
import sys
import os
import json
sys.path.append('./')
sys.path.append('..')
from config import Configs
from common import PyRIVLogicMgr
conf = Configs.config()
DBPath = conf.get('DB', 'dockerDBPath')
import time
import subprocess

# notes: the GPS input args json file script is in:
# /home/user/Documents/FW-357_horizon/rdb-vehicle-api/test_space/test_tools/generateGPSPoint.py

confPathList = [DBPath + 'mini/', DBPath + 'mmg/']

testDB = {
    'mmg': {
        'proto_ver': 0.14,
        'GPSFile': os.getcwd() + '/Params/GPSFile_mmg.json',
        'db': DBPath + 'fw-721/'
    }
}

testDB11 = {
    'CK': {
        'db': DBPath + '40db/',
        'GPSFile': os.getcwd() + '/Params/GPSFile_CK.json',
        'proto_ver': 0.13
    },
    'fuji': {
        'db': DBPath + 'Fujiv14/',
        'GPSFile': os.getcwd() + '/Params/GPSFile_fuji.json',
        'proto_ver': 0.14
    },
    'mmg': {
        'db': DBPath + 'mmgv14_roaddbPB/',
        'GPSFile': os.getcwd() + '/Params/GPSFile_mmg.json',
        'proto_ver': 0.14
    }
}
testDB1 = {
    'mmg': {
        'db': DBPath + 'mmg/',
        'GPSFile': os.getcwd() + '/Params/GPSFile_mmg.json',
        'proto_ver': 0.13
    }
}

def getDivAndSeg(conf_path, proto_ver):
    masterFile = ''
    files = os.listdir(conf_path)
    for i in files:
        if i.endswith('.json') and i.startswith('master'):
            masterFile = i
            break
    with open(conf_path + masterFile, 'r') as f:
        tempJson = json.load(f)
    divisionList = []
    segmentList = []
    if proto_ver == 0.13:
        roadList = tempJson['roads']
        for i in roadList:
            if isinstance(i['segment_id'], str) and i['segment_id'] not in segmentList:
                segmentList.append(i['segment_id'])
            if isinstance(i['segment_id'], list):
                for k in i['segment_id']:
                    if k not in segmentList:
                        segmentList.append(k)
            for j in i['division_ids']:
                if j not in divisionList:
                    divisionList.append(j)
        divList = []
        segList = []
        for i in divisionList:
            divList.append(int(i))
        for i in segmentList:
            segList.append(int(i))
        return divList, segList
    if proto_ver == 0.14:
        roadList = tempJson['landscapes']
        for i in roadList:
            if isinstance(i['geoarea_id'], str) and i['geoarea_id'] not in segmentList:
                segmentList.append(i['geoarea_id'])
            if isinstance(i['geoarea_id'], list):
                for k in i['geoarea_id']:
                    if k not in segmentList:
                        segmentList.append(k)
            for j in i['division_ids']:
                if j not in divisionList:
                    divisionList.append(j)
        divList = []
        segList = []
        for i in divisionList:
            divList.append(int(i))
        for i in segmentList:
            segList.append(int(i))
        return divList, segList

def findJSONFiles(conf_path):
    pbFiles = []
    files = os.listdir(conf_path)
    for i in files:
        if i.endswith('.json') and not i.startswith('master') and not i.startswith('index'):
            pbFiles.append(i)
    return pbFiles


def riv_mngr_whole_mode_initWithConf_t(conf_path):
    mngr = PyRIVLogicMgr.RIVLogicMgr()
    print('>>>>>>>>>>>>>>>>>>Test initWithConf/WHOLE/INNER_MODE>>>>>>>>>>>>>>>>>')
    conf = PyRIVLogicMgr.conf_t()
    conf.dbPath = conf_path
    conf.dbType_ = PyRIVLogicMgr.EDBType.E_DB_TYPE_PB
    conf.cacheMode = PyRIVLogicMgr.CACHE_MODE_E.CACHE_MODE_WHOLE_E
    # conf.dataCheckMode = INNER_MODE
    mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_ERROR)
    return_code = mngr.initWithConf(conf)
    assert return_code == 0, ("initWithConf/INNER_MODE: fail; return code: ", return_code)
    mngr.destroy()
    print('>>>>>>>>>>>>>>>>>>Test initWithConf/WHOLE/CUSTOM_MODE>>>>>>>>>>>>>>>>>')
    conf = PyRIVLogicMgr.conf_t()
    conf.dbPath = conf_path
    conf.dbType_ = PyRIVLogicMgr.EDBType.E_DB_TYPE_PB
    conf.cacheMode = PyRIVLogicMgr.CACHE_MODE_E.CACHE_MODE_WHOLE_E
    # conf.dataCheckMode = CUSTOM_MODE
    mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_ERROR)
    return_code = mngr.initWithConf(conf)
    assert return_code == 0, ("initWithConf/CUSTOM_MODE: fail; return code: ", return_code)
    mngr.destroy()
    print('>>>>>>>>>>>>>>>>>>Test initWithConf/WHOLE/minGetLayer>>>>>>>>>>>>>>>>>')
    conf = PyRIVLogicMgr.conf_t()
    conf.dbPath = conf_path
    conf.dbType_ = PyRIVLogicMgr.EDBType.E_DB_TYPE_PB
    conf.cacheMode = PyRIVLogicMgr.CACHE_MODE_E.CACHE_MODE_WHOLE_E
    # conf.dataCheckMode = INNER_MODE
    # conf.minGetLayer = 999
    mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_ERROR)
    return_code = mngr.initWithConf(conf)
    assert return_code == 0, ("initWithConf/minGetLayer: fail; return code: ", return_code)
    mngr.destroy()
    print('>>>>>>>>>>>>>>>>>>Test initWithConf for more times>>>>>>>>>>>>>>>>>')
    for i in confPathList:
        conf = PyRIVLogicMgr.conf_t()
        conf.dbPath = i
        conf.dbType_ = PyRIVLogicMgr.EDBType.E_DB_TYPE_PB
        conf.cacheMode = PyRIVLogicMgr.CACHE_MODE_E.CACHE_MODE_WHOLE_E
        mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_ERROR)
        return_code = mngr.initWithConf(conf)
        assert return_code == 0, ("initWithConf for more times: fail; return code: ", return_code)
        mngr.destroy()


def riv_mngr_whole_mode_initLog(conf_path):
    mngr = PyRIVLogicMgr.RIVLogicMgr()
    print('>>>>>>>>>>>>>>>>>>Test initLog>>>>>>>>>>>>>>>>>')
    return_code = mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_INFO)
    assert return_code == 0, ("initLog enum_INFO: fail; return code: ", return_code)
    return_code = mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_DEBUG)
    assert return_code == 0, ("initLog enum_DEBUG: fail; return code: ", return_code)
    conf = PyRIVLogicMgr.conf_t()
    conf.dbPath = conf_path
    conf.dbType_ = PyRIVLogicMgr.EDBType.E_DB_TYPE_PB
    conf.cacheMode = PyRIVLogicMgr.CACHE_MODE_E.CACHE_MODE_WHOLE_E
    return_code = mngr.initWithConf(conf)
    assert return_code == 0, ("initWithPath enum_DEBUG: fail; return code: ", return_code)
    mngr.destroy()
    print('>>>>>>>>>>>>>>>>>>Test initLog Pass>>>>>>>>>>>>>>>>>')


def riv_mngr_whole_mode_destroy(conf_path):
    mngr = PyRIVLogicMgr.RIVLogicMgr()
    print('>>>>>>>>>>>>>>>>>>Test destroy>>>>>>>>>>>>>>>>>')
    mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_ERROR)
    conf = PyRIVLogicMgr.conf_t()
    conf.dbPath = conf_path
    conf.dbType_ = PyRIVLogicMgr.EDBType.E_DB_TYPE_PB
    conf.cacheMode = PyRIVLogicMgr.CACHE_MODE_E.CACHE_MODE_WHOLE_E
    mngr.initWithConf(conf)
    return_code = mngr.destroy()
    assert return_code == 0, ("destroy: fail; return code: ", return_code)
    return_code = mngr.destroy()
    print("re-destroy return code: ", return_code)
    assert return_code == 0, ("re-destroy: fail; return code: ", return_code)
    print('>>>>>>>>>>>>>>>>>>Test destroy Pass>>>>>>>>>>>>>>>>>')


def riv_mngr_whole_mode_getHorizon(conf_path, GPSFile):
    gpsFile = GPSFile
    dbpath = conf_path
    mngr = PyRIVLogicMgr.RIVLogicMgr()
    print('>>>>>>>>>>>>>>>>>>Test getHorizon/valid position>>>>>>>>>>>>>>>>>')
    mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_ERROR)
    conf = PyRIVLogicMgr.conf_t()
    conf.dbPath = conf_path
    conf.dbType_ = PyRIVLogicMgr.EDBType.E_DB_TYPE_PB
    conf.cacheMode = PyRIVLogicMgr.CACHE_MODE_E.CACHE_MODE_WHOLE_E
    return_code = mngr.initWithConf(conf)
    assert return_code == 0, ("initWithPath/enum_INFO: fail; return code: ", return_code)
    with open(gpsFile, 'r') as f:
        gpsFile = json.load(f)
    for i in gpsFile:
        pos = PyRIVLogicMgr.WGS84_t()
        # pos.lon = 82.553950000
        # pos.lat = 39.518752778
        pos.lon = i[0]
        pos.lat = i[1]
        pos.alt = 0
        horizon_result = mngr.getHorizon(pos, 100)
        assert horizon_result.errorCode == 0, ("getHorizon Position: ", pos.lon, pos.lat, pos.alt,
                                               "API returned error code:", horizon_result.errorCode)
        assert horizon_result.spHorizon != None, ("getHorizon Position: ", pos.lon, pos.lat, pos.alt,
                                                  "spHorizon is: ", horizon_result.spHorizon)
        roadList = []
        for r in horizon_result.spHorizon.getLandscapes():
            roadList.append(r.getID())
        roadList = sorted(roadList)
        juncionList = []
        for j in horizon_result.spHorizon.getJunctions():
            juncionList.append(j.getID())
        juncionList = sorted(juncionList)
    mngr.destroy()
    # check test result in Rviewer

    print('>>>>>>>>>>>>>>>>>>Test getHorizon/invalid position>>>>>>>>>>>>>>>>>')
    mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_ERROR)
    conf = PyRIVLogicMgr.conf_t()
    conf.dbPath = conf_path
    conf.dbType_ = PyRIVLogicMgr.EDBType.E_DB_TYPE_PB
    conf.cacheMode = PyRIVLogicMgr.CACHE_MODE_E.CACHE_MODE_WHOLE_E
    return_code = mngr.initWithConf(conf)
    assert return_code == 0, ("getHorizon/invalid position: fail; return code: ", return_code)
    pos = PyRIVLogicMgr.WGS84_t()
    pos.lon = 0
    pos.lat = 0
    pos.alt = 0
    horizon_result = mngr.getHorizon(pos, 5)
    assert horizon_result.errorCode == 201, ("getHorizon/invalid position: return code: ", horizon_result.errorCode)
    mngr.destroy()
    print('>>>>>>>>>>>>>>>>>>Test getHorizon Pass>>>>>>>>>>>>>>>>>')

def getIsLaneOrdered(road):
    print('road id: ', road.getID(), 'is lane ordered: ', road.isLanesOrdered())


def riv_mngr_whole_mode_getWholeHorizon(conf_path, pfFiles, proto_ver):
    mngr = PyRIVLogicMgr.RIVLogicMgr()
    print('>>>>>>>>>>>>>>>>>>Test getWholeHorizon>>>>>>>>>>>>>>>>>')
    conf = PyRIVLogicMgr.conf_t()
    conf.dbPath = conf_path
    conf.dbType_ = PyRIVLogicMgr.EDBType.E_DB_TYPE_PB
    conf.cacheMode = PyRIVLogicMgr.CACHE_MODE_E.CACHE_MODE_WHOLE_E
    mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_ERROR)
    return_code = mngr.initWithConf(conf)
    assert return_code == 0, ("getWholeHorizon init: fail; return code: ", return_code)
    horizon_result = mngr.getWholeHorizon()
    roadList = []
    for r in horizon_result.spHorizon.getLandscapes():
        roadList.append(r.getID())
        #getIsLaneOrdered(r)
    roadList = sorted(roadList)
    juncionList = []
    for j in horizon_result.spHorizon.getJunctions():
        juncionList.append(j.getID())
        # for r in j.getLandscapes():
        #     getIsLaneOrdered(r)
    juncionList = sorted(juncionList)
    mngr.destroy()
    # check test result
    masterFile = ''
    files = os.listdir(conf_path)
    for i in files:
        if i.endswith('.json') and i.startswith('master'):
            masterFile = i
            break
    with open(conf_path + masterFile, 'r') as f:
        tempJson = json.load(f)
    roadResult = []
    if proto_ver == 0.13:
        roads = tempJson['roads']
        for i in roads:
            roadResult.append(i['id'])
        roadResult = sorted(roadResult)
        assert roadList == roadResult, "getWholeHorizon test result road list is different"

        pbJunctionList = []
        for i in pfFiles:
            with open(conf_path + i, 'r') as f:
                tempJson = json.load(f)
            roads = tempJson['roads']
            for j in roads:
                if 'intersection_id' in j.keys():
                    if isinstance(j['intersection_id'], str) and j['intersection_id'] not in pbJunctionList:
                        pbJunctionList.append(j['intersection_id'])
        pbJunctionList = sorted(pbJunctionList)
        assert juncionList == pbJunctionList, "getWholeHorizon test result junction list is different"
        print('>>>>>>>>>>>>>>>>>>Test getWholeHorizon Pass>>>>>>>>>>>>>>>>>')
    if proto_ver == 0.14:
        roads = tempJson['landscapes']
        for i in roads:
            roadResult.append(i['id'])
        roadResult = sorted(roadResult)
        assert roadList == roadResult, "getWholeHorizon test result road list is different"

        pbJunctionList = []
        for i in pfFiles:
            with open(conf_path + i, 'r') as f:
                tempJson = json.load(f)
            roads = tempJson['landscapes']
            for j in roads:
                if 'intersection_id' in j.keys():
                    if isinstance(j['intersection_id'], str) and j['intersection_id'] not in pbJunctionList:
                        pbJunctionList.append(j['intersection_id'])
        pbJunctionList = sorted(pbJunctionList)
        assert juncionList == pbJunctionList, "getWholeHorizon test result junction list is different"
        print('>>>>>>>>>>>>>>>>>>Test getWholeHorizon Pass>>>>>>>>>>>>>>>>>')


def riv_mngr_whole_mode_getDataBySegment(conf_path, segmentList, proto_ver):
    mngr = PyRIVLogicMgr.RIVLogicMgr()
    print('>>>>>>>>>>>>>>>>>>Test getDataBySegment>>>>>>>>>>>>>>>>>')
    mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_ERROR)
    conf = PyRIVLogicMgr.conf_t()
    conf.dbPath = conf_path
    conf.dbType_ = PyRIVLogicMgr.EDBType.E_DB_TYPE_PB
    conf.cacheMode = PyRIVLogicMgr.CACHE_MODE_E.CACHE_MODE_WHOLE_E
    return_code = mngr.initWithConf(conf)
    assert return_code == 0, ("getDataBySegment init: fail; return code: ", return_code)
    horizon_result = mngr.getDataBySegment(segmentList)
    # check_horizon_result(horizon_result)
    assert horizon_result.errorCode == 0, ("getDataBySegment API returned error code:", horizon_result.errorCode)
    assert horizon_result.spHorizon != None, "getDataBySegment spHorizon is null"
    # Traverse horizon
    roadList = []
    for r in horizon_result.spHorizon.getLandscapes():
        roadList.append(r.getID())
    roadList = sorted(roadList)
    mngr.destroy()
    # check test result in master db
    masterFile = ''
    files = os.listdir(conf_path)
    for i in files:
        if i.endswith('.json') and i.startswith('master'):
            masterFile = i
            break
    with open(conf_path + masterFile, 'r') as f:
        tempJson = json.load(f)
    roadResult = []
    if proto_ver == 0.13:
        roads = tempJson['roads']
        for i in roads:
            if isinstance(i['passed_segment_ids'], str) and i['passed_segment_ids'] in segmentList:
                roadResult.append(i['id'])
            if isinstance(i['passed_segment_ids'], list):
                for j in i['passed_segment_ids']:
                    if int(j) in segmentList and i['id'] not in roadResult:
                        roadResult.append(i['id'])
        roadResult = sorted(roadResult)
        assert roadList == roadResult, "getDataBySegment test result is different"
        print('>>>>>>>>>>>>>>>>>>Test getDataBySegment Pass>>>>>>>>>>>>>>>>>')
    if proto_ver == 0.14:
        roads = tempJson['landscapes']
        for i in roads:
            if isinstance(i['passed_geoarea_ids'], str) and i['passed_geoarea_ids'] in segmentList:
                roadResult.append(i['id'])
            if isinstance(i['passed_geoarea_ids'], list):
                for j in i['passed_geoarea_ids']:
                    if int(j) in segmentList and i['id'] not in roadResult:
                        roadResult.append(i['id'])
        roadResult = sorted(roadResult)
        assert roadList == roadResult, "getDataBySegment test result is different"
        print('>>>>>>>>>>>>>>>>>>Test getDataBySegment Pass>>>>>>>>>>>>>>>>>')

def riv_mngr_whole_mode_getDataByDivision(conf_path, divisionList, proto_ver):
    mngr = PyRIVLogicMgr.RIVLogicMgr()
    print('>>>>>>>>>>>>>>>>>>Test getDataByDivision>>>>>>>>>>>>>>>>>')
    mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_ERROR)
    conf = PyRIVLogicMgr.conf_t()
    conf.dbPath = conf_path
    conf.dbType_ = PyRIVLogicMgr.EDBType.E_DB_TYPE_PB
    conf.cacheMode = PyRIVLogicMgr.CACHE_MODE_E.CACHE_MODE_WHOLE_E
    return_code = mngr.initWithConf(conf)
    assert return_code == 0, ("getDataByDivision init: fail; return code: ", return_code)
    horizon_result = mngr.getDataByDivision(divisionList)
    # check_horizon_result(horizon_result)
    assert horizon_result.errorCode == 0, ("getDataByDivision API returned error code:", horizon_result.errorCode)
    assert horizon_result.spHorizon != None, "getDataByDivision spHorizon is null"
    # Traverse horizon
    roadList = []
    for r in horizon_result.spHorizon.getLandscapes():
        roadList.append(r.getID())
    roadList = sorted(roadList)
    mngr.destroy()
    # check test result
    masterFile = ''
    files = os.listdir(conf_path)
    for i in files:
        if i.endswith('.json') and i.startswith('master'):
            masterFile = i
            break
    with open(conf_path + masterFile, 'r') as f:
        tempJson = json.load(f)
    roadResult = []
    if proto_ver == 0.13:
        roads = tempJson['roads']
        for i in roads:
            if isinstance(i['division_ids'], str) and i['division_ids'] in divisionList:
                roadResult.append(i['id'])
            if isinstance(i['division_ids'], list):
                for j in i['division_ids']:
                    if int(j) in divisionList and i['id'] not in roadResult:
                        roadResult.append(i['id'])
        roadResult = sorted(roadResult)
        assert roadList == roadResult, "getDataByDivision test result is different"
        print('>>>>>>>>>>>>>>>>>>Test getDataByDivision Pass>>>>>>>>>>>>>>>>>')
    if proto_ver == 0.14:
        roads = tempJson['landscapes']
        for i in roads:
            if isinstance(i['division_ids'], str) and i['division_ids'] in divisionList:
                roadResult.append(i['id'])
            if isinstance(i['division_ids'], list):
                for j in i['division_ids']:
                    if int(j) in divisionList and i['id'] not in roadResult:
                        roadResult.append(i['id'])
        roadResult = sorted(roadResult)
        assert roadList == roadResult, "getDataByDivision test result is different"
        print('>>>>>>>>>>>>>>>>>>Test getDataByDivision Pass>>>>>>>>>>>>>>>>>')


def riv_mngr_whole_mode_getAllDivisionIDs(conf_path, proto_ver):
    mngr = PyRIVLogicMgr.RIVLogicMgr()
    print('>>>>>>>>>>>>>>>>>>Test getAllDivisionIDs>>>>>>>>>>>>>>>>>')
    mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_ERROR)
    conf = PyRIVLogicMgr.conf_t()
    conf.dbPath = conf_path
    conf.dbType_ = PyRIVLogicMgr.EDBType.E_DB_TYPE_PB
    conf.cacheMode = PyRIVLogicMgr.CACHE_MODE_E.CACHE_MODE_WHOLE_E
    return_code = mngr.initWithConf(conf)
    assert return_code == 0, ("getAllDivisionIDs init: fail; return code: ", return_code)
    all_div_ids = mngr.getAllDivisionIDs()
    print('all_div_ids: ', sorted(all_div_ids))
    assert len(all_div_ids) != 0, ("getAllDivisionIDs map length is 0")
    print("==== getAllDivisionIDs result ====")
    sdkDivisionDict = {}
    for seg_id, div_id_set in all_div_ids.items():
        tempDict = {}
        div_ids = []
        for div_id in div_id_set:
            div_ids.append(div_id)
        tempDict[seg_id] = sorted(div_ids)
        sdkDivisionDict[seg_id] = tempDict[seg_id]
    mngr.destroy()
    sortedSDKDivDict = sorted(sdkDivisionDict.items(), key=lambda x: x[0])
    # print('sdkDivisionDict: ', sortedSDKDivDict)
    # check test result
    masterFile = ''
    files = os.listdir(conf_path)
    for i in files:
        if i.endswith('.json') and i.startswith('master'):
            masterFile = i
            break
    with open(conf_path + masterFile, 'r') as f:
        tempJson = json.load(f)
    roadResultTemp = {}
    if proto_ver == 0.13:
        roads = tempJson['roads']
        for i in roads:
            for j in i['passed_segment_ids']:
                if int(j) not in roadResultTemp.keys():
                    roadResultTemp[int(j)] = []
                    for k in i['division_ids']:
                        if int(k) not in roadResultTemp[int(j)]:
                            roadResultTemp[int(j)].append(int(k))
                else:
                    for k in i['division_ids']:
                        if int(k) not in roadResultTemp[int(j)]:
                            roadResultTemp[int(j)].append(int(k))
        for i in roadResultTemp.keys():
            roadResultTemp[i] = sorted(roadResultTemp[i])
        sortedRoadResult = sorted(roadResultTemp.items(), key=lambda x: x[0])
        assert sortedSDKDivDict == sortedRoadResult, "getAllDivisionIDs SDK is not same with PB"
        print('>>>>>>>>>>>>>>>>>>Test getAllDivisionIDs Pass>>>>>>>>>>>>>>>>>')
    if proto_ver == 0.14:
        roads = tempJson['landscapes']
        for i in roads:
            for j in i['passed_geoarea_ids']:
                if int(j) not in roadResultTemp.keys():
                    roadResultTemp[int(j)] = []
                    for k in i['division_ids']:
                        if int(k) not in roadResultTemp[int(j)]:
                            roadResultTemp[int(j)].append(int(k))
                else:
                    for k in i['division_ids']:
                        if int(k) not in roadResultTemp[int(j)]:
                            roadResultTemp[int(j)].append(int(k))
        for i in roadResultTemp.keys():
            roadResultTemp[i] = sorted(roadResultTemp[i])
        sortedRoadResult = sorted(roadResultTemp.items(), key=lambda x: x[0])
        assert sortedSDKDivDict == sortedRoadResult, "getAllDivisionIDs SDK is not same with PB"
        print('>>>>>>>>>>>>>>>>>>Test getAllDivisionIDs Pass>>>>>>>>>>>>>>>>>')


def riv_mngr_whole_mode_getPassRoadIDs(conf_path, segmentList, proto_ver):
    mngr = PyRIVLogicMgr.RIVLogicMgr()
    print('>>>>>>>>>>>>>>>>>>Test getPassRoadIDs>>>>>>>>>>>>>>>>>')
    mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_ERROR)
    conf = PyRIVLogicMgr.conf_t()
    conf.dbPath = conf_path
    conf.dbType_ = PyRIVLogicMgr.EDBType.E_DB_TYPE_PB
    conf.cacheMode = PyRIVLogicMgr.CACHE_MODE_E.CACHE_MODE_WHOLE_E
    return_code = mngr.initWithConf(conf)
    assert return_code == 0, ("getPassRoadIDs init: fail; return code: ", return_code)
    road_ids = mngr.getPassRoadIDs(segmentList)
    print('getPassRoadIDs: ', sorted(road_ids))
    assert len(road_ids) != 0, ("getPassRoadIDs road id list length is 0")
    # print("==== getPassRoadIDs result ====")
    mngr.destroy()
    # check test result
    masterFile = ''
    files = os.listdir(conf_path)
    for i in files:
        if i.endswith('.json') and i.startswith('master'):
            masterFile = i
            break
    with open(conf_path + masterFile, 'r') as f:
        tempJson = json.load(f)
    if proto_ver == 0.13:
        roads = tempJson['roads']
        roadResult = []
        for i in roads:
            if isinstance(i['passed_segment_ids'], list):
                for j in i['passed_segment_ids']:
                    if int(j) in segmentList:
                        roadResult.append(i['id'])
        roadResult = set(roadResult)
        assert road_ids.difference(roadResult) == set([]) and roadResult.difference(road_ids) == set([]), \
            ("getPassRoadIDs SDK is not same with PB")
        print('>>>>>>>>>>>>>>>>>>Test getPassRoadIDs Pass>>>>>>>>>>>>>>>>>')
    if proto_ver == 0.14:
        roads = tempJson['landscapes']
        roadResult = []
        for i in roads:
            if isinstance(i['passed_geoarea_ids'], list):
                for j in i['passed_geoarea_ids']:
                    if int(j) in segmentList:
                        roadResult.append(i['id'])
        roadResult = set(roadResult)
        assert road_ids.difference(roadResult) == set([]) and roadResult.difference(road_ids) == set([]), \
            ("getPassRoadIDs SDK is not same with PB")
        print('>>>>>>>>>>>>>>>>>>Test getPassRoadIDs Pass>>>>>>>>>>>>>>>>>')


def getDataVersion():
    # now, pywrapper don't implemented
    mngr = PyRIVLogicMgr.RIVLogicMgr()
    VersionResult = mngr.VersionResult_t
    print(VersionResult)

def riv_mngr_whole_mode_test():
    print('>>>>>>>>>>>>>>>>>>Start riv_mngr_whole_mode_test >>>>>>>>>>>>>>>>>')
    for i in testDB.keys():
        print('>>>>>>>>>>>>>>>>>>db under test: ', i, '>>>>>>>>>>>>>>>>>')
        #riv_mngr_whole_mode_initWithConf_t(testDB[i]['db'])
        #riv_mngr_whole_mode_initLog(testDB[i]['db'])
        #riv_mngr_whole_mode_destroy(testDB[i]['db'])
        riv_mngr_whole_mode_getHorizon(testDB[i]['db'], testDB[i]['GPSFile'])
        #jsonFiles = findJSONFiles(testDB[i]['db'])
        #riv_mngr_whole_mode_getWholeHorizon(testDB[i]['db'], jsonFiles, testDB[i]['proto_ver'])
        #divisionList, segmentList = getDivAndSeg(testDB[i]['db'], testDB[i]['proto_ver'])
        #riv_mngr_whole_mode_getDataBySegment(testDB[i]['db'], segmentList, testDB[i]['proto_ver'])
        #riv_mngr_whole_mode_getDataByDivision(testDB[i]['db'], set(divisionList), testDB[i]['proto_ver'])
        #riv_mngr_whole_mode_getAllDivisionIDs(testDB[i]['db'], testDB[i]['proto_ver'])
        #riv_mngr_whole_mode_getPassRoadIDs(testDB[i]['db'], segmentList, testDB[i]['proto_ver'])
    print('>>>>>>>>>>>>>>>>>>End riv_mngr_whole_mode_test >>>>>>>>>>>>>>>>>')


if __name__ == '__main__':
    riv_mngr_whole_mode_test()

def test_RIVWholeMode():
    riv_mngr_whole_mode_test()
