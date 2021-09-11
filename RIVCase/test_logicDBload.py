#!/usr/bin/python3
# encoding=utf-8
import sys
sys.path.append('./')
sys.path.append('..')
sys.path.append('./RIVCase')
from test_RIVWholeMode import getDivAndSeg
from config import Configs
from common import PyRIVLogicMgr
conf = Configs.config()
DBPath = conf.get('DB', 'dockerDBPath')
logicDB13 = {
    'lacksegment': {
        'dbPath': DBPath + 'fake1_mini/',
        'initReturnCode': 0,
        'getWholeHorizonReturnCode': 0,
        'position': [-83.232421875, 42.604980468, 0.0]
    },
    'lackroad': {
        'dbPath': DBPath + 'mini_lackroad/',
        'initReturnCode': 0,
        'getWholeHorizonReturnCode': 0,
        'position': [-83.232421875, 42.604980468, 0.0]
    },
    'wrongtimestamp': {
        'dbPath': DBPath + 'mini_notcorrecttimestamp/',
        'initReturnCode': 0,
        'getWholeHorizonReturnCode': 0,
        'position': [-83.232421875, 42.604980468, 0.0]
    },
    'masterwrongtimestamp': {
        'dbPath': DBPath + 'mini_masternotcorrecttimestamp/',
        'initReturnCode': 105,
        'getWholeHorizonReturnCode': 20001,
        'position': [-83.232421875, 42.604980468, 0.0]
    },
    'notExistDB': {
        'dbPath': './',
        'initReturnCode': 105,
        'getWholeHorizonReturnCode': 20001,
        'position': [0, 0, 0]
    }
}

logicDB13_right = {
    'mmg': {
        'dbPath': DBPath + 'mmg/',
        'initReturnCode': 0,
        'getWholeHorizonReturnCode': 0,
        'position': [10.138743599527558, 48.01297064703959, 555.98]
    },
    'moresegment': {
        'dbPath': DBPath + 'fake3_mini/',
        'initReturnCode': 0,
        'getWholeHorizonReturnCode': 0,
        'position': [-83.232421875, 42.604980468, 0.0]
    }
}

logicDB14 = {
    'lacksegment14': {
        'dbPath': DBPath + 'fake2_Fujiv14/',
        'initReturnCode': 0,
        'getWholeHorizonReturnCode': 0,
        'position': [138.910470193631, 35.19875645760097, 0.0]
    },
    'masterwrongtimestamp14': {
        'dbPath': DBPath + 'Fujiv14_notcorrecttimestamp/',
        'initReturnCode': 105,
        'getWholeHorizonReturnCode': 20001,
        'position': [138.910470193631, 35.19875645760097, 0.0]
    }
}
logicDB14_right={
    'fuji14': {
        'dbPath': DBPath + 'Fujiv14/',
        'initReturnCode': 0,
        'getWholeHorizonReturnCode': 0,
        'position': [138.910470193631, 35.19875645760097, 0.0]
    },
    'moresegment14': {
        'dbPath': DBPath + 'fake1_Fujiv14/',
        'initReturnCode': 0,
        'getWholeHorizonReturnCode': 0,
        'position': [138.910470193631, 35.19875645760097, 0.0]
    }
}


def logicDBload(db):
    mngr = PyRIVLogicMgr.RIVLogicMgr()
    for i in db.keys():
        print('---------------TestDB in Whole: ', i)
        conf = PyRIVLogicMgr.conf_t()
        conf.dbPath = db[i]['dbPath']
        conf.dbType_ = PyRIVLogicMgr.EDBType.E_DB_TYPE_PB
        conf.cacheMode = PyRIVLogicMgr.CACHE_MODE_E.CACHE_MODE_WHOLE_E
        mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_WARN)
        print('                init in Whole mode:')
        return_code = mngr.initWithConf(conf)
        assert return_code == db[i]['initReturnCode'], ("---------------initWithConf fail return code: ", return_code)
        print('                getWholeHorizon in Whole mode:')
        horizon_result = mngr.getWholeHorizon()
        assert horizon_result.errorCode == db[i]['getWholeHorizonReturnCode'], (
                                            "getWholeHorizon API returned error code:", horizon_result.errorCode)
        if horizon_result.errorCode == 0:
            if db == logicDB13:
                divisionList, segmentList = getDivAndSeg(db[i]['dbPath'], 0.13)
                print('                getDataBySegment in Whole mode:')
                getDataBySegment_result = mngr.getDataBySegment(segmentList)
                assert getDataBySegment_result.errorCode == 107, (
                                                "getDataBySegment API returned error code not 107:", getDataBySegment_result.errorCode)
                assert horizon_result.spHorizon != None, "getDataBySegment spHorizon is null"
            if db == logicDB14:
                divisionList, segmentList = getDivAndSeg(db[i]['dbPath'], 0.14)
                print('                getDataBySegment in Whole mode:')
                getDataBySegment_result = mngr.getDataBySegment(segmentList)
                assert getDataBySegment_result.errorCode == 107, (
                                                "getDataBySegment API returned error code not 107:", getDataBySegment_result.errorCode)
                assert getDataBySegment_result.spHorizon != None, "getDataBySegment spHorizon is null"
        mngr.destroy()
    for i in db.keys():
        print('---------------TestDB in Realtime: ', i)
        conf = PyRIVLogicMgr.conf_t()
        conf.dbPath = db[i]['dbPath']
        conf.dbType_ = PyRIVLogicMgr.EDBType.E_DB_TYPE_PB
        conf.cacheMode = PyRIVLogicMgr.CACHE_MODE_E.CACHE_MODE_REALTIME_E
        mngr.initLog(PyRIVLogicMgr.LOG_LEVEL.enum_WARN)
        return_code = mngr.initWithConf(conf)
        assert return_code == db[i]['initReturnCode'], ("---------------initWithConf fail return code: ", return_code)
        pos = PyRIVLogicMgr.WGS84_t()
        pos.lon = db[i]['position'][0]
        pos.lat = db[i]['position'][1]
        pos.alt = 0
        print('                updatePosition in Realtime mode:')
        mngr.updatePosition(pos)
        horizon_result = mngr.getWholeHorizon()
        assert horizon_result.errorCode == db[i]['getWholeHorizonReturnCode'], (
                                                "getWholeHorizon API returned error code:", horizon_result.errorCode)
        if horizon_result.errorCode == 0:
            if db == logicDB13:
                divisionList, segmentList = getDivAndSeg(db[i]['dbPath'], 0.13)
                print('                getDataBySegment in Realtime mode:')
                getDataBySegment_result = mngr.getDataBySegment(segmentList)
                assert getDataBySegment_result.errorCode == 107, (
                                                "getDataBySegment API returned error code not 107:", getDataBySegment_result.errorCode)
                assert getDataBySegment_result.spHorizon != None, "getDataBySegment spHorizon is null"
            if db == logicDB14:
                divisionList, segmentList = getDivAndSeg(db[i]['dbPath'], 0.14)
                print('                getDataBySegment in Realtime mode:')
                getDataBySegment_result = mngr.getDataBySegment(segmentList)
                assert getDataBySegment_result.errorCode == 107, (
                                                "getDataBySegment API returned error code:", getDataBySegment_result.errorCode)
                assert getDataBySegment_result.spHorizon != None, "getDataBySegment spHorizon is null"
        mngr.destroy()


def test_logicDBload():
    #test_logicDBload(logicDB13)
    print('\n' * 5)
    logicDBload(logicDB14)
