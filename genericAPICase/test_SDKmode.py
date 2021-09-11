#!/usr/bin/python3
# encoding=utf-8
import sys
import os
import subprocess
import json
import re
sys.path.append('./')
sys.path.append('..')
from config import Configs
conf = Configs.config()
dockerTestSpacePath = conf.get('test_space', 'dockerTestSpacePath')
PW = conf.get('test_space', 'sudoPW')
DBPath = conf.get('DB', 'dockerDBPath')

logicDB = {
    'CK': {
        'dbPath': dockerTestSpacePath + 'test_data/db/40db/',
        'paraFiles': DBPath + 'SDKCK/',
        'csparaFiles': DBPath + 'CSCK/'
    },
    'HondaPG1': {
        'dbPath': DBPath + 'HondaPG1/',
        'paraFiles': DBPath + 'SDKhondaPG1/',
        'csparaFiles': DBPath + 'CShondaPG1/'
    },
    'mmgv14_roaddbPB': {
        'dbPath': DBPath + 'mmgv14_roaddbPB/',
        'paraFiles': DBPath + 'SDKmmgv14/',
        'csparaFiles': DBPath + 'CSmmgv14/'
    },
    'largescale_fuji': {
        'dbPath': DBPath + 'largescale_fuji/',
        'paraFiles': DBPath + 'SDKlargescalefuji/',
        'csparaFiles': DBPath + 'CSlargescalefuji/',
    },
    'largescale_fuji': {
        'dbPath': DBPath + 'largescale_TemaAnnArbor/',
        'paraFiles': DBPath + 'SDKlargescaleTemaAnnArbor/',
        'csparaFiles': DBPath + 'CSlargescaleTemaAnnArbor/',
    },
    'largescale_UK': {
        'dbPath': DBPath + 'largescale_UK/',
        'paraFiles': DBPath + 'SDKlargescaleUK/',
        'csparaFiles': DBPath + 'CSlargescaleUK/',
    },
    'largescale_USi696': {
        'dbPath': DBPath + 'largescale_USi696/',
        'paraFiles': DBPath + 'SDKlargescaleUSi696/',
        'csparaFiles': DBPath + 'CSlargescaleUSi696/',
    },
    'largescale_Wolfsburg': {
        'dbPath': DBPath + 'largescale_Wolfsburg/',
        'paraFiles': DBPath + 'SDKlargescaleWolfsburg/',
        'csparaFiles': DBPath + 'CSlargescaleWolfsburg/',
    }
}


def test_SDKmode():
    for i in logicDB.keys():
        print('---------------TestDB in SDK mode: ' + i + '; DB: ' + logicDB[i]['dbPath'])
        # step 1, modify configuration.json file
        with open(dockerTestSpacePath + 'test_tools/configuration.json', 'r') as jsonfile:
            tempjson = json.load(jsonfile)
        tempjson['Database']['db_path'] = logicDB[i]['dbPath']
        jsonStr = json.dumps(tempjson, indent=4)
        with open(dockerTestSpacePath + 'test_tools/configuration.json', 'w') as f:
            f.write(jsonStr)
        # step 2, execute all params files
        status, ret = subprocess.getstatusoutput('cd ' + dockerTestSpacePath + 'test_tools/ && ./testcase_run.sh -d ' + logicDB[i]['paraFiles'])
        assert 'testcase' in ret, (ret)
        timestamp = ret.split('\n')[-1].split()[-1].split('_')[-1].split('.')[0]
        assert re.search(r"(\d{12})", timestamp) is not None, ('timestamp test fail: ', timestamp)
        cmd = 'grep \'different\' ' + dockerTestSpacePath + 'test_tools/testcase_' + timestamp + '.log'
        status, ret = subprocess.getstatusoutput(cmd)
        assert ret == '', (i + " in SDK mode is fail, log file is: testcase_" + timestamp + '.log')

def test_CSmode():
    for i in logicDB.keys():
        print('---------------TestDB in CS mode: ' + i + '; DB: ' + logicDB[i]['dbPath'])
        # step 1, modify configuration.json file
        with open(dockerTestSpacePath + 'test_tools/configuration.json', 'r') as jsonfile:
            tempjson = json.load(jsonfile)
        tempjson['Database']['db_path'] = logicDB[i]['dbPath']
        jsonStr = json.dumps(tempjson, indent=4)
        with open(dockerTestSpacePath + 'test_tools/configuration.json', 'w') as f:
            f.write(jsonStr)
        # step 2, cp db files to /opt/ygomi/roadDB/etc/roadDB_PB/
        os.system('echo \'' + PW + '\' | sudo -S rm /opt/ygomi/roadDB/etc/roadDB_PB/*')
        os.system('echo \'' + PW + '\' | sudo -S cp ' + logicDB[i]['dbPath'] + '* /opt/ygomi/roadDB/etc/roadDB_PB/')
        os.system('echo \'' + PW + '\' | sudo -S service rdb-vehicleapi-svc restart ')
        status, ret = subprocess.getstatusoutput('ps -ef|grep rdb_vehicle_api_svc |grep -v grep')
        assert 'rdb_vehicle_api_svc' in ret, ('no rdb_vehicle_api_svc process')
        # step 3, execute all params files
        status, ret = subprocess.getstatusoutput('cd ' + dockerTestSpacePath + 'test_tools/ && ./testcase_run.sh -d ' + logicDB[i]['csparaFiles'] + ' -m 1')
        timestamp = ret.split('\n')[-1].split()[-1].split('_')[-1].split('.')[0]
        # check if timestamp is timestamp !!!!
        assert re.search(r"(\d{12})", timestamp) is not None, ('timestamp test fail: ', ret)
        cmd = 'grep \'different\' ' + dockerTestSpacePath + 'test_tools/testcase_' + timestamp + '.log'
        status, ret = subprocess.getstatusoutput(cmd)
        assert ret == '', (i + " in CS mode is fail, log file is: testcase_" + timestamp + '.log')

# if __name__=='__main__':
#     test_SDKmode()
#     test_CSmode()
