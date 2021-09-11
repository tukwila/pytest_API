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

def test_HorizonInSDK():
    # step 1, modify configuration.json file Database
    with open(dockerTestSpacePath + 'test_tools/configuration.json', 'r') as jsonfile:
        tempjson = json.load(jsonfile)
    tempjson['Database']['db_path'] = '../test_data/db/40db/'
    jsonStr = json.dumps(tempjson, indent=4)
    with open(dockerTestSpacePath + 'test_tools/configuration.json', 'w') as f:
        f.write(jsonStr)
    # step 2, execute all params files
    status, ret = subprocess.getstatusoutput('cd ' + dockerTestSpacePath + 'test_tools/ && ./testcase_run.sh -d ../test_case/HorizonTC')
    assert 'testcase' in ret, (ret)
    timestamp = ret.split('\n')[-1].split()[-1].split('_')[-1].split('.')[0]
    assert re.search(r"(\d{12})", timestamp) is not None, ('timestamp test fail: ', timestamp)
    cmd = 'grep \'different\' ' + dockerTestSpacePath + 'test_tools/testcase_' + timestamp + '.log'
    status, ret = subprocess.getstatusoutput(cmd)
    assert ret == '', ("Horizon in SDK mode is fail, log file is: testcase_" + timestamp + '.log')


def test_HorizonInCS():
    # step 1, modify configuration.json file Database
    with open(dockerTestSpacePath + 'test_tools/configuration.json', 'r') as jsonfile:
        tempjson = json.load(jsonfile)
    tempjson['Database']['db_path'] = '../test_data/db/40db/'
    jsonStr = json.dumps(tempjson, indent=4)
    with open(dockerTestSpacePath + 'test_tools/configuration.json', 'w') as f:
        f.write(jsonStr)
    # step 2, cp db files to /opt/ygomi/roadDB/etc/roadDB_PB/
    os.system('echo \'' + PW + '\' | sudo -S rm /opt/ygomi/roadDB/etc/roadDB_PB/*')
    os.system('echo \'' + PW + '\' | sudo -S cp ' + dockerTestSpacePath + '/test_data/db/40db/* /opt/ygomi/roadDB/etc/roadDB_PB/')
    os.system('echo \'' + PW + '\' | sudo -S service rdb-vehicleapi-svc restart ')
    status, ret = subprocess.getstatusoutput('ps -ef|grep rdb_vehicle_api_svc |grep -v grep')
    assert 'rdb_vehicle_api_svc' in ret, ('no rdb_vehicle_api_svc process')
    # step 3, execute all params files
    status, ret = subprocess.getstatusoutput('cd ' + dockerTestSpacePath + 'test_tools/ && ./testcase_run.sh -d ../test_case/HorizonCSTC -m 1')
    assert 'testcase' in ret, (ret)
    timestamp = ret.split('\n')[-1].split()[-1].split('_')[-1].split('.')[0]
    assert re.search(r"(\d{12})", timestamp) is not None, ('timestamp test fail: ', timestamp)
    cmd = 'grep \'different\' ' + dockerTestSpacePath + 'test_tools/testcase_' + timestamp + '.log'
    status, ret = subprocess.getstatusoutput(cmd)
    assert ret == '', ("Horizon in CS mode is fail, log file is: testcase_" + timestamp + '.log')

if __name__=='__main__':
    #test_HorizonInSDK()
    test_HorizonInCS()