#!/usr/bin/python3
# -*- coding: UTF-8 -*-
import pytest
import os
from common.parseHtml import *

if __name__ == '__main__':
    cmd1 = 'pytest --html=' + os.path.dirname(os.path.abspath(__file__)) + '/report/Report_VehicleLoader.html --self-contained-html VehicleLoaderCase'
    cmd2 = 'pytest --html=' + os.path.dirname(os.path.abspath(__file__)) + '/report/Report_RIVmgr.html --self-contained-html RIVCase'
    cmd3 = 'pytest --html=' + os.path.dirname(os.path.abspath(__file__)) + '/report/Report_genericAPI.html --self-contained-html genericAPICase'
    os.system(cmd1)
    os.system(cmd2)
    os.system(cmd3)

    mergeHTML()
