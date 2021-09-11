# -*- coding: UTF-8 -*-
import os
import configparser

def config():
    cf = configparser.ConfigParser()
    path = os.path.dirname(os.path.abspath(__file__))
    cf.read(path + '/config.ini')
    return cf
    # print('sections:' , ' ' , cf.sections())
    # print('options:' ,' ' , cf.options('DB'))
    # print('remoteDBServerIP: ', cf.get('DB', 'remoteDBServerIP'))
    # print('localDBPath:', cf.get('DB', 'localDBPath'))