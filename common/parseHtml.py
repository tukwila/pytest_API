# -*- coding: UTF-8 -*-
from bs4 import BeautifulSoup

def mergeHTML():
    VehicleLoaderHTML = './report/Report_VehicleLoader.html'
    RIVmgrHTML = './report/Report_RIVmgr.html'
    GenericHTML = './report/Report_genericAPI.html'

    with open(VehicleLoaderHTML, 'r') as f:
        VehicleLoader_htmlstr = f.read()
    soup = BeautifulSoup(VehicleLoader_htmlstr, 'lxml')
    tbodys = soup.find_all('tbody')

    with open(GenericHTML, 'r') as f:
        Generic_htmlstr = f.read()
    soup2 = BeautifulSoup(Generic_htmlstr, 'lxml')
    tbodys2 = soup2.find_all('tbody')

    with open(RIVmgrHTML, 'r') as f1:
        RIVmgr_htmlstr = f1.read()

    soup1 = BeautifulSoup(RIVmgr_htmlstr, 'lxml')
    RIVmgr_tbody = soup1.find_all('tbody')
    for i in tbodys:
        RIVmgr_tbody[-1].insert_after(i)
    for j in tbodys2:
        RIVmgr_tbody[-1].insert_after(j)

    for i in soup1.find_all('input'):
        i.decompose()

    for i in soup1.find_all('span'):
        i.decompose()

    soup1.find_all('p')[-1].decompose()
    soup1.find_all('p')[-1].decompose()
    soup1.find_all('h2')[-2].decompose()

    soup1.h1.string = 'API Testing Report'

    with open('./report/Report_All.html', 'w') as f2:
        f2.write(soup1.prettify())

if __name__ == '__main__':
    mergeHTML()
