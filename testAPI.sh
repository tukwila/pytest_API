#!/usr/bin/env bash

sudo docker stop master
sudo docker rm master
#docker pull harbor.roaddb.net/cm-containers/rdb-xenial-cr
echo '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>create docker master>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
sudo docker run -itd --name="master" -v /home/user/rdbTest:/rdbTest -v /home/user/:/data -w /rdbTest -h localhost.roaddb.ygomi.com harbor.roaddb.net/cm-containers/rdb-xenial-cr bash
echo '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>get rdb-vehicle deb>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
cd /home/user/rdbTest
vehicleapi=$(ls rdb-vehicle-api*)
loaderapi=$(ls rdb-vehicle-data-loader*)
pw='test1'
echo '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>install libs>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
echo $pw | sudo -S docker exec -i master dpkg -i /data/rdbTest/$vehicleapi
echo $pw | sudo -S docker exec -i master dpkg -i /data/rdbTest/$loaderapi
echo $pw | sudo -S docker exec -i master pip --proxy=http://10.69.60.221:8080 install pytest
echo $pw | sudo -S docker exec -i master pip --proxy=http://10.69.60.221:8080 install pytest-html
echo $pw | sudo -S docker exec -i master pip --proxy=http://10.69.60.221:8080 install bs4
sudo docker exec -i master bash -c "cd /data/rdbTest/ && tar -xf rdb_vehicle_api_inst-1.3.0.tgz"
# modify CS service log level from 4 error to 5 critical
echo $pw | sudo -S docker exec -u roaddb master bash -c "cd /data/rdbTest/rdb_vehicle_api_inst-1.3.0 && sudo sed -i 's#-l 4#-l 5#g' rdb-vehicleapi-svc.sh"
echo $pw | sudo -S docker exec -u roaddb master bash -c "cd /data/rdbTest/rdb_vehicle_api_inst-1.3.0 && sudo ./uninstall.sh && sudo ./install.sh"
echo '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>run testcases>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
echo $pw | sudo -S docker exec -u roaddb master bash -c "rm /data/rdbTest/pytest_API/report/*"
echo $pw | sudo -S docker exec -u roaddb master bash -c "cd /data/rdbTest/pytest_API && source config.sh && python3 run.py"
echo $pw | sudo -S cp /home/user/rdbTest/pytest_API/report/Report_All.html /var/www/html/index.html
