#!/bin/bash

# install script + package
sudo python3 setup.py install

# install supervisor for manage script start/stop
sudo apt-get -y install supervisor
sudo cp etc/supervisor/conf.d/pymbserver.conf /etc/supervisor/conf.d/
sudo supervisorctl update
