#!/bin/bash

cd ~/spotmicroai || exit
export PYTHONPATH=.

python3 calibration/calibration/stand_position.py
