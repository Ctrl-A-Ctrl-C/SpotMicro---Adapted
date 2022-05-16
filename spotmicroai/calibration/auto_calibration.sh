#!/bin/bash

cd ~/spotmicroai || exit
export PYTHONPATH=.

python3 calibration/calibration/auto_calibration.py
