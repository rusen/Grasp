#!/bin/bash

python reRunSimulation.py test 8 data_100epoch_nosoftmax.bin /home/rusi007/Dropbox
python reRunSimulation.py validate 8 data_100epoch_nosoftmax.bin /home/rusi007/Dropbox
