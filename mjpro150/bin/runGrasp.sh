#!/bin/bash

rm -rf ./tmp
rm -rf ../model/BHAM/*_include*.xml
rm -rf ../model/BHAM/*_Test.xml

N=4
DROPBOXFOLDER=/home/rusi/Dropbox

parallel --timeout 900 --jobs $N ./basicGrasp ../model/BHAM $DROPBOXFOLDER visualOff $RANDOM ::: {1..10000}
