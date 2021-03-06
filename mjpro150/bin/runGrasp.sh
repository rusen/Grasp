#!/bin/bash

rm -rf ./tmp
rm -rf ../model/BHAM/*_include*.xml
rm -rf ../model/BHAM/*_Test.xml

parallel --timeout 1200 --jobs $1 ./basicGrasp dummy data.bin ../model/BHAM $2 visualOff 0 ::: {1..10000}
