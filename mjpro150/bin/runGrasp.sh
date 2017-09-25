#!/bin/bash

rm -rf ./tmp
rm -rf ../model/BHAM/*_include*.xml
rm -rf ../model/BHAM/*_Test.xml

parallel --timeout 1200 --jobs $1 ./basicGrasp ../model/BHAM $2 visualOff ::: {1..10000}
