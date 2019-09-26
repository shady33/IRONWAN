#!/bin/bash
make MODE=release
cd simulations
../flora -u Qtenv -n ../src:../simulations:../../inet/examples:../../inet/src:../../inet/tutorials -l ../../inet/src/INET loraMultipleGateways.ini -c AESENeighbours
