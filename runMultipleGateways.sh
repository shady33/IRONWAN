#!/bin/bash
make MODE=debug
cd simulations
../flora_dbg -u Cmdenv -n ../src:../simulations:../../inet/examples:../../inet/src:../../inet/tutorials -l ../../inet/src/INET loraMultipleGateways.ini
