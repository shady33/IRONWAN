#!/bin/bash
make MODE=debug
cd simulations
opp_runall -j2 -b1 ../flora_dbg -u Cmdenv -n ../src:../simulations:../../inet/examples:../../inet/src:../../inet/tutorials -l ../../inet/src/INET loraMultipleGateways.ini -c LoRaWAN
