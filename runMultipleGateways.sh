#!/bin/bash
make MODE=release
cd simulations
opp_runall -j5 -b1 ../flora -u Cmdenv loraMultipleGateways.ini -c AESENeighbours
