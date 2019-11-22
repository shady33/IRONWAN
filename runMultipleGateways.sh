#!/bin/bash
make MODE=release
cd simulations
opp_runall -j32 -b1 ../lora-multi-gateway -u Cmdenv loraMultipleGateways_Potoo.ini -c AESENeighbours &
