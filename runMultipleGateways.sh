#!/bin/bash
make MODE=release
cd simulations
../flora -u Cmdenv loraMultipleGateways.ini -c AESENeighbours
