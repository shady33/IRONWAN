#!/bin/bash
make MODE=release
cd simulations
../flora -u Qtenv loraMultipleGateways.ini -c AESENeighbours
