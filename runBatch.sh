#!/bin/bash
cd simulations
opp_runall -j32 -b1 ../flora_dbg -u Cmdenv -c CheckSending -n ../src:../simulations:../../inet/examples:../../inet/src:../../inet/tutorials -l ../../inet/src/INET loraDQ.ini &
