until ../flora_dbg -u Cmdenv -n ../src:../simulations:../../inet/src:../../inet/tutorials -l ../../inet/src/INET -f loraDQSimscape.ini -c DQ>`date +%s.%N`.rtt; do
    cd results
    mv DQ-Mslots=5,mslotduration=0.1s,nslotduration=0.15s,avg-\#0.out `date +%s.%N`.out
    #mv CheckSending-avg-\#0.out `date +%s.%N`.out
    echo "Crashed"
    sleep 1
    cd ..
done

