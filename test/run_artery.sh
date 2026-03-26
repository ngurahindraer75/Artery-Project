#!/bin/bash
OPP_RUNALL=/opt/omnetpp/bin/opp_runall
OPP_RUN=/opt/omnetpp/bin/opp_run_dbg
NED_FOLDERS="/home/vagrant/artery/src/artery:/home/vagrant/artery/src/traci:/home/vagrant/artery/extern/veins/examples/veins:/home/vagrant/artery/extern/veins/src/veins:/home/vagrant/artery/extern/inet/src:/home/vagrant/artery/extern/inet/examples:/home/vagrant/artery/extern/inet/tutorials:/home/vagrant/artery/extern/inet/showcases"
LIBRARIES="-l/home/vagrant/artery/test/src/artery/libartery_core.so -l/home/vagrant/artery/test/src/traci/libtraci.so -l/home/vagrant/artery/test/extern/libveins.so -l/home/vagrant/artery/test/extern/libINET.so -l/home/vagrant/artery/test/src/artery/storyboard/libartery_storyboard.so -l/home/vagrant/artery/test/src/artery/envmod/libartery_envmod.so"

RUNALL=false
for arg do
    shift
    [[ "$arg" == -j* ]] && RUNALL=true && J=$arg && continue
    [[ "$arg" == -b* ]] && RUNALL=true && B=$arg && continue
    # run opp_runall with default values for -j* and -b* options by just specifying '--all'
    [[ "$arg" == "--all" ]] && RUNALL=true && continue
    set -- "$@" "$arg"
done

if [ "$RUNALL" = true ] ; then
    $OPP_RUNALL $J $B $OPP_RUN -n $NED_FOLDERS $LIBRARIES $@
else
    $OPP_RUN -n $NED_FOLDERS $LIBRARIES $@
fi
