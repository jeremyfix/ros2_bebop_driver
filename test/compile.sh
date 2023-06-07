#!/usr/bin/bash

g++ -o test_bebop test_bebop.cpp bebop.cpp -I $ARSDK_PATH/include -I. -L $ARSDK_PATH/lib -larsal -lardiscovery -larcontroller -larnetworkal -larcommands -ljson -lmux -lpomp -lulog -larstream -larstream2  -lrtsp -lsdp -larnetwork -larmedia -lfutils -larsal -I../include/
