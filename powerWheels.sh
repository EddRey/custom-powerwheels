#!/bin/bash
sudo chmod 660 /dev/ttyS0
until ./gracefulShutdownRestart.py & ./MPmain.py &
do
    echo "Unexpected crash with exit code $?. Restarting... " >&2
    sleep 1
done
