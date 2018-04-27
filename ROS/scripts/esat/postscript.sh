#!/bin/bash
RemoteHost=$(cat $_CONDOR_JOB_AD | grep RemoteHost | head -1 | cut -d '=' -f 2 | cut -d '@' -f 2 | cut -d '.' -f 1)

rm /tmp/singlebel

# singularity fails to kill all consequently.
killall -u r0453462
