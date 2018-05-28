#!/bin/bash

ClusterId=$(cat $_CONDOR_JOB_AD | grep ClusterId | cut -d '=' -f 2 | tail -1 | tr -d [:space:])
ProcId=$(cat $_CONDOR_JOB_AD | grep ProcId | tail -1 | cut -d '=' -f 2 | tr -d [:space:])
JobStatus=$(cat $_CONDOR_JOB_AD | grep JobStatus | head -1 | cut -d '=' -f 2 | tr -d [:space:])
RemoteHost=$(cat $_CONDOR_JOB_AD | grep RemoteHost | head -1 | cut -d '=' -f 2 | cut -d '@' -f 2 | cut -d '.' -f 1)

if [[ -e /tmp/singlebel ]]; then
    while [ $JobStatus = 2 ] ; do
        ssh opal /usr/bin/condor_hold ${ClusterId}.${ProcId}
        JobStatus=$(cat $_CONDOR_JOB_AD | grep JobStatus | head -1 | cut -d '=' -f 2 | tr -d [:space:])
        sleep 10
    done
else
    touch /tmp/singlebel
fi
