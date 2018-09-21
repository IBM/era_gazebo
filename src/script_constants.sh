#!/bin/bash

SLEEP_TIME=25 #s

WORKLOAD_SMALL=10 #s
WORKLOAD_LARGE=60 #s

PLAYBACK_SCALING=2.0

if [ -z "$1" ]; then
  WORKLOAD_SIZE=small
  WORKLOAD_DURATION=$WORKLOAD_SMALL
elif [[ "$1" == "small" ]]; then
  WORKLOAD_SIZE=$1
  WORKLOAD_DURATION=$WORKLOAD_SMALL
elif [[ "$1" == "large" ]]; then
  WORKLOAD_SIZE=$1
  WORKLOAD_DURATION=$WORKLOAD_LARGE
else
  echo "'$0 $1' is not a valid option"
  exit 1
fi

if [ -z "$2" ]; then
  WORKLOAD_TYPE=rotator
elif [[ "$2" == "rotator" ]]; then
  WORKLOAD_TYPE=$2
elif [[ "$2" == "wanderer" ]]; then
  WORKLOAD_TYPE=$2
else
  echo "'$0 $2' is not a valid option"
  exit 1
fi


if [ -z "$3" ]; then
  AUTO=no
else
  AUTO=yes
fi 

#PROJECT_PATH=$4
WORKLOAD_PATH=$5
PROFILES_PATH=$6

source $PROJECT_PATH/devel/setup.bash
