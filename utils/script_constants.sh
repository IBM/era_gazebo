#!/bin/bash
#
# Copyright 2018 IBM
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

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
