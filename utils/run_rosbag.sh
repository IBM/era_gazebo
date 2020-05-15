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

PROJECT_PATH=$4

source $PROJECT_PATH/era_gazebo/src/script_constants.sh

mkdir -p $PROFILES_PATH

if [[ "$AUTO" == "yes" ]]; then
  echo "[$0] Sleeping for $SLEEP_TIME seconds..."
  sleep $SLEEP_TIME
fi

echo "[$0] Launching bot_$WORKLOAD_TYPE..."
rosrun era_gazebo bot_$WORKLOAD_TYPE r0 &
rosrun era_gazebo bot_$WORKLOAD_TYPE r1 &

echo "[$0] Recording..."
rosbag record -a --duration=$WORKLOAD_DURATION \
    -O $PROJECT_PATH/bags/era_auto/bot_$WORKLOAD_TYPE-$WORKLOAD_SIZE \
    __name:=workload_recorder #\
    # tf \
    # /r0/mobile_base/commands/velocity \
    # /r1/mobile_base/commands/velocity &

echo "[$0] Sleeping for $WORKLOAD_DURATION..."
sleep $WORKLOAD_DURATION

rosnode kill workload_recorder r0/bot_$WORKLOAD_TYPE r1/bot_$WORKLOAD_TYPE

echo "[$0] Done"
rosnode kill -a
