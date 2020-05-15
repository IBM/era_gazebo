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

if [[ "$AUTO" == "yes" ]]; then
  echo "[$0] Sleeping for $SLEEP_TIME seconds..."
  sleep $SLEEP_TIME
fi

bags="$(find "$PROJECT_PATH/bags/era_auto" \
     -name "bot_$WORKLOAD_TYPE-$WORKLOAD_SIZE*.bag*" \
     | sort | tr '\n' ' ')"

if [ -z "$bags" ]; then
    echo "ERROR: No bags found!"
    exit 1
fi

echo "[$0] Profiling bags: $bags..."

for bag in $bags
do
  if [ ! -e ]; then
    echo "[$0] ERROR: Could not find bag '$bag'"
    rosnode kill -a
    exit 1
  fi
done

callgrind_control -i on

for bag in $bags
do
  roslaunch era_gazebo workload_playback.launch bag_name:="$bag"
done

callgrind_control -d
callgrind_control -i off

echo "[$0] Done"
rosnode kill -a
