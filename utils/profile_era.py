#!/usr/bin/env python
# 
# Copyright 2018 IBM
# 
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

from __future__ import print_function
import subprocess

outout_filename = 'output.perf'

# Get ROS nodes' PIDs
print('Getting ROS nodes\' PIDs')
pids = []
node_list = subprocess.check_output(['rosnode', 'list'])
for node in node_list.split('\n'):
    #if ('gazebo' in node) or ('rosout' in node) or ('player' in node) or (not node):
    if not 'ros_interface' in node:
        # Ignore not relevant nodes
        continue

    node_info = subprocess.check_output('rosnode info ' + node + '| grep Pid', shell=True)
    key, pid = node_info.split(': ')
    pids.append(pid.rstrip())

# Run Linux perf
perf_command = 'sudo perf record -g -F 99 --output=' + outout_filename + ' --pid=' + ",".join(pids)
print('Running Linux perf [' + perf_command + ']')
print('Use the following command to generate a basic report: sudo perf report -g -i ' + outout_filename)
subprocess.check_output(perf_command, shell=True)
