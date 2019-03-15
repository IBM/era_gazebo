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
import os
import sys
import getopt
import subprocess
import time
import re

def usage():
    print("Usage: %s [-t timeout_secs] [-c command] [-o output_filename] [-p profiler]" % sys.argv[0])
    print("       (profiler: perf_stat, perf_record, operf)")
    
gr_perf_to_csv = '/home/augustojv/catkin_ws/src/dsrc/gr-foo/utils/gr-perf-to-csv'
output_filename = 'output.perf'
timeout = 60
command = ""
profiler = "perf_stat"

# Read command line args
arguments, values = getopt.getopt(sys.argv[1:],"t:c:o:p:h")

for currentArgument, currentValue in arguments:
    if currentArgument == '-t':
        timeout = currentValue
    elif currentArgument == '-c':
        command = currentValue
    elif currentArgument == '-o':
        output_filename = currentValue
    elif currentArgument == '-p':
        profiler = currentValue
    else:
        usage()
        exit(0)

print('Command:', command)
print('Profiler:', profiler.strip())
print('Timeout:', timeout)
print('Output filename:', output_filename)


pids = []
kill_process = False
if (not command):
    
    # Wait a few seconds until everything is up and running
    time.sleep(5)

    # Get ROS nodes' PIDs
    node_list = subprocess.check_output(['rosnode', 'list'])
    for node in node_list.split('\n'):
        if ('gazebo' in node) or ('rosout' in node) or ('player' in node) or (not node):
        #if not 'ros_interface' in node:
            # Ignore not relevant nodes
            continue
        node_info = subprocess.check_output('rosnode info ' + node + '| grep Pid', shell=True)
        key, pid = node_info.split(': ')
        pids.append(pid.rstrip())

else:
    # Start the specified command and get its PID
    FNULL = open(os.devnull, 'w')
    p1 = subprocess.Popen(command.split(), stdout=FNULL, stderr=subprocess.STDOUT)
    #p1 = subprocess.Popen(command.split(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    pids.append(str(p1.pid))
    time.sleep(5)
    kill_process = True
    
    #controlport_found = False
    #while not controlport_found:
    #    line = p1.stderr.readline()
    #    if 'controlport' in line:
    #        m = re.search('-p (\d+)', line.rstrip())
    #        port = m.group(1)
    #        print('ControlPort found:', port)
    #        controlport_found = True
    #
    #fout = open(str(p1.pid) + '.csv','w');
    #p2 = subprocess.Popen([gr_perf_to_csv, '127.0.0.1', str(port)], stdout=fout, stderr=subprocess.STDOUT)

#################################################################################################
# IMPORTANT! The profiler command has to run with root privileges. An 'easy' way to allow this
# is to add the user to the sudoers file for the specific profiler command. For example:
#   username ALL=(ALL) NOPASSWD: /usr/bin/perf

if (profiler.strip() == "perf_stat"):
    perf_command   = 'sudo /usr/bin/perf stat -d -d -d --per-thread --field-separator , --output=' + output_filename + ' --pid=' + ",".join(pids) + ' sleep ' + str(timeout)
    #perf_command   = 'sudo /usr/bin/perf stat -d --output=' + output_filename + ' --pid=' + ",".join(pids) + ' sleep ' + str(timeout)
    report_command = 'Use the following command to generate a basic report: [cat ' + output_filename + ']'
elif (profiler.strip() == "perf_record"):
    perf_command   = 'sudo /usr/bin/perf record -g -F 997 --output=' + output_filename + ' --pid=' + ",".join(pids) + ' sleep ' + str(timeout)
    report_command = 'Use the following command to generate a basic report: [sudo perf report --sort comm -i ' + output_filename + ']'
elif (profiler.strip() == "operf"):
    perf_command = 'sudo /usr/bin/timeout ' + str(timeout) + 's /usr/bin/operf --separate-thread --pid ' + ",".join(pids)
    report_command = 'Use the following command to generate a basic report: [opreport --demangle=smart --symbols --threshold 1]'
else:
    print('[profile_era] Profiler ' + profiler + ' not supported!')
    sys.exit(1)    

print('[profile_era] Running profiler for ' + str(timeout) + ' secs: [' + perf_command + ']')
subprocess.call(perf_command, shell=True)
print('[profile_era] Profiling completed. ' + report_command)

if kill_process:
    print('[profile_era] Stopping process', str(p1.pid))
    p1.kill()
#    p2.kill()
#
#fout.close()
