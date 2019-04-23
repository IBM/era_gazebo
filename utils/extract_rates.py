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
from __future__ import division
import subprocess
import re
import os

command = 'timeout 30s /home/augustojv/catkin_ws/src/dsrc/gr-ieee802-11/examples/wifi_transceiver_standalone.py --encoding 0 --freq 5.89G --samp-rate 10M --tx-gain 0.75 --rx-gain 0.75 --chan-est 0 --lo-offset 0 --msg-count 200000 --pdu-length 1500 --strobe-period '
strobe_periods_ms = [1000, 500, 100, 50, 40, 30, 20, 10, 8, 6, 4, 2, 1]

FNULL = open(os.devnull, 'w')

print('Strobe Rate (Hz)\tInjection Rate (bits/sec)\tProcessing Rate (bits/sec)')
for strobe_period_ms in strobe_periods_ms:
    try:
        tmp = command + str(strobe_period_ms)
        #print('Running:', tmp)
        output = subprocess.check_output(tmp, shell=True, stderr=FNULL)
    except subprocess.CalledProcessError as e:
        # timeout always returns 124 which is not an error (so we process the output here)
        for line in e.output.splitlines():
            if 'Rate' in line:
                m = re.search('Rate \(bits/sec\): ([\d\.e\+]+)', line)
                rate_avg = m.group(1)
        strobe_rate    = 1000/strobe_period_ms
        injection_rate =  1500 * 8 * strobe_rate;
        print(str(strobe_rate) + '\t' + str(injection_rate) + '\t' + rate_avg)
