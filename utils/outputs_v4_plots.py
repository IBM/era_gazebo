# -*- coding: utf-8 -*-
"""
Created on Wed May  1 13:01:46 2019

@author: ARUNPAIDIMARRI
"""

from parse_plot_perf_stat import read_file
from parse_plot_perf_stat import merge_two_datas
from parse_plot_perf_stat import plot_bar_graph

# %% Common parameters
duration = 100
keyword = 'cycles'

# %% Read files
txonly_usrp_volk_enc0_strobe100 = read_file("outputs_v4/txonly_usrp_volk_10M_enc0_strobe100.csv", keyword, duration)
txonly_usrp_volk_enc0_strobe10  = read_file("outputs_v4/txonly_usrp_volk_10M_enc0_strobe10.csv",  keyword, duration)
txonly_usrp_volk_enc0_strobe1   = read_file("outputs_v4/txonly_usrp_volk_10M_enc0_strobe1.csv",   keyword, duration)

txonly_usrp_volk_enc5_strobe100 = read_file("outputs_v4/txonly_usrp_volk_10M_enc5_strobe100.csv", keyword, duration)
txonly_usrp_volk_enc5_strobe10  = read_file("outputs_v4/txonly_usrp_volk_10M_enc5_strobe10.csv",  keyword, duration)
txonly_usrp_volk_enc5_strobe1   = read_file("outputs_v4/txonly_usrp_volk_10M_enc5_strobe1.csv",   keyword, duration)

# %% TX Only Plots Enc0
temp = merge_two_datas(txonly_usrp_volk_enc0_strobe100, txonly_usrp_volk_enc0_strobe10)
txonly_usrp_volk_enc0 = merge_two_datas(temp, txonly_usrp_volk_enc0_strobe1)
labels = ['10 pkts / second', '100 pkts / second', '240 pkts / second']
title = 'TX Only - 10MHz - BPSK - Effect of Packet Rates'
fig = plot_bar_graph(txonly_usrp_volk_enc0, labels, 12, keyword, title, fig_num=1, yminmax = [1e6, 6e9])

# %% TX Only Plots Enc5
temp = merge_two_datas(txonly_usrp_volk_enc5_strobe100, txonly_usrp_volk_enc5_strobe10)
txonly_usrp_volk_enc5 = merge_two_datas(temp, txonly_usrp_volk_enc5_strobe1)
labels = ['10 pkts / second', '100 pkts / second', '950 pkts / second']
title = 'TX Only - 10MHz - 16QAM - Effect of Packet Rates'
fig = plot_bar_graph(txonly_usrp_volk_enc5, labels, 12, keyword, title, fig_num=2, yminmax = [1e6, 6e9])

# %% TX Only Plots Enc0 versus Enc5 - strobe = 100
txonly_usrp_volk = merge_two_datas(txonly_usrp_volk_enc0_strobe100, txonly_usrp_volk_enc5_strobe100)
labels = ['BPSK', '16-QAM']
title = 'TX Only - 10MHz - BPSK versus 16QAM - 10 packets / second'
fig = plot_bar_graph(txonly_usrp_volk, labels, 12, keyword, title, fig_num=3, yminmax = [1e6, 6e9])

# %% TX Only Plots Enc0 versus Enc5 - strobe = 10
txonly_usrp_volk = merge_two_datas(txonly_usrp_volk_enc0_strobe10, txonly_usrp_volk_enc5_strobe10)
labels = ['BPSK', '16-QAM']
title = 'TX Only - 10MHz - BPSK versus 16QAM - 100 packets / second'
fig = plot_bar_graph(txonly_usrp_volk, labels, 12, keyword, title, fig_num=4, yminmax = [1e6, 6e9])

# %% TX Only Plots Enc0 versus Enc5 - strobe = 1
txonly_usrp_volk = merge_two_datas(txonly_usrp_volk_enc0_strobe1, txonly_usrp_volk_enc5_strobe1)
labels = ['BPSK', '16-QAM']
title = 'TX Only - 10MHz - BPSK versus 16QAM - Max datarate'
fig = plot_bar_graph(txonly_usrp_volk, labels, 12, keyword, title, fig_num=5, yminmax = [1e6, 6e9])

# %%
input('Press any key to exit program:')
