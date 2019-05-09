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
txonly_usrp_volk_enc0 = read_file("outputs_v3/txonly_usrp_volk_10M_enc0.csv", keyword, duration)
txonly_usrp_novolk_enc0 = read_file("outputs_v3/txonly_usrp_novolk_10M_enc0.csv", keyword, duration)
txonly_nousrp_volk_enc0 = read_file("outputs_v3/txonly_nousrp_volk_10M_enc0.csv", keyword, duration)
txonly_nousrp_novolk_enc0 = read_file("outputs_v3/txonly_nousrp_novolk_10M_enc0.csv", keyword, duration)

txonly_usrp_volk_enc5 = read_file("outputs_v3/txonly_usrp_volk_10M_enc5.csv", keyword, duration)
txonly_usrp_novolk_enc5 = read_file("outputs_v3/txonly_usrp_novolk_10M_enc5.csv", keyword, duration)
txonly_nousrp_volk_enc5 = read_file("outputs_v3/txonly_nousrp_volk_10M_enc5.csv", keyword, duration)
txonly_nousrp_novolk_enc5 = read_file("outputs_v3/txonly_nousrp_novolk_10M_enc5.csv", keyword, duration)

rxonly_usrp_volk_noise = read_file("outputs_v3/rxonly2_usrp_volk_10M.csv", keyword, duration)
rxonly_usrp_novolk_noise = read_file("outputs_v3/rxonly2_usrp_novolk_10M.csv", keyword, duration)
rxonly_nousrp_volk_noise = read_file("outputs_v3/rxonly2_nousrp_volk_10M.csv", keyword, duration)
rxonly_nousrp_novolk_noise = read_file("outputs_v3/rxonly2_nousrp_novolk_10M.csv", keyword, duration)

rxonly_usrp_volk_enc0 = read_file("outputs_v3/rxonly2_usrp_volk_10M_enc0.csv", keyword, duration)
rxonly_usrp_novolk_enc0 = read_file("outputs_v3/rxonly2_usrp_novolk_10M_enc0.csv", keyword, duration)
rxonly_nousrp_volk_enc0 = read_file("outputs_v3/rxonly2_nousrp_volk_10M_enc0.csv", keyword, duration)
rxonly_nousrp_novolk_enc0 = read_file("outputs_v3/rxonly2_nousrp_novolk_10M_enc0.csv", keyword, duration)

rxonly_usrp_volk_enc5 = read_file("outputs_v3/rxonly2_usrp_volk_10M_enc5.csv", keyword, duration)
rxonly_usrp_novolk_enc5 = read_file("outputs_v3/rxonly2_usrp_novolk_10M_enc5.csv", keyword, duration)
rxonly_nousrp_volk_enc5 = read_file("outputs_v3/rxonly2_nousrp_volk_10M_enc5.csv", keyword, duration)
rxonly_nousrp_novolk_enc5 = read_file("outputs_v3/rxonly2_nousrp_novolk_10M_enc5.csv", keyword, duration)

# %% TX Only Plots Enc0
txonly_usrp_enc0 = merge_two_datas(txonly_usrp_volk_enc0, txonly_usrp_novolk_enc0)
txonly_nousrp_enc0 = merge_two_datas(txonly_nousrp_volk_enc0, txonly_nousrp_novolk_enc0)
txonly_enc0 = merge_two_datas(txonly_usrp_enc0, txonly_nousrp_enc0)
labels = ['usrp_volk', 'usrp_novolk', 'nousrp_volk', 'nousrp_novolk']
title = 'TX Only (240 packets/s) - 10MHz - Encoding BPSK'
fig = plot_bar_graph(txonly_enc0, labels, 14, keyword, title, fig_num=1, yminmax = [1e6, 6e9])

# %% TX Only Plots Enc5
txonly_usrp_enc5 = merge_two_datas(txonly_usrp_volk_enc5, txonly_usrp_novolk_enc5)
txonly_nousrp_enc5 = merge_two_datas(txonly_nousrp_volk_enc5, txonly_nousrp_novolk_enc5)
txonly_enc5 = merge_two_datas(txonly_usrp_enc5, txonly_nousrp_enc5)
labels = ['usrp_volk', 'usrp_novolk', 'nousrp_volk', 'nousrp_novolk']
title = 'TX Only (950 packets/s) - 10MHz - Encoding 16QAM'
fig = plot_bar_graph(txonly_enc5, labels, 14, keyword, title, fig_num=2, yminmax = [1e6, 6e9])

# %% TX Only Plots Enc0 versus Enc5
txonly_usrp_enc5 = merge_two_datas(txonly_usrp_volk_enc5, txonly_usrp_novolk_enc5)
txonly_usrp_enc0 = merge_two_datas(txonly_usrp_volk_enc0, txonly_usrp_novolk_enc0)
txonly_usrp = merge_two_datas(txonly_usrp_enc0, txonly_usrp_enc5)
labels = ['usrp_volk_bpsk', 'usrp_novolk_bpsk', 'usrp_volk_16qam', 'usrp_novolk_16qam']
title = 'TX Only - 10MHz - BPSK versus 16QAM'
fig = plot_bar_graph(txonly_usrp, labels, 12, keyword, title, fig_num=3, yminmax = [1e6, 6e9])

# %% TX Only Plots Enc0 versus Enc5 - simplified
txonly_usrp_volk = merge_two_datas(txonly_usrp_volk_enc0, txonly_usrp_volk_enc5)
labels = ['BPSK', '16-QAM']
title = 'TX Only - 10MHz - BPSK versus 16QAM'
fig = plot_bar_graph(txonly_usrp_volk, labels, 12, keyword, title, fig_num=4, yminmax = [1e6, 6e9])

# %% RX Only 2 Enc0
rxonly_usrp_enc0 = merge_two_datas(rxonly_usrp_volk_enc0, rxonly_usrp_novolk_enc0)
rxonly_nousrp_enc0 = merge_two_datas(rxonly_nousrp_volk_enc0, rxonly_nousrp_novolk_enc0)
rxonly_enc0 = merge_two_datas(rxonly_usrp_enc0, rxonly_nousrp_enc0)
labels = ['usrp_volk', 'usrp_novolk', 'nousrp_volk', 'nousrp_novolk']
title = 'RX Only (240 packets/s) - 10MHz - BPSK'
fig = plot_bar_graph(rxonly_enc0, labels, 0, keyword, title, 5, yminmax = [1e6, 6e9])

# %% RX Only 2 Enc5
rxonly_usrp_enc5 = merge_two_datas(rxonly_usrp_volk_enc5, rxonly_usrp_novolk_enc5)
rxonly_nousrp_enc5 = merge_two_datas(rxonly_nousrp_volk_enc5, rxonly_nousrp_novolk_enc5)
rxonly_enc5 = merge_two_datas(rxonly_usrp_enc5, rxonly_nousrp_enc5)
labels = ['usrp_volk', 'usrp_novolk', 'nousrp_volk', 'nousrp_novolk']
title = 'RX Only (950 packets/s) - 10MHz - 16QAM'
fig = plot_bar_graph(rxonly_enc5, labels, 0, keyword, title, 6, yminmax = [1e6, 6e9])

# %% RX Only 2 Noise versus Enc0 versus Enc5
rxonly_usrp_noise = merge_two_datas(rxonly_usrp_volk_noise, rxonly_usrp_novolk_noise)
rxonly_usrp_enc0 = merge_two_datas(rxonly_usrp_volk_enc0, rxonly_usrp_novolk_enc0)
rxonly_usrp_enc5 = merge_two_datas(rxonly_usrp_volk_enc5, rxonly_usrp_novolk_enc5)
rxonly_usrp_enc05 = merge_two_datas(rxonly_usrp_enc0, rxonly_usrp_enc5)
rxonly_usrp = merge_two_datas(rxonly_usrp_enc05, rxonly_usrp_noise)
labels = ['usrp_volk_bpsk', 'usrp_novolk_bpsk', 'usrp_volk_16qam', 'usrp_novolk_16qam', 'usrp_volk_noise', 'usrp_novolk_noise']
title = 'RX Only - 10MHz - Noise versus BPSK versus 16QAM'
fig = plot_bar_graph(rxonly_usrp, labels, 0, keyword, title, 7, yminmax = [1e6, 6e9])

# %% RX Only 2 Noise versus Enc0 versus Enc5 - simplified
rxonly_usrp_volk_enc05 = merge_two_datas(rxonly_usrp_volk_enc0, rxonly_usrp_volk_enc5)
rxonly_usrp_volk = merge_two_datas(rxonly_usrp_volk_enc05, rxonly_usrp_volk_noise)
labels = ['BPSK', '16-QAM', 'Noise']
title = 'RX Only - 10MHz - Noise versus BPSK versus 16QAM'
fig = plot_bar_graph(rxonly_usrp_volk, labels, 0, keyword, title, 8, yminmax = [1e6, 6e9])

# %% TX RX with USRP - BPSK
txonly_usrp_enc0 = merge_two_datas(txonly_usrp_volk_enc0, txonly_usrp_novolk_enc0)
rxonly_usrp_enc0 = merge_two_datas(rxonly_usrp_volk_enc0, rxonly_usrp_novolk_enc0)
rxtx_usrp_enc0   = merge_two_datas(rxonly_usrp_enc0, txonly_usrp_enc0)
labels = ['rx_volk', 'rx_novolk', 'tx_volk', 'tx_novolk']
title = 'RXonly vs TXonly (240 packets/s) - 10MHz BPSK'
fig = plot_bar_graph(rxtx_usrp_enc0, labels, 27, keyword, title, 9, yminmax = [1e6, 6e9], figsize=(24,5))

# %% TX RX with USRP - 16-QAM
txonly_usrp_enc5 = merge_two_datas(txonly_usrp_volk_enc5, txonly_usrp_novolk_enc5)
rxonly_usrp_enc5 = merge_two_datas(rxonly_usrp_volk_enc5, rxonly_usrp_novolk_enc5)
rxtx_usrp_enc5   = merge_two_datas(rxonly_usrp_enc5, txonly_usrp_enc5)
labels = ['rx_volk', 'rx_novolk', 'tx_volk', 'tx_novolk']
title = 'RXonly vs TXonly (950 packets/s) - 10MHz 16-QAM'
fig = plot_bar_graph(rxtx_usrp_enc5, labels, 27, keyword, title, 10, yminmax = [1e6, 6e9], figsize=(24,5))

# %% RX versus TX - BPSK simplified
rxtx_usrp_volk_enc0 = merge_two_datas(rxonly_usrp_volk_enc0, txonly_usrp_volk_enc0)
labels = ['RX', 'TX']
title = 'RXonly vs TXonly (240 packets/s) - 10MHz BPSK'
fig = plot_bar_graph(rxtx_usrp_volk_enc0, labels, 27, keyword, title, 11, yminmax = [1e6, 6e9], figsize=(24,5))

# %% RX versus TX - 16-QAM simplified
rxtx_usrp_volk_enc5 = merge_two_datas(rxonly_usrp_volk_enc5, txonly_usrp_volk_enc5)
labels = ['RX', 'TX']
title = 'RXonly vs TXonly (950 packets/s) - 10MHz 16-QAM'
fig = plot_bar_graph(rxtx_usrp_volk_enc5, labels, 27, keyword, title, 12, yminmax = [1e6, 6e9], figsize=(24,5))

# %% RX versus TX - simplified
txonly_usrp_volk_enc05 = merge_two_datas(txonly_usrp_volk_enc0, txonly_usrp_volk_enc5)
rxonly_usrp_volk_enc05 = merge_two_datas(rxonly_usrp_volk_enc0, rxonly_usrp_volk_enc5)
rxtx_usrp_volk_enc05 = merge_two_datas(rxonly_usrp_volk_enc05, txonly_usrp_volk_enc05)
labels = ['RX - BPSK', 'RX - 16-QAM', 'TX - BPSK', 'TX - 16-QAM']
title = 'RXonly vs TXonly - 10MHz - BPSK and 16-QAM'
fig = plot_bar_graph(rxtx_usrp_volk_enc05, labels, 27, keyword, title, 13, yminmax = [1e6, 6e9], figsize=(24,5))

# %% Rates
import matplotlib.pyplot as plt
plt.rc('pdf',fonttype = 42)
plt.rc('ps',fonttype = 42)
plt.rcParams['mathtext.fontset']='custom'
plt.rcParams['mathtext.default']='sf'
plt.rcParams['mathtext.sf']='sans\-serif:normal:bold:normal:normal:normal'

period = [1000, 500, 100, 50, 20, 10, 5, 2, 1]
rate_in = [1500*8.0*1000/x for x in period]
rate_rx_enc0 = [0.9999, 1.9999, 9.9951, 19.960, 49.916, 99.293, 197.81, 236.89, 234.89]
rate_rx_enc0 = [1500*8*x for x in rate_rx_enc0]
rate_rx_enc5 = [0.9999, 1.9999, 9.9918, 19.973, 49.803, 99.278, 197.13, 488.52, 952.03]
rate_rx_enc5 = [1500*8*x for x in rate_rx_enc5]

fig = plt.figure(14, figsize = (6,4))
fig.clf()
ax = fig.add_subplot(1,1,1)
ax.plot(rate_in, rate_rx_enc0, linewidth = 2, marker = 'o', label = 'BPSK')
ax.plot(rate_in, rate_rx_enc5, linewidth = 2, marker = 'd', label = '16-QAM')
ax.set_ylabel('RX Bit Rate (bps)', fontsize = 16)
ax.set_xlabel('Injetion Rate (bps)', fontsize = 16)
ax.set_yscale('log')
ax.set_xscale('log')
ax.set_xlim([1e4, 2e7])
ax.set_ylim([1e4, 2e7])
ax.tick_params(axis='both', which='major', labelsize=15)
ax.grid(axis = 'both')
ax.legend(loc='best', fontsize = 15)
plt.tight_layout()
plt.pause(0.05)

# %%
input('Press any key to exit program:')
