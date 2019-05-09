#!/usr/bin/python3

"""
Created on Mon Feb 25 12:47:55 2019

@author: ARUNPAIDIMARRI
"""
import numpy as np
import matplotlib.pyplot as plt
import csv
import argparse
import itertools
plt.rc('pdf',fonttype = 42)
plt.rc('ps',fonttype = 42)
plt.rcParams['mathtext.fontset']='custom'
plt.rcParams['mathtext.default']='sf'
plt.rcParams['mathtext.sf']='sans\-serif:normal:bold:normal:normal:normal'

# %% Plot Comparison
def plot_bar_graph(datas, labels, num_bars, keyword, title, fig_num = None, yminmax = None, figsize = (12,5), display = True):
    keyword = keyword + '/sec'
    title = title + ' - ' + keyword
    if num_bars == 0:
        num_bars = len(datas)
    else:
        num_bars = min([num_bars, len(datas)])
    num_bars = len(datas) if num_bars == 0 else num_bars
    opacity = 0.7
    colors = itertools.cycle('bgrcmyk')
    
    data = sorted(datas, key = lambda x: max(x[1:]), reverse=True)
    if fig_num == None:
        fig_num = 3
    fig = plt.figure(fig_num, figsize)
    fig.clf()
    ax = fig.add_subplot(1,1,1)
    num_cols = len(data[0]) - 1
    bar_width = 1.0/(num_cols + 1)
    for idx in range(num_cols):
        xvals = np.arange(num_bars) + (idx)*bar_width
        values = [x[idx+1] for x in data[:num_bars]]
        ax.bar(xvals, values, bar_width, alpha = opacity, color = next(colors), label = labels[idx])
    ax.set_ylabel(keyword, fontsize = 16)
    #ax.set_xlabel('Function Name', fontsize = 15)
    ax.set_yscale('log')
    xtick_vals = np.arange(num_bars) + (num_cols-1)*bar_width/2
    func_names = [x[0] for x in data[:num_bars]]
    ax.set_xticks([])
    ax.set_xticks(xtick_vals, minor = False)
    ax.set_xticklabels(func_names, rotation=45, ha='right')
    if yminmax is not None:
        ax.set_ylim(yminmax)
    ax.tick_params(axis='both', which='major', labelsize=12)
    ax.grid(axis='y')
    ax.legend(loc='best')
    ax.set_title(title, fontsize = 16)
    plt.tight_layout()
    if display == True:
        plt.pause(0.05)
    return fig

# %%
def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

# %%    
def read_file(filename, keyword, duration):
    data = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for idx, line in enumerate(reader):
            if len(line) >= 8 and line[3]==keyword:
                if is_number(line[1]):
                    prog = line[0]
                    prog = prog[:prog.rfind('-')]
                    count = float(line[1])/duration
                    to_append = True
                    for dat in data:
                        if dat[0]==prog:
                            dat[1]+=count
                            to_append = False
                    if prog=='python2':
                        to_append = False
                    if to_append:
                        data.append([prog, count])
    print('Number of lines with keyword =', keyword, 'is', len(data))
    return data

# %%
def merge_two_datas(data1, data2):
    data = []
    data1size = len(data1[0])-1
    data2size = len(data2[0])-1
    data1dumm = [0 for i in range(data1size)]
    data2dumm = [0 for i in range(data2size)]
    data_extras = []
    for dat in data1:
        line = [dat[0]]
        line.extend(dat[1:])
        line.extend(data2dumm)
        data.append(line)
    for datnew in data2:
        to_append = True
        for dat in data:
            if dat[0] == datnew[0]:
                dat[-data2size:] = datnew[1:]
                to_append = False
        if to_append:
            line = [datnew[0]]
            line.extend(data1dumm)
            line.extend(datnew[1:])
            data_extras.append(line)
    for dat_e in data_extras:
        name = dat_e[0]
        for i in range(1,len(name)):
            if name[-i].isalpha():
                break
        if i > 1:
            name = name[:-i+1]
        to_append = True
        for dat in data:
            nam = dat[0]
            for i in range(1,len(nam)):
                if nam[-i].isalpha():
                    break
            if i > 1:
                nam = nam[:-i+1]
            if nam == name and dat[-1] == 0:
                dat[-data2size:] = dat_e[-data2size:]
                to_append = False
                break
        if to_append:
            data.append(dat_e)
    return data

# %%
def merge_datas(datas):
    data = datas[0]
    if len(data) > 1:
        for i in range(1, len(datas)):
            data = merge_two_datas(data, datas[i])
    return data

# %%
def argparser():
    parser = argparse.ArgumentParser(description = 'Process perf_stat CSV files to plots')
    parser.add_argument('files', metavar='FILE', nargs='+',
                        help='List of CSV files')
    parser.add_argument('-k', '--keyword', dest='keyword',
                        choices = ['cycles', 'instructions'],
                        default = 'cycles',
                        help='Keyword to search for in CSV files. Defaults to cycles')
    parser.add_argument('-d', '--duration', dest='duration', type=int, default=100,
                        help='Duration for which perf_stat was run for normalization. Default 100sec')
    parser.add_argument('-t', '--title', dest='title', default='temp',
                        help='Title of plot and name of output file. Defaults to temp.')
    parser.add_argument('-n', '--num_bars', dest='num_bars', default=0, type=int,
                        help='Number of bars to plot. Default (0) for all')
    parser.add_argument('-m', '--mode', dest='out_mode', default='disp',
                        choices = ['disp', 'save', 'both'],
                        help='Default mode is to only display plot. Optionally, save png file')
    return parser

# %%
def main():
    parser = argparser()
    args = parser.parse_args()
    datas = [read_file(f, args.keyword, args.duration) for f in args.files]
    data = merge_datas(datas)
    display_plot = (args.out_mode != 'save')
    save_image = (args.out_mode != 'disp')
    fig = plot_bar_graph(data, labels=args.files, num_bars=args.num_bars,
                         keyword=args.keyword, title=args.title, fig_num=1,
                         yminmax = None, display = (args.out_mode != 'save'))
    if save_image:
        fig.savefig(args.title+'.png')
        print('Output plot is at:', args.title+'.png')
    input('Press any key to quit program')
    plt.close(fig)
    
# %%
if __name__ == '__main__':
    main()
