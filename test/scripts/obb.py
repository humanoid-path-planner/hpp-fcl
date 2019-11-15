import matplotlib.pyplot as plt
import csv, sys, numpy as np
from math import sqrt

filename = sys.argv[1] 

with open(filename, 'r') as file:
    reader = csv.reader (file, strict = True)
    fieldnames = None
    #fieldnames = reader.fieldnames
    for row in reader:
        if fieldnames is None:
            fieldnames = [ n.strip() for n in row]
            values = [ [] for _ in fieldnames ]
            continue

        values[0].append (int(row[0]))
        for i, v in enumerate(row[1:]):
            values[i+1].append (float(v))

# Compute mean and variance for each values, for each separating axis
means = [ [ 0., ] * 12 for _ in fieldnames[4:] ]
stddevs = [ [ 0., ] * 12 for _ in fieldnames[4:] ]
nb_occurence = [ 0, ] * 12

for i, id in enumerate(values[0]):
    nb_occurence[id] += 1

for i, id in enumerate(values[0]):
    for k, n in enumerate(fieldnames[4:]):
        v = values[k+4][i]
        means  [k][id] += v / nb_occurence[id]
        stddevs[k][id] += v * v / nb_occurence[id]

for k, n in enumerate(fieldnames[4:]):
    for id in range(12):
        #means  [k][id] /= nb_occurence[id]
        #stddevs[k][id] = sqrt (stddevs[k][id]) / nb_occurence[id] - means[k][id])
        stddevs[k][id] = sqrt (stddevs[k][id] - means[k][id]*means[k][id])

subplots = False
Nrows = 1
Ncols = 3
iplot = 1
time_vs_sep_axis = True
nb_occ_sep_axis = False
avg_time_vs_impl = True

if time_vs_sep_axis:
    if subplots: plt.subplot (Nrows, Ncols, iplot)
    else:        plt.figure (iplot)
    plt.title ("Time, with std dev, versus separating axis")
    for k, n in enumerate(fieldnames[4:]):
        #plt.errorbar ([ np.linspace(0, 11, 12) + shift for shift in np.linspace (-0.2, 0.2, ) ], means[k], stddevs[k], label=n)
        plt.errorbar (np.linspace(0, 11, 12), means[k], stddevs[k], label=n)
        # plt.errorbar (np.linspace(0, 11, 12), means[k],  [ [ 0 ] * len(stddevs[k]), stddevs[k] ], label=n)
    plt.xlim([-0.5,11.5])
    plt.ylabel('Time (ns)')
    plt.xlabel('Separating axis')
    plt.legend(loc='upper left')

    axx = plt.gca().twinx()
    axx.hist (values[0], bins=[ i-0.5 for i in range(13) ], bottom=-0.5, cumulative=True,
            rwidth=0.5, fill=False, label='Cumulative occurence')
    axx.set_ylabel('Nb occurence of a separating axis.')
    plt.legend(loc='lower right')

iplot += 1
if nb_occ_sep_axis:
    if subplots: plt.subplot (Nrows, Ncols, iplot)
    else:        plt.figure (iplot)
    plt.title ("Nb of occurence per separating axis")
    plt.hist (values[0], bins=[ i-0.5 for i in range(13) ])
    plt.ylabel('Nb occurence')
    plt.xlabel('Separating axis')
    dlb_id = 1
    d_id = 2
    #plt.title ("Time, with std dev, versus distance")
    #for k, n in enumerate(fieldnames[4:]):
        #plt.plot (values[dlb_id], values[k+4], '.', label=n)

iplot += 1
if avg_time_vs_impl:
    if subplots: plt.subplot (Nrows, Ncols, iplot)
    else:        plt.figure (iplot)
    plt.title ("Average time versus the implementation")
    #plt.boxplot(values[4:], labels=fieldnames[4:], showmeans=True)
    _mins = np.min (values[4:], axis=1)
    _maxs = np.max (values[4:], axis=1)
    _means = np.mean (values[4:], axis=1)
    _stddev = np.std (values[4:], axis=1)
    _sorted = sorted ( zip(fieldnames[4:], _means, _stddev, _mins, _maxs), key=lambda x: x[1])
    plt.errorbar(
            [ f for f,m,s,l,u in _sorted],
            [ m for f,m,s,l,u in _sorted],
            [ s for f,m,s,l,u in _sorted],
            fmt='go', linestyle='', label='mean and std deviation')
    plt.plot (
            [ f for f,m,s,l,u in _sorted],
            [ l for f,m,s,l,u in _sorted],
            'b+', ls='', label='min')
    plt.plot (
            [ f for f,m,s,l,u in _sorted],
            [ u for f,m,s,l,u in _sorted],
            'r+', ls='', label='max')
    plt.ylabel('Time (ns)')
    plt.xticks(rotation=20)
    plt.legend()

plt.show()
