#!/usr/bin/env python3
"""Signal-quality statistics: single-sample glitches in notch and wheel velocity."""
import sys, pickle
sys.path.insert(0, '/home/ros/tram_ws/tools')
import numpy as np
from scipy.signal import medfilt

runs = pickle.load(open('/home/ros/tram_ws/calib/runs.pkl', 'rb'))
SC = 1.0 / 3.6

tot = spike_n = 0
gl = []
for r in runs:
    n = r['notch']
    nb = medfilt(n, 5)
    d = np.abs(n - nb)
    s = d > 0.5
    tot += len(n)
    spike_n += int(s.sum())
    if s.any():
        i = np.where(s)[0]
        b = np.where(np.diff(i) > 2)[0]
        st = np.r_[i[0], i[b + 1]]
        en = np.r_[i[b], i[-1]]
        gl.append(en - st + 1)
gl = np.concatenate(gl) if gl else np.array([1])
print(f'notch: {spike_n}/{tot} samples differ from 5-median = {100.0*spike_n/tot:.4f} %')
print(f'  glitch run lengths: median {np.median(gl):.0f}  p90 {np.percentile(gl,90):.0f}  max {gl.max()}  '
      f'frac single-sample {np.mean(gl == 1):.3f}')

# wheel velocity glitches
tot = sp = 0
mx = []
for r in runs:
    v = r['front'] * SC
    b = medfilt(v, 5)
    d = np.abs(v - b)
    s = d > 0.5
    tot += len(v); sp += int(s.sum())
    mx.append(d.max())
print(f'wheel: {sp}/{tot} samples differ from 5-median by >0.5 m/s = {100.0*sp/tot:.4f} %  max dev {max(mx):.2f} m/s')

# time gaps
for key, nm in (('front', 'front'), ('rear', 'rear'), ('notch', 'notch')):
    gaps = []
    for r in runs:
        d = np.diff(r['t'])
        gaps.append(d[d > 0.5])
    g = np.concatenate(gaps) if gaps else np.array([])
    if len(g):
        print(f'{nm}: gaps>0.5s: n={len(g)} max={g.max():.2f}s  total={g.sum():.0f}s')
    else:
        print(f'{nm}: no gaps > 0.5 s (grid resampled at 20 Hz)')

# duplicate / non-monotonic stamps in the raw bag?
print('\nrecomputing raw stamp gaps for 3 bags...')
sys.path.insert(0, '/home/ros/tram_ws/tools')
from bag_io import read_run, find_bags
for nm in ('30618_0e41eac3', '30639_e4379d7f', '30618_27e994fc'):
    b = [x for x in find_bags('/home/ros/Downloads/dataset/data') if nm in x.name][0]
    r = read_run(b)
    for k in ('front', 'rear', 'notch'):
        t = r.t[k]
        d = np.diff(t)
        print(f'  {nm} {k:6s}: n={len(t)} rate={1/np.median(d):.2f}Hz dt_min={d.min():.4f} dt_max={d.max():.3f} '
              f'dt<=0:{(d<=0).sum()} gaps>0.5:{(d>0.5).sum()}')
