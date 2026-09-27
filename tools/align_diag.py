"""Диагностика начальной привязки: остаток, неоднозначность, ветвь карты.

Прогоняет окно привязки по всем прогонам датасета и печатает распределение
остатка сопоставления и коэффициента неоднозначности (вторая по качеству
гипотеза / лучшая).  Используется для выбора порогов `match_max_residual_m`
и `match_max_ambiguity`.
"""
import sys
import math
import os

sys.path.insert(0, '/home/ros/tram_ws/src/tram_backup_odometry')
import numpy as np

from tram_backup_odometry.params import Config, default_path
from tram_backup_odometry import replay as R
from tram_backup_odometry.core import BackupOdometryCore

DATA = '/home/ros/Downloads/dataset/data'
WINDOW = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
LIMIT = int(sys.argv[2]) if len(sys.argv) > 2 else 0

cfg = Config()
cfg.model.model_file = default_path('model.json')
cfg.route.map_file = default_path('route_map.json')
cfg.route.gnss_init_window_s = WINDOW

bags = sorted(d for d in os.listdir(DATA)
              if os.path.isfile(os.path.join(DATA, d, 'metadata.yaml')))
if LIMIT:
    bags = bags[:LIMIT]

rows = []
for name in bags:
    cfg.route.gnss_init_window_s = WINDOW
    res = R.replay(os.path.join(DATA, name), cfg, with_reference=False)
    if not np.isfinite(res.align_ambiguity):
        continue
    from tram_backup_odometry.route import MatchResult
    r = res.diag_align
    rows.append((name, r.ok, r.weak, r.ambiguous, r.residual, r.ambiguity,
                 r.span_m, r.branch, r.reverse, math.degrees(r.yaw), r.ratio))

print(f'window {WINDOW:.0f} s, {len(rows)} runs with a match attempt')
print(f'{"bag":16}{"ok":>4}{"weak":>6}{"amb":>5}{"resid":>8}{"ambig":>8}'
      f'{"span_m":>8}{"br":>4}{"rev":>5}{"yaw":>8}{"ratio":>8}')
for r in sorted(rows, key=lambda x: x[4]):
    print(f'{r[0]:16}{int(r[1]):4d}{int(r[2]):6d}{int(r[3]):5d}{r[4]:8.2f}'
          f'{r[5]:8.2f}{r[6]:8.0f}{r[7]:4d}{int(r[8]):5d}{r[9]:8.2f}{r[10]:8.4f}')

a = np.array([[r[4], r[5], r[6]] for r in rows])
ok = np.array([r[1] for r in rows])
print(f'\nresidual  : med {np.median(a[:,0]):.2f}  p75 {np.percentile(a[:,0],75):.2f}  '
      f'p90 {np.percentile(a[:,0],90):.2f}  max {a[:,0].max():.2f}')
print(f'ambiguity : med {np.median(a[:,1]):.2f}  p75 {np.percentile(a[:,1],75):.2f}  '
      f'p90 {np.percentile(a[:,1],90):.2f}')
print(f'span_m    : med {np.median(a[:,2]):.0f}  min {a[:,2].min():.0f}')
print(f'accepted  : {int(ok.sum())}/{len(ok)}  weak {int(sum(r[2] for r in rows))}  '
      f'ambiguous {int(sum(r[3] for r in rows))}')
