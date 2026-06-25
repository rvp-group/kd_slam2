#!/usr/bin/env python3
"""
Interactive color-matrix visualization of descriptor_experiment output.

File columns (after '#' header lines):
  i  j  best_med_canon_j  med_dist  best_euc_canon_j  euc_dist  gt_dist[m]

Keyboard:
  c  -- reset cutoff to 95th-percentile of the current field
  q  -- quit
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import RadioButtons, Slider


# ------------------------------------------------------------------ #
# I/O                                                                  #
# ------------------------------------------------------------------ #

def load(path):
    """Return (N, mats) where mats maps field-name -> symmetric NxN array."""
    pairs = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            p = line.split()
            pairs.append((int(p[0]), int(p[1]),
                          float(p[3]),   # med_dist
                          float(p[5]),   # euc_dist
                          float(p[6])))  # gt_dist

    if not pairs:
        raise ValueError(f"No data rows found in {path}")

    N = max(max(a, b) for a, b, *_ in pairs) + 1
    print(f"  {len(pairs)} pairs, N={N} clouds")

    mats = {
        'med_dist': np.full((N, N), np.nan),
        'euc_dist': np.full((N, N), np.nan),
        'gt_dist':  np.full((N, N), np.nan),
    }
    # Diagonal = 0 (self-distance)
    for m in mats.values():
        np.fill_diagonal(m, 0.0)

    for i, j, med, euc, gt in pairs:
        mats['med_dist'][i, j] = mats['med_dist'][j, i] = med
        mats['euc_dist'][i, j] = mats['euc_dist'][j, i] = euc
        mats['gt_dist' ][i, j] = mats['gt_dist' ][j, i] = gt

    return N, mats


# ------------------------------------------------------------------ #
# Helpers                                                              #
# ------------------------------------------------------------------ #

FIELD_NAMES = ['med_dist', 'euc_dist', 'gt_dist']


def pct_cutoff(mat, pct=95):
    """pct-th percentile of finite, positive values."""
    vals = mat[np.isfinite(mat) & (mat > 0)]
    return float(np.percentile(vals, pct)) if len(vals) else 1.0


# ------------------------------------------------------------------ #
# Main                                                                 #
# ------------------------------------------------------------------ #

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <output.txt>")
        sys.exit(1)

    path = sys.argv[1]
    print(f"Loading {path} ...")
    N, mats = load(path)

    # Mutable state (used inside closures)
    state = {
        'field':  FIELD_NAMES[0],
        'cutoff': pct_cutoff(mats[FIELD_NAMES[0]]),
    }

    # ---- Figure layout ----
    fig = plt.figure(figsize=(13, 8))
    fig.suptitle(path, fontsize=8, color='gray')

    ax_mat  = fig.add_axes([0.05, 0.10, 0.60, 0.82])   # matrix
    ax_cbar = fig.add_axes([0.67, 0.10, 0.02, 0.82])   # colorbar
    ax_rad  = fig.add_axes([0.72, 0.55, 0.26, 0.38])   # radio buttons
    ax_sld  = fig.add_axes([0.72, 0.22, 0.24, 0.05])   # cutoff slider
    ax_info = fig.add_axes([0.72, 0.10, 0.26, 0.10])   # stats text

    # ---- Color map: NaN / missing -> dark grey ----
    cmap = plt.cm.viridis.copy()
    cmap.set_bad('#333333')

    def clipped():
        return np.clip(mats[state['field']], 0.0, state['cutoff'])

    img = ax_mat.imshow(clipped(), aspect='auto', cmap=cmap,
                        interpolation='nearest',
                        vmin=0.0, vmax=state['cutoff'])
    ax_mat.set_title(state['field'], fontsize=11)
    ax_mat.set_xlabel('cloud j')
    ax_mat.set_ylabel('cloud i')

    cbar = fig.colorbar(img, cax=ax_cbar)

    # ---- Stats box ----
    ax_info.axis('off')
    stats_text = ax_info.text(0.05, 0.95, '', va='top', ha='left',
                              fontsize=8, transform=ax_info.transAxes,
                              family='monospace')

    def update_stats():
        m = mats[state['field']]
        vals = m[np.isfinite(m) & (m > 0)]
        if len(vals):
            txt = (f"min  {vals.min():.4f}\n"
                   f"max  {vals.max():.4f}\n"
                   f"med  {np.median(vals):.4f}\n"
                   f"p95  {np.percentile(vals, 95):.4f}")
        else:
            txt = "(no data)"
        stats_text.set_text(txt)

    update_stats()

    # ---- Radio buttons ----
    radio = RadioButtons(ax_rad, FIELD_NAMES, active=0, activecolor='steelblue')
    ax_rad.set_title('Field', fontsize=9, pad=4)

    # ---- Cutoff slider ----
    sld_max = max(pct_cutoff(mats[f], 99) for f in FIELD_NAMES) * 2.0
    slider = Slider(ax_sld, 'cutoff', 0.0, sld_max,
                    valinit=state['cutoff'])
    slider.valtext.set_fontsize(8)

    # ---- Redraw ----
    def redraw():
        c = state['cutoff']
        img.set_data(clipped())
        img.set_clim(vmin=0.0, vmax=c)
        ax_mat.set_title(state['field'], fontsize=11)
        update_stats()
        fig.canvas.draw_idle()

    # ---- Callbacks ----
    def on_radio(label):
        state['field']  = label
        state['cutoff'] = pct_cutoff(mats[label])
        slider.set_val(state['cutoff'])   # triggers on_slider -> redraw

    def on_slider(val):
        state['cutoff'] = val
        redraw()

    def on_key(event):
        if event.key == 'c':
            state['cutoff'] = pct_cutoff(mats[state['field']])
            slider.set_val(state['cutoff'])
        elif event.key == 'q':
            plt.close('all')

    radio.on_clicked(on_radio)
    slider.on_changed(on_slider)
    fig.canvas.mpl_connect('key_press_event', on_key)

    plt.show()


if __name__ == '__main__':
    main()
