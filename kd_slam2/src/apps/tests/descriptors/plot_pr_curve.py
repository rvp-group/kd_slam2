#!/usr/bin/env python3
"""
Interactive precision/recall curve from descriptor_compare output.

File columns (after '#' header lines):
  i  j  best_med_canon_j  med_dist  best_euc_canon_j  euc_dist  gt_dist[m]

Controls:
  - Radio buttons : select descriptor field (med_dist / euc_dist)
  - gt slider     : ground-truth distance threshold (what counts as a true match)
  - desc slider   : mark a specific operating point on the curve
  q               : quit
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import RadioButtons, Slider


# ------------------------------------------------------------------ #
# I/O                                                                  #
# ------------------------------------------------------------------ #

def load(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            p = line.split()
            rows.append((float(p[3]),   # med_dist
                         float(p[5]),   # euc_dist
                         float(p[6])))  # gt_dist
    if not rows:
        raise ValueError(f"No data rows found in {path}")
    arr = np.array(rows, dtype=np.float64)
    print(f"  {len(rows)} pairs loaded")
    return arr   # shape (N, 3)


# ------------------------------------------------------------------ #
# PR computation                                                        #
# ------------------------------------------------------------------ #

FIELD_NAMES = ['med_dist', 'euc_dist']
FIELD_COL   = {'med_dist': 0, 'euc_dist': 1}

def compute_pr(arr, field_col, gt_threshold):
    """
    Return (desc_thresholds, precision, recall) arrays.
    The arrays are indexed so that at index k, precision[k] and recall[k]
    are the values obtained by retrieving all pairs with desc_dist <= desc_thresholds[k].
    A leading point (threshold=0, P=1, R=0) is prepended.
    """
    desc = arr[:, field_col]
    gt   = arr[:, 2]

    order       = np.argsort(desc, kind='stable')
    desc_sorted = desc[order]
    is_pos      = gt[order] < gt_threshold

    total_pos = int(is_pos.sum())
    if total_pos == 0:
        return np.array([0.0, desc_sorted[-1]]), \
               np.array([1.0, 1.0]), \
               np.array([0.0, 0.0])

    tp = np.cumsum(is_pos).astype(np.float64)
    fp = np.cumsum(~is_pos).astype(np.float64)

    precision = tp / (tp + fp)
    recall    = tp / total_pos

    thresholds = np.concatenate([[0.0],        desc_sorted])
    precision  = np.concatenate([[1.0],        precision])
    recall     = np.concatenate([[0.0],        recall])

    return thresholds, precision, recall


def find_op(thresholds, precision, recall, desc_thresh):
    """Return (P, R) at the operating point closest to desc_thresh."""
    idx = int(np.searchsorted(thresholds, desc_thresh, side='right')) - 1
    idx = max(0, min(idx, len(thresholds) - 1))
    return float(precision[idx]), float(recall[idx])


# ------------------------------------------------------------------ #
# Main                                                                 #
# ------------------------------------------------------------------ #

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <pairwise.txt>")
        sys.exit(1)

    path = sys.argv[1]
    print(f"Loading {path} ...")
    arr = load(path)

    state = {
        'field':          FIELD_NAMES[0],
        'gt_threshold':   5.0,
        'desc_threshold': float(np.percentile(arr[:, 0], 10)),
    }

    # ---- Figure layout ----
    fig = plt.figure(figsize=(13, 7))
    fig.suptitle(path, fontsize=8, color='gray')

    ax_pr   = fig.add_axes([0.07, 0.12, 0.55, 0.80])   # PR curve
    ax_rad  = fig.add_axes([0.68, 0.62, 0.28, 0.25])   # radio buttons
    ax_gt   = fig.add_axes([0.68, 0.45, 0.25, 0.05])   # gt slider
    ax_desc = fig.add_axes([0.68, 0.30, 0.25, 0.05])   # desc slider
    ax_info = fig.add_axes([0.68, 0.10, 0.28, 0.17])   # stats text

    ax_pr.set_xlabel('Recall',    fontsize=11)
    ax_pr.set_ylabel('Precision', fontsize=11)
    ax_pr.set_xlim(0, 1)
    ax_pr.set_ylim(0, 1.05)
    ax_pr.grid(True, alpha=0.3)

    [curve]  = ax_pr.plot([], [], lw=2, color='steelblue')
    [marker] = ax_pr.plot([], [], 'o', color='tomato', ms=9, zorder=5,
                          label='operating point')

    # ---- Stats box ----
    ax_info.axis('off')
    stats_text = ax_info.text(0.05, 0.95, '', va='top', ha='left',
                              fontsize=9, transform=ax_info.transAxes,
                              family='monospace')

    # ---- Slider ranges ----
    gt_max   = float(np.percentile(arr[:, 2], 99))
    desc_max = float(max(np.percentile(arr[:, 0], 99),
                         np.percentile(arr[:, 1], 99)))

    radio       = RadioButtons(ax_rad, FIELD_NAMES, active=0, activecolor='steelblue')
    gt_slider   = Slider(ax_gt,   'gt [m]',    0.1, gt_max,   valinit=state['gt_threshold'])
    desc_slider = Slider(ax_desc, 'desc thr',  0.0, desc_max, valinit=state['desc_threshold'])
    gt_slider.valtext.set_fontsize(8)
    desc_slider.valtext.set_fontsize(8)
    ax_rad.set_title('Field', fontsize=9, pad=4)

    # ---- Redraw ----
    def redraw():
        col = FIELD_COL[state['field']]
        thresholds, precision, recall = compute_pr(arr, col, state['gt_threshold'])

        curve.set_xdata(recall)
        curve.set_ydata(precision)

        p, r = find_op(thresholds, precision, recall, state['desc_threshold'])
        marker.set_xdata([r])
        marker.set_ydata([p])

        n_pos   = int((arr[:, 2] < state['gt_threshold']).sum())
        n_pairs = len(arr)
        stats_text.set_text(
            f"field       {state['field']}\n"
            f"gt_thresh   {state['gt_threshold']:.2f} m\n"
            f"desc_thresh {state['desc_threshold']:.4f}\n"
            f"precision   {p:.4f}\n"
            f"recall      {r:.4f}\n"
            f"positives   {n_pos} / {n_pairs}"
        )
        ax_pr.set_title(state['field'], fontsize=11)
        fig.canvas.draw_idle()

    # ---- Callbacks ----
    def on_radio(label):
        state['field'] = label
        redraw()

    def on_gt(val):
        state['gt_threshold'] = val
        redraw()

    def on_desc(val):
        state['desc_threshold'] = val
        redraw()

    def on_key(event):
        if event.key == 'q':
            plt.close('all')

    radio.on_clicked(on_radio)
    gt_slider.on_changed(on_gt)
    desc_slider.on_changed(on_desc)
    fig.canvas.mpl_connect('key_press_event', on_key)

    redraw()
    plt.show()


if __name__ == '__main__':
    main()
