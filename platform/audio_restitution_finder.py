"""
restitution_from_audio.py

Coefficient of restitution from the sound of a bead bouncing on a plate.

For a bead bouncing straight up and down, the time between impacts is
proportional to the rebound speed, so for consecutive intervals
    e = interval[k+1] / interval[k]
No drop height or release time is needed - only the impacts themselves, so
whatever happens in the recording before the first impact is ignored.

Usage:
    python restitution_from_audio.py path/to/recordings_folder
    python restitution_from_audio.py path/to/recordings_folder --out somewhere_else

Every .wav in the folder is analyzed.

Outputs (in --out, default <folder>/restitution_results/):
    summary.csv            one row per file + an ALL row
    bounces.csv            every interval/ratio used, per file
    <file>_bounces.svg     envelope with detected impacts + e vs impact speed
    all_files_e_vs_speed.svg

ALWAYS look at the per-file SVGs: the red markers should sit on the start of
each bounce and nothing else.

Needs: numpy, scipy, matplotlib
"""

import argparse
import csv
import glob
import os

import numpy as np
import scipy.io.wavfile as wavfile
import scipy.signal as sig
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

G = 9.81

# ---------------------------------------------------------------------
# Detection settings (defaults worked on teflon_on_aluminum.wav)
# ---------------------------------------------------------------------
HIGHPASS_HZ = 1000.0      # removes room rumble / handling noise; impacts are broadband clicks
ENV_LOWPASS_HZ = 2000.0   # smooths the envelope
PEAK_FACTOR = 5.0         # envelope peak must exceed this x the noise floor
CLUSTER_GAP_S = 0.025     # peaks closer than this belong to one impact (plate ring-down)
ONSET_FRACTION = 0.25     # impact time = first sample above this fraction of its peak
RATIO_MIN = 0.30          # allowed range for interval ratios while following a bounce train
RATIO_MAX = 0.95
START = 'loudest'         # 'loudest' = start at the loudest impact (normally the first drop),
                          # 'first'   = start at the first detected impact


def load_wav(path):
    sr, x = wavfile.read(path)
    x = x.astype(float)
    if x.ndim > 1:
        x = x.mean(axis=1)
    peak = np.abs(x).max()
    if peak > 0:
        x /= peak
    return sr, x


def detect_impacts(sr, x):
    """Returns (onset_times, peak_amplitudes, envelope, hp_signal)."""
    sos_hp = sig.butter(4, HIGHPASS_HZ, 'hp', fs=sr, output='sos')
    hp = sig.sosfiltfilt(sos_hp, x)
    env = np.abs(sig.hilbert(hp))
    sos_lp = sig.butter(2, ENV_LOWPASS_HZ, fs=sr, output='sos')
    env = sig.sosfiltfilt(sos_lp, env)
    noise = np.median(env)

    pk, _ = sig.find_peaks(env, height=noise * PEAK_FACTOR,
                           prominence=noise * (PEAK_FACTOR - 1),
                           distance=max(1, int(0.003 * sr)))

    # group ring-down peaks into single impacts
    clusters = []
    last_t = None
    for p in pk:
        t = p / sr
        if last_t is None or t - last_t > CLUSTER_GAP_S:
            clusters.append([p, p])          # [first peak index, loudest peak index]
        elif env[p] > env[clusters[-1][1]]:
            clusters[-1][1] = p
        last_t = t

    onsets, amps = [], []
    a = np.abs(hp)
    for first, loud in clusters:
        i0 = max(0, first - int(0.010 * sr))
        i1 = min(len(a), loud + int(0.005 * sr))
        seg = a[i0:i1]
        thr = ONSET_FRACTION * seg.max()
        onsets.append((i0 + int(np.argmax(seg > thr))) / sr)
        amps.append(env[loud])
    return np.array(onsets), np.array(amps), env, hp


def follow_bounce_train(onsets, amps):
    """Picks the bounce sequence: start impact, then each next impact must give
    an interval ratio inside [RATIO_MIN, RATIO_MAX]. Once a few ratios exist,
    the candidate closest to their median is chosen, so stray sounds are skipped."""
    if len(onsets) < 3:
        return []
    i0 = int(np.argmax(amps)) if START == 'loudest' else 0
    seq = [i0]
    if i0 + 1 >= len(onsets):
        return seq
    seq.append(i0 + 1)
    ratios = []
    while True:
        d_last = onsets[seq[-1]] - onsets[seq[-2]]
        cands = [j for j in range(seq[-1] + 1, len(onsets))
                 if RATIO_MIN * d_last <= onsets[j] - onsets[seq[-1]] <= RATIO_MAX * d_last]
        if not cands:
            break
        if ratios:
            ref = np.median(ratios)
            j = min(cands, key=lambda k: abs((onsets[k] - onsets[seq[-1]]) / d_last - ref))
        else:
            j = cands[0]
        ratios.append((onsets[j] - onsets[seq[-1]]) / d_last)
        seq.append(j)
    return seq


def analyze_file(path, out_dir):
    sr, x = load_wav(path)
    onsets, amps, env, hp = detect_impacts(sr, x)
    seq = follow_bounce_train(onsets, amps)
    name = os.path.splitext(os.path.basename(path))[0]

    t_seq = onsets[seq] if seq else np.array([])
    d = np.diff(t_seq)                      # flight times between impacts
    e = d[1:] / d[:-1] if len(d) > 1 else np.array([])
    # impact k (k>=1 in the train): in-speed = g*d[k-1]/2 ; e[k-1] = out/in at that impact
    v_in = G * d[:-1] / 2.0 if len(d) > 1 else np.array([])

    res = dict(file=name, n_impacts_detected=len(onsets), n_bounces_used=len(seq),
               n_ratios=len(e))
    if len(e):
        res.update(e_mean=float(e.mean()), e_median=float(np.median(e)),
                   e_std=float(e.std(ddof=1)) if len(e) > 1 else float('nan'),
                   first_rebound_height_mm=float(G * d[0] ** 2 / 8.0 * 1e3),
                   implied_drop_height_mm=float(G * d[0] ** 2 / 8.0 / e.mean() ** 2 * 1e3),
                   max_impact_speed_m_s=float(v_in.max()), min_impact_speed_m_s=float(v_in.min()))
    else:
        res.update(e_mean=float('nan'), e_median=float('nan'), e_std=float('nan'),
                   first_rebound_height_mm=float('nan'), implied_drop_height_mm=float('nan'),
                   max_impact_speed_m_s=float('nan'), min_impact_speed_m_s=float('nan'))

    rows = []
    for k in range(len(d)):
        rows.append(dict(file=name, impact_index=k + 1, impact_time_s=float(t_seq[k + 1]),
                         interval_before_s=float(d[k]),
                         ratio_e=float(e[k - 1]) if k >= 1 else float('nan'),
                         impact_speed_in_m_s=float(G * d[k - 1] / 2.0) if k >= 1 else float('nan')))

    # --- plot ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))
    t = np.arange(len(env)) / sr
    ax1.plot(t, env, color='0.4', lw=0.6, label='envelope (high-passed)')
    ax1.plot(onsets, np.interp(onsets, t, env), 'o', mfc='none', mec='0.6', ms=6, label='detected sounds')
    if seq:
        ax1.plot(t_seq, np.interp(t_seq, t, env), 'o', color='tab:red', ms=7, label='bounce train used')
        for k, tt in enumerate(t_seq):
            ax1.annotate(str(k), (tt, np.interp(tt, t, env)), textcoords='offset points',
                         xytext=(0, 6), ha='center', fontsize=8, color='tab:red')
    ax1.set_xlabel('time (s)'); ax1.set_ylabel('envelope'); ax1.legend(loc='upper right', fontsize=8)
    ax1.set_title(f'{name}: e = {res["e_mean"]:.3f} +/- {res["e_std"]:.3f} ({len(e)} ratios)')
    if len(e):
        ax2.plot(v_in, e, 'o-', color='tab:red')
        ax2.axhline(e.mean(), color='0.5', ls='--', lw=1)
    ax2.set_xlabel('impact speed (m/s)'); ax2.set_ylabel('e'); ax2.set_ylim(0, 1)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f'{name}_bounces.svg'))
    plt.close(fig)

    return res, rows, v_in, e


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folder', help='folder containing the .wav recordings')
    ap.add_argument('--out', default=None,
                    help='output folder (default: <folder>/restitution_results)')
    args = ap.parse_args()

    if not os.path.isdir(args.folder):
        raise SystemExit(f'Not a folder: {args.folder}')
    files = sorted(f for f in glob.glob(os.path.join(args.folder, '*'))
                   if f.lower().endswith('.wav'))
    if not files:
        raise SystemExit(f'No .wav files found in {args.folder}')
    if args.out is None:
        args.out = os.path.join(args.folder, 'restitution_results')
    os.makedirs(args.out, exist_ok=True)
    print(f"Found {len(files)} .wav files in {os.path.abspath(args.folder)}\n")

    summaries, all_rows, all_e = [], [], []
    fig, ax = plt.subplots(figsize=(8, 5))
    for f in files:
        res, rows, v_in, e = analyze_file(f, args.out)
        summaries.append(res)
        all_rows += rows
        all_e += list(e)
        if len(e):
            ax.plot(v_in, e, 'o-', ms=4, lw=0.8, label=res['file'])
        print(f"{res['file']:30s} bounces {res['n_bounces_used']:2d}  "
              f"e = {res['e_mean']:.3f} +/- {res['e_std']:.3f}  "
              f"(median {res['e_median']:.3f})  implied drop {res['implied_drop_height_mm']:.0f} mm")

    all_e = np.array(all_e)
    if len(all_e):
        overall = dict(file='ALL', n_impacts_detected='', n_bounces_used='', n_ratios=len(all_e),
                       e_mean=float(all_e.mean()), e_median=float(np.median(all_e)),
                       e_std=float(all_e.std(ddof=1)) if len(all_e) > 1 else float('nan'),
                       first_rebound_height_mm='', implied_drop_height_mm='',
                       max_impact_speed_m_s='', min_impact_speed_m_s='')
        summaries.append(overall)
        ax.axhline(all_e.mean(), color='k', ls='--', lw=1, label=f'mean {all_e.mean():.3f}')
        print(f"\nALL FILES: e = {all_e.mean():.3f} +/- {overall['e_std']:.3f} "
              f"(median {overall['e_median']:.3f}, {len(all_e)} ratios, "
              f"std error {overall['e_std']/np.sqrt(len(all_e)):.3f})")
    ax.set_xlabel('impact speed (m/s)'); ax.set_ylabel('e'); ax.set_ylim(0, 1)
    ax.legend(fontsize=7); ax.set_title('Restitution vs impact speed, all files')
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'all_files_e_vs_speed.svg'))
    plt.close(fig)

    with open(os.path.join(args.out, 'summary.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(summaries[0].keys()))
        w.writeheader(); w.writerows(summaries)
    if all_rows:
        with open(os.path.join(args.out, 'bounces.csv'), 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=list(all_rows[0].keys()))
            w.writeheader(); w.writerows(all_rows)
    print(f"\nResults in {os.path.abspath(args.out)}")


if __name__ == '__main__':
    main()

# Notes
# - Air drag shortens flight times slightly (~1-2% for a 3 mm bead at ~2 m/s), so e from
#   timing reads a touch low at the highest speeds. Negligible next to drop-to-drop scatter.
# - The train stops once bounces get closer than CLUSTER_GAP_S (they merge with the plate's
#   ring-down). Lower it if your plate rings less; raise it if one impact shows up as two.
# - The bead should bounce in place. If it hops sideways or spins hard, the timing still
#   gives the vertical restitution, but drops that wander off the plate end the train early.