"""
animate_bead.py - visualize a bead rolling on the platform, from a
stewart_slip_test.py output folder.

Left panel : whole plate, top-down in the PLATE's frame (the plate looks still,
             the bead moves on it). Trail + arrow showing the current downhill
             direction (length scales with tilt).
Right panel: zoomed view that follows the bead. Two body-fixed marks:
             red = "north pole", blue = "south pole". Filled = that pole is on the
             upper half (facing you), faded = underneath. At spawn north points up.

Reads <folder>/bead_positions.csv (the VTKs don't contain bead orientation).

Usage:
    python animate_bead.py output_20261023_154210            (every step)

Optional:
    --step N   only that sweep step (see summary.csv)
    --full     whole step, ramp included (default: first 10 s after the ramp)
    --t0/t1    custom window in seconds of motion time within each step
    --fps      playback frame rate. Data is 100 Hz and every frame is shown,
               so fps 25 = 4x slow motion, fps 100 = real time.
    --every N  only draw every Nth data frame (faster render, choppier spin).

Output: <folder>/animations/bead_step<N>_<fric>deg_<t0>-<t1>s.mp4
        (.gif if ffmpeg isn't installed)

Needs: numpy, pandas, scipy, matplotlib (+ ffmpeg for mp4)
"""

import argparse
import os

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Circle, FancyArrow

R_BEAD = 1.6e-3
RING_INNER_R = 100.6e-3
TRAIL_S = 3.0          # seconds of trail shown on the plate view
ZOOM_HALF = 3.0        # zoom panel half-width, in bead radii


def quats_from_columns(c, order):
    """c: (N,4) array of the logged rot_w,rot_x,rot_y,rot_z columns (really ori[0..3]).
    order 'wxyz' -> ori[0] is w; order 'xyzw' -> ori[3] is w. Returns scipy Rotation."""
    if order == 'wxyz':
        return Rot.from_quat(np.column_stack([c[:, 1], c[:, 2], c[:, 3], c[:, 0]]))
    return Rot.from_quat(c)          # scipy uses (x, y, z, w)


def detect_quat_order(df):
    """Pick the component order whose frame-to-frame rotation matches the logged angVel."""
    c = df[['rot_w', 'rot_x', 'rot_y', 'rot_z']].to_numpy()
    w = df[['ang_vel_x', 'ang_vel_y', 'ang_vel_z']].to_numpy()
    t = df['sim_time'].to_numpy()
    dt = np.diff(t)
    w_mid = 0.5 * (w[1:] + w[:-1])
    ok = (dt > 0) & (dt < 0.05) & (np.linalg.norm(w_mid, axis=1) * dt < 2.0) & \
         (np.linalg.norm(w_mid, axis=1) > 1.0)
    scores = {}
    for order in ('wxyz', 'xyzw'):
        R = quats_from_columns(c, order)
        dR = R[1:] * R[:-1].inv()              # world-frame increment between frames
        w_est = dR.as_rotvec() / dt[:, None]
        err = np.linalg.norm(w_est[ok] - w_mid[ok], axis=1) / np.linalg.norm(w_mid[ok], axis=1)
        scores[order] = np.median(err) if ok.any() else np.inf
    best = min(scores, key=scores.get)
    return best, scores


def plate_rotations(df):
    """Plate rotation from the logged tilt rotation vector (deg)."""
    rv = np.radians(np.column_stack([df['tilt_rx_deg'], df['tilt_ry_deg'], np.zeros(len(df))]))
    return Rot.from_rotvec(rv)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folder', help='stewart_slip_test.py output folder')
    ap.add_argument('--step', type=int, default=None, help='only this step (default: all steps)')
    ap.add_argument('--full', action='store_true', help='whole step including the ramp')
    ap.add_argument('--t0', type=float, default=None)
    ap.add_argument('--t1', type=float, default=None)
    ap.add_argument('--fps', type=float, default=25.0)
    ap.add_argument('--every', type=int, default=1)
    args = ap.parse_args()

    df_all = pd.read_csv(os.path.join(args.folder, 'bead_positions.csv'))
    steps = sorted(df_all['step'].unique())
    if args.step is not None:
        if args.step not in steps:
            raise SystemExit(f"No rows for step {args.step}. Steps present: {steps}")
        steps = [args.step]
    out_dir = os.path.join(args.folder, 'animations')
    os.makedirs(out_dir, exist_ok=True)
    print(f"Rendering {len(steps)} step(s) into {out_dir}\n")
    for s in steps:
        render_step(df_all[df_all['step'] == s].copy(), s, args, out_dir)


def render_step(df, step, args, out_dir):
    bead = df['bead_id'].iloc[0]
    df = df[df['bead_id'] == bead].sort_values('sim_time').reset_index(drop=True)

    order, scores = detect_quat_order(df)
    print(f"quaternion order: {order}  (median spin mismatch: "
          f"wxyz {scores['wxyz']:.3f}, xyzw {scores['xyzw']:.3f})")
    if scores[order] > 0.2:
        print("WARNING: neither order matches the logged spin well - pole marks may be wrong")

    t_mot = df['t_motion'].to_numpy()
    ramp_done = df.index[df['ramp'] >= 1.0]
    t_ramp_end = t_mot[ramp_done[0]] if len(ramp_done) else t_mot[0]
    t0 = args.t0 if args.t0 is not None else t_ramp_end
    t1 = args.t1 if args.t1 is not None else t0 + 10.0
    sel = (t_mot >= t0) & (t_mot <= t1)
    if sel.sum() < 2:
        raise SystemExit(f"No data in t_motion window {t0}-{t1} s (step runs {t_mot[0]:.1f}-{t_mot[-1]:.1f} s)")

    # --- geometry in the plate frame ---
    R_plate = plate_rotations(df)
    R_bead = quats_from_columns(df[['rot_w', 'rot_x', 'rot_y', 'rot_z']].to_numpy(), order)
    north_world = R_bead.apply([0.0, 0.0, 1.0])               # body z-axis in world frame
    north_plate = R_plate.inv().apply(north_world)            # ...expressed in the plate frame
    down_plate = R_plate.inv().apply([0.0, 0.0, -1.0])        # gravity in the plate frame
    px, py = df['px'].to_numpy(), df['py'].to_numpy()
    t_sim = df['sim_time'].to_numpy()
    vx, vy = np.gradient(px, t_sim), np.gradient(py, t_sim)
    speed = np.hypot(vx, vy)
    fric = df['alu_fric_deg'].iloc[0]

    idx = np.where(sel)[0][::max(1, args.every)]

    # --- figure ---
    fig, (axP, axZ) = plt.subplots(1, 2, figsize=(12, 6))
    lim = RING_INNER_R * 1.08 * 1e3
    axP.set_xlim(-lim, lim); axP.set_ylim(-lim, lim); axP.set_aspect('equal')
    axP.add_patch(Circle((0, 0), RING_INNER_R * 1e3, fill=False, lw=1.5, color='0.3'))
    axP.set_xlabel('x on plate (mm)'); axP.set_ylabel('y on plate (mm)')
    trail, = axP.plot([], [], '-', color='tab:blue', lw=1, alpha=0.6)
    bead_dot, = axP.plot([], [], 'o', color='tab:blue', ms=6)
    arrow_holder = [None]
    title = axP.set_title('')

    zr = ZOOM_HALF
    axZ.set_xlim(-zr, zr); axZ.set_ylim(-zr, zr); axZ.set_aspect('equal')
    axZ.set_xticks([]); axZ.set_yticks([])
    axZ.add_patch(Circle((0, 0), 1.0, facecolor='0.92', edgecolor='0.2', lw=1.5))
    north_mk, = axZ.plot([], [], 'o', ms=14, mec='tab:red', mew=2)
    south_mk, = axZ.plot([], [], 'o', ms=14, mec='tab:blue', mew=2)
    vel_line, = axZ.plot([], [], '-', color='0.5', lw=1)
    info = axZ.text(-zr * 0.95, -zr * 0.95, '', fontsize=9, va='bottom', family='monospace')
    axZ.set_title('bead close-up (follows bead, plate frame)\nred = north pole, blue = south; filled = on top')

    def draw(k):
        i = idx[k]
        # plate view
        j0 = np.searchsorted(t_sim, t_sim[i] - TRAIL_S)
        trail.set_data(px[j0:i + 1] * 1e3, py[j0:i + 1] * 1e3)
        bead_dot.set_data([px[i] * 1e3], [py[i] * 1e3])
        if arrow_holder[0] is not None:
            arrow_holder[0].remove()
        g = down_plate[i]
        tilt = np.degrees(np.arcsin(min(1.0, np.hypot(g[0], g[1]))))
        gh = np.hypot(g[0], g[1])
        if gh > 1e-6:
            L = 60.0 * tilt / 10.0
            arrow_holder[0] = axP.add_patch(FancyArrow(0, 0, g[0] / gh * L, g[1] / gh * L,
                                                       width=1.5, color='tab:orange', alpha=0.8))
        else:
            arrow_holder[0] = None
        title.set_text(f'step {args.step}  alu friction {fric:g} deg   t = {t_mot[i]:6.2f} s   '
                       f'tilt {tilt:4.1f} deg (orange = downhill)')

        # close-up: pole positions projected onto the plate, in bead radii
        n = north_plate[i]
        north_mk.set_data([n[0]], [n[1]])
        south_mk.set_data([-n[0]], [-n[1]])
        north_mk.set_markerfacecolor('tab:red' if n[2] >= 0 else 'none')
        north_mk.set_alpha(1.0 if n[2] >= 0 else 0.35)
        south_mk.set_markerfacecolor('tab:blue' if n[2] < 0 else 'none')
        south_mk.set_alpha(1.0 if n[2] < 0 else 0.35)
        north_mk.set_zorder(5 if n[2] >= 0 else 3)
        south_mk.set_zorder(5 if n[2] < 0 else 3)
        s = speed[i]
        if s > 1e-6:
            vel_line.set_data([0, vx[i] / s * zr * 0.9], [0, vy[i] / s * zr * 0.9])
        else:
            vel_line.set_data([], [])
        w = df[['ang_vel_x', 'ang_vel_y', 'ang_vel_z']].iloc[i].to_numpy()
        info.set_text(f'speed  {s * 1e3:7.1f} mm/s\n|w|R   {np.linalg.norm(w) * R_BEAD * 1e3:7.1f} mm/s\n'
                      f'grey line = direction of travel')
        return trail, bead_dot, north_mk, south_mk, vel_line, info, title

    anim = animation.FuncAnimation(fig, draw, frames=len(idx), interval=1000.0 / args.fps, blit=False)
    base = os.path.join(args.folder, f'bead_step{args.step}_{t0:.0f}-{t1:.0f}s')
    if animation.writers.is_available('ffmpeg'):
        out = base + '.mp4'
        anim.save(out, writer=animation.FFMpegWriter(fps=args.fps, bitrate=3000))
    else:
        out = base + '.gif'
        print("ffmpeg not found - writing a GIF instead (bigger, slower)")
        anim.save(out, writer=animation.PillowWriter(fps=args.fps))
    plt.close(fig)
    print(f"{len(idx)} frames, {len(idx) / args.fps:.1f} s of video "
          f"({100.0 / args.every / args.fps:.1f}x slow motion) -> {out}")


if __name__ == '__main__':
    main()