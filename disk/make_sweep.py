"""
make_sweep.py - write the json settings files for a one-at-a-time sensitivity sweep of drum_calib.py.

    python3 make_sweep.py                       # default: base + each knob varied alone, at 11 and 26 rpm
    python3 make_sweep.py --rpm 11 --only mu_face mR
    bash run_sweep.sh 6                         # then run them, 6 at a time

Each knob is varied on its own with everything else at BASE, so the result shows which knob moves
which part of the flow. Edit BASE / KNOBS below when you narrow things down.
"""
import argparse, json, os

BASE = dict(mu_bb=0.40, mu_face=0.40, mu_drum=0.40, en_bb=0.85, en_wall=0.85, mR=0.0, damping=0.0)
KNOBS = {
    "mu_bb":   [0.2, 0.3, 0.5, 0.6],
    "mu_face": [0.1, 0.2, 0.3, 0.5, 0.6],
    "mu_drum": [0.2, 0.6, 0.9],
    "en_bb":   [0.5, 0.7, 0.95],
    "en_wall": [0.5, 0.7, 0.95],
    "mR":      [0.01, 0.03, 0.06, 0.1],
    "damping": [0.2],                 # main.py's setting, for reference
}

ap = argparse.ArgumentParser()
ap.add_argument("--rpm", type=float, nargs="+", default=[11, 26])
ap.add_argument("--only", nargs="*", default=None, help="vary only these knobs")
ap.add_argument("--out", default="sweep")
ap.add_argument("--runs", default="calib_runs", help="where drum_calib.py writes the results")
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)
n = 0
for rpm in a.rpm:
    jobs = [("base", None, None)] + [(k, k, v) for k in KNOBS if a.only is None or k in a.only for v in KNOBS[k]]
    for tag, k, v in jobs:
        p = dict(BASE, rpm=rpm, out_dir=a.runs)
        if k is not None:
            p[k] = v
        p["name"] = f"{rpm:g}rpm_{tag}" + ("" if k is None else f"{v:g}")
        with open(os.path.join(a.out, p["name"] + ".json"), "w") as fh:
            json.dump(p, fh, indent=1)
        n += 1
print(f"wrote {n} settings files to {a.out}/ - run them with:  bash run_sweep.sh 6")