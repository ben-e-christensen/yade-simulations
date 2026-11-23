"""
drum_calib.py - YADE rotating drum built for matching the real flow (one bead thick, 500 acrylic beads).

    yade -n -j1 -x drum_calib.py sweep/base_11rpm.json       # headless, one run (what the sweep uses)
    yade -j1 drum_calib.py sweep/base_11rpm.json gui         # with the 3D view, to watch it
    yade -n -j1 -x drum_calib.py test                        # ~1 s smoke test with the defaults

Every setting lives in a small json file (see DEFAULTS below; anything left out uses the default).
make_sweep.py writes a whole set of them.

What's different from main.py / ramping_friction.py:
  - NO global damping (NewtonIntegrator damping=0). main.py's damping=0.2 drags every bead in free
    flight and caps the surface speed. Energy is lost at contacts instead (coefficient of restitution).
  - Viscoelastic contacts (ViscElMat): restitution en / et, contact time tc, and ROLLING RESISTANCE mR.
  - Three separate friction coefficients: bead-bead, bead-faceplate, bead-drum wall. In a drum one
    bead thick every bead rubs the faceplates all the time, so mu_face matters a lot.
  - Real bead sizes: 3.175 mm +- 0.127 mm (uniform in that range), not one size.
  - Phases: settle (drum still) -> spin up -> wait for steady flow -> record. Only the recorded
    part is written, at the camera frame rate (200 fps; 100 fps for 1 rpm).
  - monitor.csv every 0.5 s: bed angle, mean speed, beads lost - to see when the flow is steady.

Output, in out_dir/<name>/:
    repose_data.csv   sim_time,bead_id,x_pos,y_pos,z_pos,radius   (metres, y up)  -> sim_to_tracks.py
    params.json       the full settings this run used
    monitor.csv       bed angle etc. over the whole run (settle + spin-up + record)
"""
import json, math, os, sys, time
import numpy as np
from yade import geom, ymport

DEFAULTS = dict(
    name="base",
    out_dir="calib_runs",
    rpm=11.0,
    # --- contacts ---
    mu_bb=0.40,        # bead-bead friction coefficient
    mu_face=0.40,      # bead-faceplate friction coefficient
    mu_drum=0.40,      # bead-drum wall (serrations) friction coefficient
    en_bb=0.85,        # coefficient of restitution, bead-bead (normal)
    en_wall=0.85,      # coefficient of restitution, bead-faceplate and bead-drum
    et_ratio=1.0,      # tangential restitution = et_ratio * en
    mR=0.0,            # rolling resistance coefficient (dimensionless, ~0 - 0.1)
    mRtype=1,          # 1: moment ~ normal force * mR * radius (Zhou 1999)
    tc=2e-4,           # contact duration (s): sets stiffness. Smaller = stiffer = slower sim
    dt_per_tc=20,      # time steps per contact
    damping=0.0,       # NewtonIntegrator global damping (main.py used 0.2)
    # --- beads ---
    n_beads=500,
    d_mean=3.175e-3,   # bead diameter (m)
    d_tol=0.127e-3,    # +- tolerance, uniform spread
    density=1190.0,
    seed=1,
    # --- geometry (same as main.py) ---
    stl="serrated_drum.stl",
    R_base=81.5e-3,
    drum_h=3.2e-3,
    gap=0.2e-3,
    # --- timing ---
    settle_s=1.0,      # drum still, beads fall into place
    spinup_s=1.0,      # drum speed ramps from 0 to rpm
    steady_revs=1.0,   # extra revolutions before recording starts (flow forgets the start)
    record_revs=3.0,   # revolutions recorded (11 rpm burst = 16.4 s = 3 revs)
    fps=None,          # recording rate; default 200 (100 at 1 rpm or below), like the camera
    gui=False,
)


# ----------------------------------------------------------------------------- settings
words = [a for a in sys.argv[1:] if not a.startswith("-")]
args = [a for a in words if a.endswith(".json")]
P = dict(DEFAULTS)
if args:
    with open(args[0]) as fh:
        user = json.load(fh)
    unknown = set(user) - set(DEFAULTS)
    if unknown:
        raise SystemExit(f"unknown settings in {args[0]}: {sorted(unknown)}")
    P.update(user)
TEST = "test" in words
if "gui" in words:
    P["gui"] = True
if TEST:
    P.update(name="test", settle_s=0.3, spinup_s=0.3, steady_revs=0.0, record_revs=0.4 * P["rpm"] / 60)
if P["fps"] is None:
    P["fps"] = 100.0 if P["rpm"] <= 1.0 else 200.0
OUT = os.path.join(P["out_dir"], P["name"])
os.makedirs(OUT, exist_ok=True)
with open(os.path.join(OUT, "params.json"), "w") as fh:
    json.dump(P, fh, indent=1)

omega = P["rpm"] * 2 * math.pi / 60
rev_s = 60.0 / P["rpm"]
T_SPIN = P["settle_s"]
T_STEADY = T_SPIN + P["spinup_s"]
T_REC = T_STEADY + P["steady_revs"] * rev_s
T_END = T_REC + P["record_revs"] * rev_s
print(f"\n--- {P['name']}: {P['rpm']:g} rpm, mu bb/face/drum {P['mu_bb']}/{P['mu_face']}/{P['mu_drum']}, "
      f"en {P['en_bb']}/{P['en_wall']}, mR {P['mR']}, damping {P['damping']}")
print(f"--- recording {T_REC:.1f} - {T_END:.1f} s sim time at {P['fps']:g} fps -> {OUT}\n")

# ----------------------------------------------------------------------------- materials
def mat(mu, en):
    return O.materials.append(ViscElMat(density=P["density"], frictionAngle=math.atan(mu), tc=P["tc"],
                                        en=en, et=en * P["et_ratio"], mR=P["mR"], mRtype=P["mRtype"]))

bead_mat = mat(P["mu_bb"], P["en_bb"])
drum_mat = mat(P["mu_drum"], P["en_wall"])
face_mat = mat(P["mu_face"], P["en_wall"])
# per-PAIR values (otherwise YADE takes the smaller friction of the two materials)
fric_mm = MatchMaker(matches=((bead_mat, bead_mat, math.atan(P["mu_bb"])),
                              (bead_mat, drum_mat, math.atan(P["mu_drum"])),
                              (bead_mat, face_mat, math.atan(P["mu_face"]))), algo="val")
en_mm = MatchMaker(matches=((bead_mat, bead_mat, P["en_bb"]), (bead_mat, drum_mat, P["en_wall"]),
                            (bead_mat, face_mat, P["en_wall"])), algo="val")
et_mm = MatchMaker(matches=((bead_mat, bead_mat, P["en_bb"] * P["et_ratio"]),
                            (bead_mat, drum_mat, P["en_wall"] * P["et_ratio"]),
                            (bead_mat, face_mat, P["en_wall"] * P["et_ratio"])), algo="val")

# ----------------------------------------------------------------------------- geometry
R_base, drum_h, gap = P["R_base"], P["drum_h"], P["gap"]
drum_ids = O.bodies.append(ymport.stl(P["stl"], material=drum_mat, scale=0.001))
face_ids = O.bodies.append(geom.facetBox(center=(0, 0, gap / 2), extents=(R_base * 1.5, R_base * 1.5, (drum_h + gap) / 2),
                                         material=face_mat, wallMask=48))
rotating_ids = drum_ids + face_ids

# ----------------------------------------------------------------------------- beads
rng = np.random.default_rng(P["seed"])
d_max = P["d_mean"] + P["d_tol"]
spacing = d_max * 1.02
row_h = spacing * math.sqrt(3) / 2
n_grid = int(2 * R_base / spacing) + 2
bead_ids = []
for row in range(-n_grid, n_grid):                 # bottom row first, so the beads fill from the bottom
    for col in range(-n_grid, n_grid):
        if len(bead_ids) >= P["n_beads"]:
            break
        x = col * spacing + (spacing / 2 if row % 2 else 0); y = row * row_h
        if math.hypot(x, y) < R_base - 0.003:
            r = 0.5 * rng.uniform(P["d_mean"] - P["d_tol"], P["d_mean"] + P["d_tol"])
            bead_ids.append(O.bodies.append(sphere((x, y, gap / 2), r, material=bead_mat)))
if len(bead_ids) < P["n_beads"]:
    raise SystemExit(f"only fit {len(bead_ids)} beads")
bead_ids = np.array(bead_ids)
print(f"placed {len(bead_ids)} beads, d = {P['d_mean']*1e3:.3f} +- {P['d_tol']*1e3:.3f} mm")

# ----------------------------------------------------------------------------- recording
pos_path = os.path.join(OUT, "repose_data.csv")
pos_fh = open(pos_path, "w")
pos_fh.write("sim_time,bead_id,x_pos,y_pos,z_pos,radius\n")
radii = np.array([O.bodies[i].shape.radius for i in bead_ids])
mon_fh = open(os.path.join(OUT, "monitor.csv"), "w")
mon_fh.write("sim_time,phase,drum_rpm,bed_angle_deg,mean_speed_mm_s,max_speed_mm_s,beads_lost,wall_s\n")
t_wall0 = time.time()


def state():
    xyz = np.array([O.bodies[i].state.pos for i in bead_ids])
    vel = np.array([O.bodies[i].state.vel for i in bead_ids])
    return xyz, vel


DONE = [False]


def record():
    if DONE[0] or O.time < T_REC - 1e-9:
        return
    xyz, _ = state()
    block = np.column_stack([np.full(len(bead_ids), O.time), bead_ids, xyz, radii])
    np.savetxt(pos_fh, block, fmt=["%.6f", "%d", "%.7e", "%.7e", "%.7e", "%.5e"], delimiter=",")


def phase_name():
    return "settle" if O.time < T_SPIN else "spinup" if O.time < T_STEADY else "steady" if O.time < T_REC else "record"


def control():
    """drum speed schedule + monitor + stop"""
    if O.time < T_SPIN:
        w = 0.0
    elif O.time < T_STEADY:
        w = omega * (O.time - T_SPIN) / max(P["spinup_s"], 1e-9)
    else:
        w = omega
    rot.angularVelocity = w
    if O.time >= T_END and not DONE[0]:
        finish()


def monitor():
    if DONE[0]:
        return
    xyz, vel = state()
    r = np.hypot(xyz[:, 0], xyz[:, 1])
    lost = int(((r > R_base + 0.005) | (xyz[:, 2] < -drum_h) | (xyz[:, 2] > drum_h + gap * 2)).sum())
    com = xyz[:, :2].mean(axis=0)
    bed = math.degrees(math.atan2(abs(com[0]), -com[1]))
    sp = np.linalg.norm(vel[:, :2], axis=1) * 1e3
    mon_fh.write(f"{O.time:.3f},{phase_name()},{rot.angularVelocity*60/(2*math.pi):.3f},{bed:.2f},"
                 f"{sp.mean():.2f},{sp.max():.1f},{lost},{time.time()-t_wall0:.1f}\n")
    mon_fh.flush()
    print(f"t {O.time:6.2f} s [{phase_name():6s}] bed angle {bed:5.1f} deg, mean speed {sp.mean():6.1f} mm/s, "
          f"lost {lost}, {time.time()-t_wall0:6.0f} s wall", flush=True)
    if lost > 0:
        print("!!! beads escaped the drum - contacts too soft or the time step too big. Stopping.")
        finish()


def finish():
    if DONE[0]:
        return
    DONE[0] = True
    if O.running:
        O.pause()
    pos_fh.close(); mon_fh.close()
    print(f"\n--- done: {pos_path} ({(time.time()-t_wall0)/60:.1f} min wall) ---")
    print(f"next (tracking venv): python sim_to_tracks.py {pos_path} --rpm {P['rpm']:g} --fps {P['fps']:g} --skip 0 "
          f"--bead-mm {P['d_mean']*1e3:.3f} --out sim_tracks/{P['name']}")


rot = RotationEngine(ids=rotating_ids, rotationAxis=(0, 0, 1), rotateAroundZero=True, angularVelocity=0.0)
O.engines = [
    ForceResetter(),
    InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Facet_Aabb()]),
    InteractionLoop(
        [Ig2_Sphere_Sphere_ScGeom(), Ig2_Facet_Sphere_ScGeom()],
        [Ip2_ViscElMat_ViscElMat_ViscElPhys(frictAngle=fric_mm, en=en_mm, et=et_mm)],
        [Law2_ScGeom_ViscElPhys_Basic()],
    ),
    NewtonIntegrator(gravity=(0, -9.81, 0), damping=P["damping"]),
    rot,
    PyRunner(command="control()", iterPeriod=50),
    PyRunner(command="record()", virtPeriod=1.0 / P["fps"]),
    PyRunner(command="monitor()", virtPeriod=0.5),
]
O.dt = P["tc"] / P["dt_per_tc"]
print(f"dt = {O.dt:.2e} s -> {T_END / O.dt / 1e6:.2f} M steps")

if P["gui"]:
    from yade import qt
    qt.View(); qt.center()
    O.run()
else:
    O.run(); O.wait()