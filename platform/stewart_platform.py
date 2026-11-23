# stewart_slip_test.py  (v2: Hertz-Mindlin + restitution, PTFE beads)
#
# Test 1: single bead (or N beads) on the Stewart platform plate.
# Finds the bead-aluminum sliding-friction angle above which a rolling bead
# no longer skids on the floor.
#
# Physics expectation (uniform solid sphere, I = 2/5 m R^2):
#   no-skid rolling needs  tan(friction_angle) >= (2/7) * tan(peak_tilt)
#   -> ~2.88 deg friction angle at 10 deg peak tilt.
#
# Motion: the platform floor + ring are kinematic facets rotated about the
# pivot (origin, floor at pivot). Gravity stays world -z. The plate's tilt is
# defined as a rotation vector (axis * angle), same construction as the
# firmware's W orbit (Rodrigues about a horizontal axis).
#
# Contact law (v2): Hertz-Mindlin with viscous damping set from a measured
# coefficient of restitution (RESTITUTION). v1 used undamped Cundall-Strack
# with zero global damping, and the bead bounced indefinitely (mm-scale hops,
# floor contact only a few % of the time). No rolling resistance yet.
#
# Friction is set PER MATERIAL PAIR (bead-bead, bead-aluminum) with a
# MatchMaker, so aluminum friction can be higher than bead friction. Every
# step verifies that the first floor contact actually got the intended value.
#
# Sweep mode: each friction step deletes and re-spawns the bead(s) at the
# plate center at rest, levels the platform, and ramps the tilt back in over
# RAMP_CYCLES (prevents the bead from picking up a permanent drift velocity).
# Summary stats skip the ramp only; the CSVs record everything.
#
# Forever mode (SWEEP = False): fixed friction, one ramp-in, runs until you
# stop it, printing a stats window every STEP_DURATION_S.
#
# UNTESTED - see "things to check on first run" at the bottom of this file.

from yade import geom, utils
import math
import os

# =====================================================================
# USER PARAMETERS
# =====================================================================
N_BEADS = 1                    # beads, placed as a hex patch centered on the plate
R_BEAD = 1.6e-3                # 1/8 in diameter
BEAD_DENSITY = 2131.0          # PTFE, McMaster: 0.077 lb/in^3
BEAD_FRICTION_MU = 0.065       # PTFE bead-bead (McMaster lists 0.05-0.08); used when N_BEADS > 1
RESTITUTION = 0.66             # measured: PTFE bead on aluminum plate, 5 audio drops

# Plate geometry (matches the OpenSCAD: ring ID = 2*(4in - 1mm), wall 0.5 in)
RING_INNER_R = 4 * 25.4e-3 - 1.0e-3                # 100.6 mm
WALL_H = 0.5 * 25.4e-3                             # 12.7 mm
RING_SEGMENTS = 300
ALU_DENSITY = 2700.0

# Platform motion
PATTERN = 'figure8'            # 'figure8', 'rock', or 'orbit' (orbit = firmware W)
TILT_DEG = 10.0                # PEAK tilt magnitude of the plate, deg
PERIOD_S = 1.4                 # seconds per full pattern cycle (one complete figure-8).
                               # Bead swing grows with PERIOD_S^2 - keep <= ~1.8 s at 10 deg
                               # so the bead stays off the wall (script prints an estimate).
RAMP_CYCLES = 5                # tilt fades in from 0 to full over this many cycles at the
                               # start of each step (avoids kicking the bead into a drift)

# Friction sweep (bead-aluminum)
SWEEP = True
ALU_FRICTION_SCHEDULE_DEG = [i * 0.25 for i in range(21)] + [6.0, 8.0, 10.0]  # 0..5 by 0.25, then 6, 8, 10
STEP_DURATION_S = 30.0         # sim seconds per friction step (also stats window in forever mode)
ALU_FRICTION_FIXED_DEG = 3.0   # used only when SWEEP = False

# Contact / integration
YOUNG = 1e7                    # softened like the drum sims (real PTFE ~0.5 GPa, Al ~70 GPa)
POISSON = 0.3
DT_SAFETY = 0.5                # O.dt = DT_SAFETY * PWaveTimeStep()
GLOBAL_DAMPING = 0.0           # keep 0: all dissipation comes from RESTITUTION at contacts

# Platform kinematics
KIN_UPDATE_EVERY = 10          # iterations between platform velocity updates
RESYNC_EVERY_S = 0.5           # snap facet positions to the exact rigid pose this often

# Output
OUT_DIR = 'stewart_output'
POS_LOG_HZ = 100
CONTACT_LOG_HZ = 100
RECORD_VTK = True
VTK_HZ = 100
SLIDE_RATIO = 0.999            # |Fs| >= this * mu*|Fn|  ->  contact counted as sliding

# =====================================================================
# Sanity checks / estimates
# =====================================================================
if PATTERN not in ('figure8', 'rock', 'orbit'):
    raise ValueError("PATTERN must be 'figure8', 'rock', or 'orbit'")
if not (0.0 < RESTITUTION < 1.0):
    raise ValueError("RESTITUTION must be between 0 and 1 (exclusive)")
if STEP_DURATION_S <= RAMP_CYCLES * PERIOD_S:
    raise ValueError("STEP_DURATION_S must be longer than the ramp (RAMP_CYCLES * PERIOD_S)")

_fric_values = ALU_FRICTION_SCHEDULE_DEG if SWEEP else [ALU_FRICTION_FIXED_DEG]
BEAD_FRICTION_DEG = math.degrees(math.atan(BEAD_FRICTION_MU))
PREDICTED_DEG = math.degrees(math.atan((2.0 / 7.0) * math.tan(math.radians(TILT_DEG))))

# Rough estimate of how far a rolling bead swings from center once at full tilt
_w = 2.0 * math.pi / PERIOD_S
_A = math.radians(TILT_DEG)
if PATTERN == 'figure8':
    _sx = (5.0 / 7.0) * 9.81 * math.sin(_A / 1.25) / _w ** 2          # 1x component
    _sy = (5.0 / 7.0) * 9.81 * math.sin(_A / 1.25) / (2 * _w) ** 2    # 2x component
    EST_SWING = math.hypot(_sx, _sy)
else:
    EST_SWING = (5.0 / 7.0) * 9.81 * math.sin(_A) / _w ** 2
EST_WALL_ROOM = RING_INNER_R - R_BEAD

os.makedirs(OUT_DIR, exist_ok=True)
if RECORD_VTK:
    os.makedirs(os.path.join(OUT_DIR, 'vtk'), exist_ok=True)

# =====================================================================
# Materials + contact physics
# =====================================================================
bead_mat = O.materials.append(FrictMat(young=YOUNG, poisson=POISSON, density=BEAD_DENSITY,
                                       frictionAngle=math.radians(BEAD_FRICTION_DEG)))
alu_mat = O.materials.append(FrictMat(young=YOUNG, poisson=POISSON, density=ALU_DENSITY,
                                      frictionAngle=math.radians(_fric_values[0])))

ip2 = Ip2_FrictMat_FrictMat_MindlinPhys()
_ip2_attrs = set(ip2.dict().keys())

# Restitution: en/es MatchMakers (Tsuji-style damping) if available, else betan/betas.
if 'en' in _ip2_attrs and 'es' in _ip2_attrs:
    ip2.en = MatchMaker(algo='val', val=RESTITUTION)
    ip2.es = MatchMaker(algo='val', val=RESTITUTION)
    DAMPING_MODE = 'en/es MatchMaker'
else:
    _lnE = math.log(RESTITUTION)
    _beta = -_lnE / math.sqrt(_lnE ** 2 + math.pi ** 2)
    ip2.betan = _beta
    ip2.betas = _beta
    DAMPING_MODE = f'betan/betas = {_beta:.3f} (linear-equivalent)'

# Per-pair friction. Checked via .dict() because setting a non-existent
# attribute on a YADE object can silently do nothing.
FRICTION_PER_PAIR = 'frictAngle' in _ip2_attrs
if not FRICTION_PER_PAIR:
    # Fallback: YADE uses min(mat1, mat2). With one bead, bead-bead never happens,
    # so raise the bead material's friction and let aluminum win the min().
    if N_BEADS > 1 and max(_fric_values) > BEAD_FRICTION_DEG:
        raise RuntimeError("This YADE build has no per-pair frictAngle on Ip2_FrictMat_FrictMat_MindlinPhys, "
                           "so aluminum friction can't exceed bead friction with N_BEADS > 1.")
    if N_BEADS == 1:
        O.materials[bead_mat].frictionAngle = math.radians(89.0)

def set_alu_friction(alu_deg):
    O.materials[alu_mat].frictionAngle = math.radians(alu_deg)
    if FRICTION_PER_PAIR:
        b, a = math.radians(BEAD_FRICTION_DEG), math.radians(alu_deg)
        ip2.frictAngle = MatchMaker(matches=((bead_mat, bead_mat, b),
                                             (bead_mat, alu_mat, a),
                                             (alu_mat, alu_mat, a)))

# =====================================================================
# Plate geometry
# =====================================================================
# Floor = ONE big equilateral triangle (inradius 150 mm) so there are no facet
# edges or shared vertices anywhere the beads can reach.
_Rc = 0.30   # circumradius of floor triangle
floor_verts = [Vector3(_Rc * math.cos(math.radians(a)), _Rc * math.sin(math.radians(a)), 0.0)
               for a in (90.0, 210.0, 330.0)]
floor_id = O.bodies.append(utils.facet(floor_verts, material=alu_mat, color=(0.6, 0.6, 0.6)))

# Ring wall: side only (wallMask=4), from z=0 to WALL_H
wall_facets = geom.facetCylinder(center=(0, 0, WALL_H / 2.0), radius=RING_INNER_R, height=WALL_H,
                                 segmentsNumber=RING_SEGMENTS, wallMask=4, material=alu_mat,
                                 color=(0.7, 0.7, 0.8))
wall_ids = O.bodies.append(wall_facets)
wall_id_set = set(wall_ids)
plat_ids = [floor_id] + list(wall_ids)

# Store each platform facet's level-pose state (copies, not references)
p0 = {bid: O.bodies[bid].state.pos * 1.0 for bid in plat_ids}
q0 = {bid: O.bodies[bid].state.ori * Quaternion.Identity for bid in plat_ids}
q_ref0 = q0[floor_id]

# =====================================================================
# Platform motion definition
# =====================================================================
def quat_from_rotvec(rv):
    ang = rv.norm()
    if ang < 1e-12:
        return Quaternion.Identity
    return Quaternion(rv / ang, ang)     # (axis, angle)

def ramp_factor(t):
    T_r = RAMP_CYCLES * PERIOD_S
    if T_r <= 0 or t >= T_r:
        return 1.0
    if t <= 0:
        return 0.0
    return 0.5 * (1.0 - math.cos(math.pi * t / T_r))   # smooth 0 -> 1

def tilt_rotvec(t):
    """Plate rotation vector (rad) at motion time t. Magnitude = tilt angle."""
    A = math.radians(TILT_DEG) * ramp_factor(t)
    ph = 2.0 * math.pi * t / PERIOD_S
    if PATTERN == 'orbit':
        # identical to firmware W: axis (sin ph, -cos ph, 0); low point starts at -x, goes CCW
        v = Vector3(math.sin(ph), -math.cos(ph), 0.0)
    elif PATTERN == 'rock':
        # tilt about y only: bead rolls back and forth along x
        v = Vector3(0.0, math.sin(ph), 0.0)
    else:
        # figure-8: about y at 1x (bead moves along x), about x at 2x (bead moves along y).
        # Divided by 1.25 so the PEAK tilt magnitude equals TILT_DEG.
        v = Vector3(math.sin(2.0 * ph), math.sin(ph), 0.0) / 1.25
    return v * A

def platform_rotation():
    """Current rigid rotation of the platform relative to its level pose."""
    return O.bodies[floor_id].state.ori * q_ref0.conjugate()

def quat_angle(q):
    _, ang = q.toAxisAngle()
    if ang > math.pi:
        ang = 2.0 * math.pi - ang
    return abs(ang)

# =====================================================================
# Beads
# =====================================================================
def hex_patch(n):
    spacing = 2.0 * R_BEAD * 1.01
    row_h = spacing * math.sqrt(3) / 2.0
    M = int(math.sqrt(n)) + 4
    pts = []
    for row in range(-M, M + 1):
        for col in range(-M, M + 1):
            x = col * spacing + (spacing / 2.0 if row % 2 else 0.0)
            y = row * row_h
            pts.append((x, y))
    pts.sort(key=lambda p: p[0] ** 2 + p[1] ** 2)
    pts = pts[:n]
    r_far = max(math.hypot(x, y) for x, y in pts)
    if r_far + R_BEAD > RING_INNER_R - 2e-3:
        raise ValueError(f"{n} beads don't fit as a centered monolayer patch")
    return pts

bead_start = hex_patch(N_BEADS)
bead_ids = []

def spawn_beads():
    global bead_ids
    z0 = R_BEAD                  # just touching the floor, zero overlap (no drop)
    bead_ids = []
    for x, y in bead_start:
        bead_ids.append(O.bodies.append(
            utils.sphere((x, y, z0), R_BEAD, material=bead_mat, color=(0.1, 0.3, 1.0))))

def erase_beads():
    global bead_ids
    for bid in bead_ids:
        O.bodies.erase(bid)
    bead_ids = []

def level_platform():
    for bid in plat_ids:
        s = O.bodies[bid].state
        s.pos = p0[bid]
        s.ori = q0[bid]
        s.vel = Vector3.Zero
        s.angVel = Vector3.Zero
    rot_eng.angularVelocity = 0.0

# =====================================================================
# State
# =====================================================================
step_idx = 0
motion_t0 = 0.0      # motion clock start (reset every sweep step)
window_t0 = 0.0      # stats window start
current_fric_deg = _fric_values[0]
stats = {}
max_track_err_deg = 0.0
friction_checked = False
friction_check_result = 'no_floor_contact'

def reset_stats():
    global stats, max_track_err_deg, friction_checked, friction_check_result
    stats = dict(n=0, floor_n=0, floor_slide=0, floor_slip_sum=0.0, floor_slip_max=0.0,
                 wall_n=0, wall_slide=0, r_max=0.0, h_n=0, h_sum=0.0, h_max=-1.0)
    max_track_err_deg = 0.0
    friction_checked = False
    friction_check_result = 'no_floor_contact'

def stats_active():
    t_motion = O.time - motion_t0
    return t_motion >= RAMP_CYCLES * PERIOD_S

# =====================================================================
# Output files (kept open; flushed at every resync and step end)
# =====================================================================
pos_f = open(os.path.join(OUT_DIR, 'bead_positions.csv'), 'w')
pos_f.write("sim_time,step,alu_fric_deg,t_motion,phase_frac,ramp,tilt_rx_deg,tilt_ry_deg,"
            "bead_id,x,y,z,px,py,pz,rot_w,rot_x,rot_y,rot_z,ang_vel_x,ang_vel_y,ang_vel_z\n")

con_f = open(os.path.join(OUT_DIR, 'contacts.csv'), 'w')
con_f.write("sim_time,step,alu_fric_deg,t_motion,id1,id2,contact,fn,fs,fs_over_cap,sliding,slip_speed\n")

sum_f = open(os.path.join(OUT_DIR, 'summary.csv'), 'w')
sum_f.write("step,alu_fric_deg,predicted_no_skid_deg,samples,floor_contact_frac,"
            "floor_slide_frac_of_contact,floor_slip_mean,floor_slip_max,wall_contact_frac,"
            "wall_slide_frac,r_max_mm,min_wall_gap_mm,h_mean_um,h_max_um,"
            "max_platform_track_err_deg,friction_check\n")

def flush_all():
    pos_f.flush(); con_f.flush(); sum_f.flush()

# =====================================================================
# Engine callbacks
# =====================================================================
def platform_update():
    """Set the RotationEngine's omega so the platform reaches the target pose
    KIN_UPDATE_EVERY steps from now. Tracks the ACTUAL pose, so integration
    error can't accumulate."""
    dT = KIN_UPDATE_EVERY * O.dt
    # Newton has already advanced bodies to O.time + O.dt this iteration
    t_target = (O.time + O.dt + dT) - motion_t0
    q_target = quat_from_rotvec(tilt_rotvec(t_target))
    dq = q_target * platform_rotation().conjugate()
    axis, ang = dq.toAxisAngle()
    if ang > math.pi:
        ang -= 2.0 * math.pi
    if abs(ang) < 1e-14:
        rot_eng.angularVelocity = 0.0
    else:
        rot_eng.rotationAxis = axis.normalized()
        rot_eng.angularVelocity = ang / dT

def resync_platform():
    """Snap every facet to the exact rigid pose implied by the floor facet's
    orientation, and record how far that pose is from the commanded one."""
    global max_track_err_deg
    R = platform_rotation()
    for bid in plat_ids:
        s = O.bodies[bid].state
        s.pos = R * p0[bid]
        s.ori = R * q0[bid]
    q_cmd = quat_from_rotvec(tilt_rotvec((O.time + O.dt) - motion_t0))
    err = math.degrees(quat_angle(q_cmd * R.conjugate()))
    if stats_active():
        max_track_err_deg = max(max_track_err_deg, err)
    flush_all()

def point_velocity(b, c):
    s = b.state
    return s.vel + s.angVel.cross(c - s.pos)

def log_contacts():
    global friction_checked, friction_check_result
    t_motion = O.time - motion_t0
    active = stats_active()
    if active:
        stats['n'] += 1
    floor_seen = wall_seen = floor_slid = wall_slid = False
    bead_set = set(bead_ids)
    for bid in bead_ids:
        for i in O.bodies[bid].intrs():
            if not i.isReal:
                continue
            other = i.id1 if i.id2 == bid else i.id2
            if other in bead_set:
                if bid > other:
                    continue        # log each bead-bead pair once
                ctype = 'bead'
            elif other == floor_id:
                ctype = 'floor'
            elif other in wall_id_set:
                ctype = 'wall'
            else:
                continue
            fn = i.phys.normalForce.norm()
            fs = i.phys.shearForce.norm()
            tan_phi = i.phys.tangensOfFrictionAngle
            cap = tan_phi * fn
            if cap > 0:
                ratio = fs / cap
            else:
                ratio = 1.0 if fn > 0 else 0.0     # zero friction: always at the cap
            sliding = 1 if ratio >= SLIDE_RATIO else 0
            c = i.geom.contactPoint
            n = i.geom.normal
            v_rel = point_velocity(O.bodies[i.id2], c) - point_velocity(O.bodies[i.id1], c)
            v_t = v_rel - n * v_rel.dot(n)
            slip = v_t.norm()
            con_f.write(f"{O.time},{step_idx},{current_fric_deg},{t_motion},{i.id1},{i.id2},{ctype},"
                        f"{fn},{fs},{ratio},{sliding},{slip}\n")
            # once per step: does the floor contact really have the intended friction?
            if ctype == 'floor' and not friction_checked:
                expected = math.tan(math.radians(current_fric_deg))
                friction_check_result = 'ok' if abs(tan_phi - expected) < 1e-6 else \
                    f'MISMATCH(tan={tan_phi:.4f},expected={expected:.4f})'
                if friction_check_result != 'ok':
                    print(f"[step {step_idx}] WARNING friction check: {friction_check_result}")
                friction_checked = True
            if active and ctype == 'floor':
                floor_seen = True
                floor_slid = floor_slid or bool(sliding)
                stats['floor_slip_sum'] += slip
                stats['floor_slip_max'] = max(stats['floor_slip_max'], slip)
            elif active and ctype == 'wall':
                wall_seen = True
                wall_slid = wall_slid or bool(sliding)
    if active:
        stats['floor_n'] += floor_seen
        stats['floor_slide'] += floor_slid
        stats['wall_n'] += wall_seen
        stats['wall_slide'] += wall_slid

def log_positions():
    t_motion = O.time - motion_t0
    rv = tilt_rotvec(t_motion)
    Rinv = platform_rotation().conjugate()
    phase = (t_motion / PERIOD_S) % 1.0
    rmp = ramp_factor(t_motion)
    active = stats_active()
    for bid in bead_ids:
        s = O.bodies[bid].state
        p = s.pos
        pp = Rinv * p                     # position in the platform frame
        if active:
            stats['r_max'] = max(stats['r_max'], math.hypot(pp[0], pp[1]))
            h = pp[2] - R_BEAD            # bead bottom height above the floor
            stats['h_n'] += 1
            stats['h_sum'] += h
            stats['h_max'] = max(stats['h_max'], h)
        o = s.ori
        w = s.angVel
        pos_f.write(f"{O.time},{step_idx},{current_fric_deg},{t_motion},{phase},{rmp},"
                    f"{math.degrees(rv[0])},{math.degrees(rv[1])},{bid},"
                    f"{p[0]},{p[1]},{p[2]},{pp[0]},{pp[1]},{pp[2]},"
                    f"{o[0]},{o[1]},{o[2]},{o[3]},{w[0]},{w[1]},{w[2]}\n")

def finish_window():
    n = max(stats['n'], 1)
    fn_ = max(stats['floor_n'], 1)
    floor_frac = stats['floor_n'] / n
    floor_slide = stats['floor_slide'] / fn_         # fraction of FLOOR-CONTACT time spent sliding
    slip_mean = stats['floor_slip_sum'] / fn_
    wall_frac = stats['wall_n'] / n
    wall_slide = stats['wall_slide'] / n
    r_max_mm = stats['r_max'] * 1e3
    gap_mm = (RING_INNER_R - stats['r_max'] - R_BEAD) * 1e3
    h_mean_um = (stats['h_sum'] / max(stats['h_n'], 1)) * 1e6
    h_max_um = stats['h_max'] * 1e6
    sum_f.write(f"{step_idx},{current_fric_deg},{PREDICTED_DEG},{stats['n']},{floor_frac},{floor_slide},"
                f"{slip_mean},{stats['floor_slip_max']},{wall_frac},{wall_slide},{r_max_mm},"
                f"{gap_mm},{h_mean_um},{h_max_um},{max_track_err_deg},{friction_check_result}\n")
    flush_all()
    print(f"[t={O.time:7.1f}s] step {step_idx:2d} | alu fric {current_fric_deg:5.2f} deg | "
          f"floor contact {100*floor_frac:5.1f}% | sliding {100*floor_slide:5.1f}% of contact | "
          f"slip mean {slip_mean*1e3:7.3f} mm/s | wall {100*wall_frac:4.1f}% | "
          f"r_max {r_max_mm:5.1f} mm | h_max {h_max_um:7.2f} um | fric check {friction_check_result}")

def start_step(k):
    global step_idx, current_fric_deg, motion_t0, window_t0
    step_idx = k
    current_fric_deg = ALU_FRICTION_SCHEDULE_DEG[k] if SWEEP else ALU_FRICTION_FIXED_DEG
    set_alu_friction(current_fric_deg)
    erase_beads()            # removes their contacts too; new contacts pick up the new friction
    level_platform()
    motion_t0 = O.time
    window_t0 = O.time
    spawn_beads()
    reset_stats()

def step_manager():
    global window_t0
    if O.time - window_t0 < STEP_DURATION_S:
        return
    finish_window()
    if SWEEP:
        if step_idx + 1 >= len(ALU_FRICTION_SCHEDULE_DEG):
            print("\n--- SWEEP COMPLETE ---")
            flush_all()
            O.pause()
            return
        start_step(step_idx + 1)
    else:
        window_t0 = O.time     # forever mode: just start a new stats window
        reset_stats()

# =====================================================================
# Engines
# =====================================================================
rot_eng = RotationEngine(ids=plat_ids, rotationAxis=(0, 0, 1), angularVelocity=0.0,
                         rotateAroundZero=True, zeroPoint=(0, 0, 0))

O.engines = [
    ForceResetter(),
    InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Facet_Aabb()]),
    InteractionLoop(
        [Ig2_Sphere_Sphere_ScGeom(), Ig2_Facet_Sphere_ScGeom()],
        [ip2],
        [Law2_ScGeom_MindlinPhys_Mindlin()]
    ),
    PyRunner(command='log_contacts()', virtPeriod=1.0 / CONTACT_LOG_HZ),
    NewtonIntegrator(gravity=(0, 0, -9.81), damping=GLOBAL_DAMPING),
    PyRunner(command='platform_update()', iterPeriod=KIN_UPDATE_EVERY),
    rot_eng,
    PyRunner(command='resync_platform()', virtPeriod=RESYNC_EVERY_S),
    PyRunner(command='log_positions()', virtPeriod=1.0 / POS_LOG_HZ),
    PyRunner(command='step_manager()', iterPeriod=100),
]
if RECORD_VTK:
    O.engines += [VTKRecorder(fileName=os.path.join(OUT_DIR, 'vtk', 'plate_'),
                              recorders=['spheres', 'facets', 'velocity', 'id'],
                              virtPeriod=1.0 / VTK_HZ)]

# =====================================================================
# Start
# =====================================================================
start_step(0)
O.dt = DT_SAFETY * PWaveTimeStep()

print("\n================ STEWART SLIP TEST (v2) ================")
print(f" beads: {N_BEADS} PTFE (density {BEAD_DENSITY}, bead-bead mu {BEAD_FRICTION_MU})")
print(f" pattern: {PATTERN}   peak tilt: {TILT_DEG} deg   period: {PERIOD_S} s")
print(f" contact: Hertz-Mindlin, restitution {RESTITUTION} via {DAMPING_MODE}")
print(f" friction per pair: {'yes (MatchMaker)' if FRICTION_PER_PAIR else 'NO - fallback to min() rule'}")
print(f" predicted no-skid friction angle: {PREDICTED_DEG:.2f} deg")
print(f" estimated bead swing from center: {EST_SWING*1e3:.0f} mm (room to wall: {EST_WALL_ROOM*1e3:.0f} mm)")
if EST_SWING > 0.8 * EST_WALL_ROOM:
    print(" WARNING: bead will likely reach the wall at this PERIOD_S / TILT_DEG - "
          "this becomes a wall test, not a floor test.")
print(f" ramp per step (not in summary stats): {RAMP_CYCLES * PERIOD_S:.1f} s of {STEP_DURATION_S:.0f} s")
if SWEEP:
    total = len(ALU_FRICTION_SCHEDULE_DEG) * STEP_DURATION_S
    print(f" sweep: {len(ALU_FRICTION_SCHEDULE_DEG)} steps x {STEP_DURATION_S:.0f} s = {total/60:.1f} sim-min")
else:
    print(f" fixed alu friction: {ALU_FRICTION_FIXED_DEG} deg, runs until stopped")
print(f" dt = {O.dt:.3e} s")
print("========================================================\n")

try:
    from yade import qt
    qt.View()
except Exception:
    pass

# =====================================================================
# Things to check on first run (untested script):
#  1. Startup printout: 'friction per pair' should say yes, and the damping
#     mode should be 'en/es MatchMaker'. If not, tell Claude what it printed.
#  2. Every summary line: 'fric check ok'. A MISMATCH means the contact did not
#     get the intended friction - stop and fix before trusting anything.
#  3. h_max should be a few microns at most (bead resting, not hopping), and
#     floor contact should be ~100%. If h_max is still tens-hundreds of um,
#     the bounce is not damped.
#  4. wall contact 0%, r_max ~50 mm, track err tiny (~1e-6 deg).
#  5. Then the real result: 'sliding % of contact' high below ~2.9 deg,
#     ~0 above.
#  'bead_id' changes every sweep step (new bead each time) - group by 'step'.
# =====================================================================