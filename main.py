from yade import pack, geom, qt, ymport
import math
import os

# --- 0. Setup ---
if not os.path.exists('vtk_output'):
    os.makedirs('vtk_output')

# UPDATED: Added headers for rotation (quaternions) and angular velocity
with open('repose_data.csv', 'w') as f:
    f.write("sim_time,bead_id,x_pos,y_pos,rot_w,rot_x,rot_y,rot_z,ang_vel_x,ang_vel_y,ang_vel_z\n")

def export_positions():
    with open('repose_data.csv', 'a') as f:
        for b in O.bodies:
            if isinstance(b.shape, Sphere):
                # Grab state vectors
                pos = b.state.pos
                ori = b.state.ori
                vel = b.state.angVel
                
                # Write all 11 data points to the CSV
                f.write(f"{O.time},{b.id},{pos[0]},{pos[1]},"
                        f"{ori[0]},{ori[1]},{ori[2]},{ori[3]},"
                        f"{vel[0]},{vel[1]},{vel[2]}\n")

# --- 1. Materials ---
fric_angle = math.atan(0.4)
sphere_mat = O.materials.append(FrictMat(young=1e7, poisson=0.3, density=1190, frictionAngle=fric_angle))
wall_mat = O.materials.append(FrictMat(young=1e7, poisson=0.3, frictionAngle=fric_angle))

# --- 2. Geometry ---
R_base = 81.5e-3  
drum_h = 3.2e-3   # 3.2mm
gap = 0.2e-3      # 0.2mm gap on ONE side

# Load the drum
drum_facets = ymport.stl('serrated_drum.stl', material=wall_mat, scale=0.001)
drum_ids = O.bodies.append(drum_facets)

# ASYMMETRIC FACEPLATES:
# We shift the center by half the gap (0.1mm) to make one side flush and one side gapped.
faceplates = geom.facetBox(
    center=(0, 0, gap/2), 
    extents=(R_base * 1.5, R_base * 1.5, (drum_h + gap) / 2), 
    material=wall_mat, 
    wallMask=48 
)
faceplate_ids = O.bodies.append(faceplates)
rotating_ids = drum_ids + faceplate_ids

# --- 3. Particles ---
R_bead = 1.6e-3
sp = pack.SpherePack()

# 1. Generate a 'surplus' of beads in a box (e.g., 800)
# This gives the random generator more options so it doesn't get stuck at 387
sp.makeCloud(minCorner=(-R_base, -R_base, -drum_h/2), 
             maxCorner=(R_base, R_base, drum_h/2 + gap), 
             rMean=R_bead, rRelFuzz=0.0, num=800)

# 2. Manually filter them into the simulation
count = 0
for center, radius in sp:
    x, y, z = center
    # Distance formula: Is the bead's center within the drum's radius?
    # We subtract 2mm (0.002) as a safety buffer so they don't touch the teeth on frame 1
    if math.sqrt(x**2 + y**2) < (R_base - 0.002):
        if count < 500:
            O.bodies.append(sphere(center, radius, material=sphere_mat))
            count += 1

print(f"Successfully added {count} beads inside the drum geometry.")

# --- 4. Engines ---
O.engines = [
    ForceResetter(),
    InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Facet_Aabb()]),
    InteractionLoop(
        [Ig2_Sphere_Sphere_ScGeom(), Ig2_Facet_Sphere_ScGeom()],
        [Ip2_FrictMat_FrictMat_FrictPhys()],
        [Law2_ScGeom_FrictPhys_CundallStrack()]
    ),
    NewtonIntegrator(gravity=(0, -9.81, 0), damping=0.2),
    
    # Both drum and both plates spin together
    RotationEngine(ids=rotating_ids, 
                   rotationAxis=(0, 0, 1), 
                   rotateAroundZero=True, 
                   angularVelocity=2.72271363311),
                   
    PyRunner(command='export_positions()', virtPeriod=0.01),
    VTKRecorder(fileName='vtk_output/drum_', recorders=['spheres', 'facets', 'colors'], virtPeriod=0.01)
]

# --- 5. Setup ---
O.dt = .5 * PWaveTimeStep()
qt.View()
qt.center()