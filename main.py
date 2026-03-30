from yade import pack, geom, qt, ymport
import math
import os

# --- 0. Setup ---
if not os.path.exists('vtk_output'):
    os.makedirs('vtk_output')

with open('repose_data.csv', 'w') as f:
    f.write("sim_time,bead_id,x_pos,y_pos\n")

def export_positions():
    with open('repose_data.csv', 'a') as f:
        for b in O.bodies:
            if isinstance(b.shape, Sphere):
                f.write(f"{O.time},{b.id},{b.state.pos[0]},{b.state.pos[1]}\n")

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
# shift the center by half the gap (0.1mm) to make one side flush and one side gapped.
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

# We center the cloud generation slightly toward the gap side 
# to ensure no beads are "pre-clipping" through the flush wall.
sp.makeCloud(minCorner=(-R_base/2, -R_base/2, -drum_h/2 + 0.0001), 
             maxCorner=(R_base/2, R_base/2, drum_h/2 + gap - 0.0001), 
             rMean=R_bead, rRelFuzz=0.0, num=500)
sp.toSimulation(material=sphere_mat)

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