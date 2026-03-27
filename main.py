from yade import pack, geom, qt, ymport
import math
import os

# --- 0. Data Tracking & Folder Setup ---
# Create the folder for VTK files if it doesn't exist to prevent crashes
if not os.path.exists('vtk_output'):
    os.makedirs('vtk_output')

# Create a fresh CSV file and write the column headers
with open('repose_data.csv', 'w') as f:
    f.write("sim_time,bead_id,x_pos,y_pos\n")

# This function will be called repeatedly by the PyRunner engine (100Hz)
def export_positions():
    with open('repose_data.csv', 'a') as f:
        for b in O.bodies:
            # We only care about tracking the beads (Spheres)
            if isinstance(b.shape, Sphere):
                f.write(f"{O.time},{b.id},{b.state.pos[0]},{b.state.pos[1]}\n")

# --- 1. Materials ---
# Set coefficient of friction to exactly 0.4 using math.atan()
fric_angle = math.atan(0.4)
sphere_mat = O.materials.append(FrictMat(young=1e7, poisson=0.3, density=1190, frictionAngle=fric_angle))
wall_mat = O.materials.append(FrictMat(young=1e7, poisson=0.3, frictionAngle=fric_angle))

# --- 2. Geometry (The Drum) ---
R_base = 81.5e-3  # OpenSCAD inner radius
length = 3.2e-3   # Matches OpenSCAD height

# Load the STL and capture the specific IDs of the teeth
drum_facets = ymport.stl('serrated_drum.stl', material=wall_mat, scale=0.001)
drum_ids = O.bodies.append(drum_facets)

# Add the glass faceplates to trap the beads in the Z-axis
faceplates = geom.facetBox(
    center=(0, 0, 0), 
    extents=(R_base * 1.5, R_base * 1.5, length / 2), 
    material=wall_mat, 
    wallMask=48 
)

# Make the faceplates see-through wireframes
for plate in faceplates:
    plate.shape.wire = True  

O.bodies.append(faceplates)

# --- 3. Particles (The Beads) ---
R_bead = 1.6e-3

# Generate the 500 beads inside the 3.2mm slice
sp = pack.SpherePack()
sp.makeCloud(minCorner=(-R_base/2, -R_base/2, -length/2 + R_bead), 
             maxCorner=(R_base/2, R_base/2, length/2 - R_bead), 
             rMean=R_bead, rRelFuzz=0.0, num=500)
sp.toSimulation(material=sphere_mat)

# --- 4. The Physics Engines ---
O.engines = [
    ForceResetter(),
    
    # Broad-phase collision detection
    InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Facet_Aabb()]),
    
    # Narrow-phase collision and force calculation
    InteractionLoop(
        [Ig2_Sphere_Sphere_ScGeom(), Ig2_Facet_Sphere_ScGeom()],
        [Ip2_FrictMat_FrictMat_FrictPhys()],
        [Law2_ScGeom_FrictPhys_CundallStrack()]
    ),
    
    # Applies constant gravity
    NewtonIntegrator(gravity=(0, -9.81, 0), damping=0.2),
    
    # Rotates ONLY the drum teeth (leaves faceplates stationary)
    # SPEED IS SET HERE WITH 'angularVelocity' (in radians per second)
    RotationEngine(ids=drum_ids, 
                   rotationAxis=(0, 0, 1), 
                   rotateAroundZero=True, 
                   angularVelocity=2.72271363311),
                   
    # --- NEW: Data Tracking Engines ---
    # 1. Runs the CSV export function at 100Hz (every 0.01 simulated seconds)
    PyRunner(command='export_positions()', virtPeriod=0.01),
    
    # 2. Exports 3D snapshots for ParaView at 100Hz
    VTKRecorder(fileName='vtk_output/drum_', recorders=['spheres', 'facets', 'colors'], virtPeriod=0.01)
]

# --- 5. Simulation Setup ---
O.dt = .5 * PWaveTimeStep()

# Open the 3D graphical interface automatically and snap camera to center
qt.View()
qt.center()