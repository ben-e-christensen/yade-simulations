from yade import pack, geom, ymport, qt
import math
import os

# --- 0. Output Setup (VTK & CSV) ---
# Create a folder for the Paraview VTK files so they don't clutter your main directory
os.makedirs('vtk_data', exist_ok=True)

# Initialize our CSV file and write the headers
csv_filename = "bead_tracking.csv"
with open(csv_filename, 'w') as f:
    f.write("time,angle_deg,bead_y,bead_z,dist_top,dist_bot,force_mag,force_constant\n")

# --- 1. Materials ---
fric_angle = math.atan(0.4)
sphere_mat = O.materials.append(FrictMat(young=1e7, poisson=0.3, density=1190, frictionAngle=fric_angle))
wall_mat = O.materials.append(FrictMat(young=1e7, poisson=0.3, frictionAngle=fric_angle))

# --- 2. Geometry: The See-Through Tube ---
cylinder_facets = ymport.stl('tube.stl', material=wall_mat, scale=0.001)

for f in cylinder_facets:
    f.shape.wire = True
    f.shape.color = (0.7, 0.7, 0.7) 

cylinder_ids = O.bodies.append(cylinder_facets)

# --- 3. The Single Bead ---
R_bead = 1.6e-3
bead_id = O.bodies.append(sphere((0, 0, 0), R_bead, material=sphere_mat))

# --- 4. The Horizon Line ---
horizon_line = geom.facetBox(center=(-0.05, 0, 0), extents=(0.0005, 0.0005, 0.1), material=wall_mat)
for f in horizon_line:
    f.shape.color = (0.2, 0.8, 0.2) 
    f.shape.wire = False
O.bodies.append(horizon_line)

# --- 5. Electrostatics & Stickiness Detector ---
force_constant = 1e-10 
target_rpm = 1.0
omega = target_rpm * (2 * math.pi / 60.0) 

bead_fell = False
full_rotation_time = (60.0 / target_rpm)

def apply_electrostatics():
    global bead_fell
    
    current_angle = O.time * omega
    bead_pos = O.bodies[bead_id].state.pos
    
    # Calculate tube orientation
    axis_y = -math.sin(current_angle)
    axis_z = math.cos(current_angle)
    d_axis = bead_pos[1] * axis_y + bead_pos[2] * axis_z
    
    # Distance to phantom end caps
    L_half = 0.0762 
    dist_top = (L_half - d_axis) - R_bead
    dist_bot = (L_half + d_axis) - R_bead
    
    min_dist = 1e-6 
    dist_top = max(dist_top, min_dist)
    dist_bot = max(dist_bot, min_dist)
    
    # Detect if the bead falls (starts checking after 1/4 rotation)
    if O.time > (full_rotation_time * 0.25):
        if dist_top > 0.005 and dist_bot > 0.005:
            bead_fell = True
            
    # Pause and report at exactly 1 full rotation
    if O.time >= full_rotation_time:
        if bead_fell:
            print(f"Force {force_constant}: The bead FELL.")
        else:
            print(f"Force {force_constant}: The bead STUCK!")
        O.pause() 
    
    # Apply inverse square force
    f_top_mag = force_constant / (dist_top**2)
    f_bot_mag = force_constant / (dist_bot**2)
    
    f_y = (axis_y * f_top_mag) - (axis_y * f_bot_mag)
    f_z = (axis_z * f_top_mag) - (axis_z * f_bot_mag)
    
    O.forces.addF(bead_id, (0, f_y, f_z))
    
    # Log data to CSV every 100th of a second
    if O.iter % 100 == 0:
        with open(csv_filename, 'a') as f:
            f.write(f"{O.time:.4f},{math.degrees(current_angle):.2f},{bead_pos[1]:.6f},{bead_pos[2]:.6f},{dist_top:.6f},{dist_bot:.6f},{max(f_top_mag, f_bot_mag):.6e},{force_constant:.2e}\n")

# --- 6. Engines ---
O.engines = [
    ForceResetter(),
    InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Facet_Aabb()]),
    InteractionLoop(
        [Ig2_Sphere_Sphere_ScGeom(), Ig2_Facet_Sphere_ScGeom()],
        [Ip2_FrictMat_FrictMat_FrictPhys()],
        [Law2_ScGeom_FrictPhys_CundallStrack()]
    ),
    NewtonIntegrator(gravity=(0, -9.81, 0), damping=0.2),
    
    RotationEngine(ids=cylinder_ids, rotationAxis=(1, 0, 0), rotateAroundZero=True, angularVelocity=omega),
    
    # Python logic & tracking
    PyRunner(command='apply_electrostatics()', iterPeriod=1),
    
    # VTK Exporter for Paraview (Saves a frame every 500 iterations)
    VTKRecorder(fileName='vtk_data/sim_', recorders=['spheres', 'facets', 'colors'], iterPeriod=500)
]

# --- 7. Simulation Timestep ---
O.dt = .5 * PWaveTimeStep()

# --- 8. UI and View Automation ---
v = qt.View()
v.axes = False  

# Snap the camera to look down the X-axis
v.viewDir = (1, 0, 0)
v.upVector = (0, 1, 0) 

qt.center()