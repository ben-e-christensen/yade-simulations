from yade import pack, geom, qt, ymport
import math
import os
import time

# --- 0. Global Variables & Setup ---
RPM = 26.0  
angular_vel = RPM * (math.pi / 30.0)

# The friction sweep parameters
friction_angles_deg = [20.0 + (i * 2.5) for i in range(13)] # [20.0, 22.5, ..., 50.0]
current_idx = 0
current_deg = friction_angles_deg[current_idx]
phase_duration = 30.0 # Real-world seconds per friction step

sim_initialized = False
phase_start_time = 0.0 
global_start_time = 0.0

if not os.path.exists('vtk_output'):
    os.makedirs('vtk_output')

with open('repose_data.csv', 'w') as f:
    f.write("sim_time,fric_deg,bead_id,x_pos,y_pos,z_pos,rot_w,rot_x,rot_y,rot_z,ang_vel_x,ang_vel_y,ang_vel_z\n")

def export_positions():
    with open('repose_data.csv', 'a') as f:
        for b in O.bodies:
            if isinstance(b.shape, Sphere):
                pos = b.state.pos
                ori = b.state.ori
                vel = b.state.angVel
                
                f.write(f"{O.time},{current_deg},{b.id},{pos[0]},{pos[1]},{pos[2]},"
                        f"{ori[0]},{ori[1]},{ori[2]},{ori[3]},"
                        f"{vel[0]},{vel[1]},{vel[2]}\n")

# --- 1. Materials (Starting at 20 Degrees) ---
start_rad = math.radians(current_deg)

sphere_mat = O.materials.append(FrictMat(young=1e7, poisson=0.3, density=1190, frictionAngle=start_rad))
wall_mat = O.materials.append(FrictMat(young=1e7, poisson=0.3, frictionAngle=start_rad))

# --- 2. Geometry ---
R_base = 81.5e-3  
drum_h = 3.2e-3   
gap = 0.2e-3      

drum_facets = ymport.stl('serrated_drum.stl', material=wall_mat, scale=0.001)
drum_ids = O.bodies.append(drum_facets)

faceplates = geom.facetBox(
    center=(0, 0, gap/2), 
    extents=(R_base * 1.5, R_base * 1.5, (drum_h + gap) / 2), 
    material=wall_mat, 
    wallMask=48 
)
faceplate_ids = O.bodies.append(faceplates)
rotating_ids = drum_ids + faceplate_ids

# --- 3. Particles (Hexagonal Grid Packing) ---
R_bead = 1.6e-3
target_beads = 500
count = 0

spacing = (R_bead * 2.0) * 1.01 
row_height = spacing * math.sqrt(3) / 2.0 
grid_size = int((R_base * 2) / spacing) + 2

for row in range(-grid_size, grid_size):
    if count >= target_beads: break
        
    for col in range(-grid_size, grid_size):
        if count >= target_beads: break
            
        x = col * spacing
        y = row * row_height
        
        if row % 2 != 0:
            x += spacing / 2.0
            
        if math.sqrt(x**2 + y**2) < (R_base - 0.003):
            # Assigning the starting Blue color for 20 degrees
            s = sphere((x, y, gap/2), R_bead, material=sphere_mat)
            s.shape.color = (0.0, 0.2, 1.0) 
            O.bodies.append(s)
            count += 1

print(f"\n--- SUCCESS: Placed {count} beads. Starting sweep at {current_deg} degrees. ---\n")

# --- 4. The Parameter Sweeper Engine ---
def friction_stepper():
    global current_idx, current_deg, phase_start_time, global_start_time, sim_initialized
    
    # Do not start the timer until the simulation actually begins running
    if not sim_initialized:
        phase_start_time = time.time()
        global_start_time = time.time()
        sim_initialized = True
        return
        
    current_time = time.time()
    
    # If 30 real-world seconds have passed
    if (current_time - phase_start_time) >= phase_duration:
        current_idx += 1
        
        # Stop condition
        if current_idx >= len(friction_angles_deg):
            total_time = current_time - global_start_time
            print(f"\n--- SWEEP COMPLETE! Ran up to 50 degrees. Total real time: {total_time:.1f}s ---")
            O.pause()
            return
            
        # Update Variables
        current_deg = friction_angles_deg[current_idx]
        new_rad = math.radians(current_deg)
        
        # 1. Update base materials for new contacts
        O.materials[sphere_mat].frictionAngle = new_rad
        O.materials[wall_mat].frictionAngle = new_rad
        
        # 2. Update existing active contacts (CRITICAL)
        for i in O.interactions:
            if i.isReal:
                i.phys.frictionAngle = new_rad
                
        # 3. Update Colors (Visual Indicator for UI and VTK)
        # Scales from 0.0 (Blue @ 20 deg) to 1.0 (Red @ 50 deg)
        color_factor = (current_deg - 20.0) / 30.0 
        new_color = (color_factor, 0.2, 1.0 - color_factor)
        
        for b in O.bodies:
            if isinstance(b.shape, Sphere):
                b.shape.color = new_color
                
        print(f"[{current_time - global_start_time:.1f}s] Stepping up friction -> Now at {current_deg} Degrees")
        
        # Reset the timer for the next phase
        phase_start_time = time.time()

# --- 5. Engines ---
O.engines = [
    ForceResetter(),
    InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Facet_Aabb()]),
    InteractionLoop(
        [Ig2_Sphere_Sphere_ScGeom(), Ig2_Facet_Sphere_ScGeom()],
        [Ip2_FrictMat_FrictMat_FrictPhys()],
        [Law2_ScGeom_FrictPhys_CundallStrack()]
    ),
    NewtonIntegrator(gravity=(0, -9.81, 0), damping=0.2),
    
    RotationEngine(ids=rotating_ids, 
                   rotationAxis=(0, 0, 1), 
                   rotateAroundZero=True, 
                   angularVelocity=angular_vel),
                   
    # Run our sweeper every 100 iterations to check the clock without lagging the CPU
    PyRunner(command='friction_stepper()', iterPeriod=100),
                   
    PyRunner(command='export_positions()', virtPeriod=0.01),
    VTKRecorder(fileName='vtk_output/drum_', recorders=['spheres', 'facets', 'colors', 'velocity'], virtPeriod=0.01)
]

# --- 6. Execution Setup ---
O.dt = .5 * PWaveTimeStep()
qt.View()
qt.center()