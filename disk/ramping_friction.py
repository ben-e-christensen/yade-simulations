from yade import pack, geom, qt, ymport, export
import math
import os

# --- 0. Global Variables & Setup ---
RPM = 26.0  
angular_vel = RPM * (math.pi / 30.0)

# The friction sweep parameters
friction_angles_deg = [20.0 + (i * 2.5) for i in range(13)] # [20.0, 22.5, ..., 50.0]
current_idx = 0
current_deg = friction_angles_deg[current_idx]
phase_duration_virtual = 30.0 # 30 virtual (simulation) seconds per friction step

sim_initialized = False
phase_start_time = 0.0 
global_start_time = 0.0

# Create output directory for VTK files
if not os.path.exists('vtk_output'):
    os.makedirs('vtk_output')

# Initialize CSV for positions (100Hz)
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

# Initialize CSV and VTK Exporter for interactions (20Hz)
vtk_interaction_exporter = export.VTKExporter('vtk_output/chains_')

with open('force_network.csv', 'w') as f:
    f.write("sim_time,id1,id2,normal_force\n")

def export_force_chains():
    # 1. Export 3D lines to VTK safely
    vtk_interaction_exporter.exportInteractions(what=dict(forceN='i.phys.normalForce.norm()'))
    
    # 2. Export raw force numbers to CSV
    with open('force_network.csv', 'a') as f:
        for i in O.interactions:
            if i.isReal:
                # Ensure we only record bead-to-bead forces, ignoring the drum walls
                if isinstance(O.bodies[i.id1].shape, Sphere) and isinstance(O.bodies[i.id2].shape, Sphere):
                    fn = i.phys.normalForce.norm()
                    f.write(f"{O.time},{i.id1},{i.id2},{fn}\n")


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

# --- 3. Particles (Hexagonal Grid Packing, PRE-TILTED) ---
R_bead = 1.6e-3
target_beads = 500
count = 0

spacing = (R_bead * 2.0) * 1.01 
row_height = spacing * math.sqrt(3) / 2.0 
grid_size = int((R_base * 2) / spacing) + 2

# Pre-tilt the bed to match the starting 20-degree friction angle
tilt_angle_deg = 20.0 
tilt_rad = math.radians(tilt_angle_deg)

for row in range(-grid_size, grid_size):
    if count >= target_beads: break
        
    for col in range(-grid_size, grid_size):
        if count >= target_beads: break
            
        x = col * spacing
        y = row * row_height
        
        # Offset every other row for hexagonal packing
        if row % 2 != 0:
            x += spacing / 2.0
            
        # Apply the 2D rotation matrix to tilt the coordinates
        x_rot = x * math.cos(tilt_rad) - y * math.sin(tilt_rad)
        y_rot = x * math.sin(tilt_rad) + y * math.cos(tilt_rad)
            
        # Safely check if the rotated coordinate is inside the drum
        if math.sqrt(x_rot**2 + y_rot**2) < (R_base - 0.003):
            s = sphere((x_rot, y_rot, gap/2), R_bead, material=sphere_mat)
            s.shape.color = (0.0, 0.2, 1.0) # Start Blue for 20 degrees
            O.bodies.append(s)
            count += 1

print(f"\n--- SUCCESS: Placed {count} beads, pre-tilted to {tilt_angle_deg} degrees. ---\n")

# --- 4. The Parameter Sweeper Engine (SIMULATION TIME) ---
def friction_stepper():
    global current_idx, current_deg, phase_start_time, global_start_time, sim_initialized
    
    # Wait until the simulation actually starts running to begin the clock
    if not sim_initialized:
        phase_start_time = O.time
        global_start_time = O.time
        sim_initialized = True
        return
        
    # Step up the friction if 30 simulation seconds have passed
    if (O.time - phase_start_time) >= phase_duration_virtual:
        current_idx += 1
        
        # Stop condition
        if current_idx >= len(friction_angles_deg):
            total_time = O.time - global_start_time
            print(f"\n--- SWEEP COMPLETE! Ran up to 50 degrees. Total sim time: {total_time:.1f}s ---")
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
                
        # 3. Update Colors (Scales from Blue @ 20 deg to Red @ 50 deg)
        color_factor = (current_deg - 20.0) / 30.0 
        new_color = (color_factor, 0.2, 1.0 - color_factor)
        
        for b in O.bodies:
            if isinstance(b.shape, Sphere):
                b.shape.color = new_color
                
        print(f"[Sim Time: {O.time:.1f}s] Stepping up friction -> Now at {current_deg} Degrees")
        
        # Reset the phase timer using Yade's internal clock
        phase_start_time = O.time

# --- 5. Engines ---
O.engines = [
    ForceResetter(),
    InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Facet_Aabb()]),
    InteractionLoop(
        [Ig2_Sphere_Sphere_ScGeom(), Ig2_Facet_Sphere_ScGeom()],
        [Ip2_FrictMat_FrictMat_FrictPhys()],
        [Law2_ScGeom_FrictPhys_CundallStrack()]
    ),
    
    # Dynamic Time Stepper
    GlobalStiffnessTimeStepper(active=1, timeStepUpdateInterval=100, timestepSafetyCoefficient=0.5),
    
    NewtonIntegrator(gravity=(0, -9.81, 0), damping=0.2),
    
    RotationEngine(ids=rotating_ids, 
                   rotationAxis=(0, 0, 1), 
                   rotateAroundZero=True, 
                   angularVelocity=angular_vel),
                   
    # Sweeper checks clock every 100 iterations
    PyRunner(command='friction_stepper()', iterPeriod=100),               
    
    # Position CSV Data tracker set to 100Hz (0.01 virtPeriod)
    PyRunner(command='export_positions()', virtPeriod=0.01),
    
    # C++ VTK Engine 1: Records spheres, colors, and velocities at 20Hz (0.05 virtPeriod)
    VTKRecorder(fileName='vtk_output/drum_', recorders=['spheres', 'colors', 'velocity', 'id'], virtPeriod=0.05),
    
    # Python Exporter 2: Records interaction force chains (VTK + CSV) at 20Hz (0.05 virtPeriod) safely!
    PyRunner(command='export_force_chains()', virtPeriod=0.05)
]

# --- 6. Execution Setup ---
qt.View()
qt.center()